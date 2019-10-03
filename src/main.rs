#![no_std]
#![no_main]

use panic_semihosting as _;

use core::fmt::Write;
use cortex_m::interrupt::{free as critical_section, CriticalSection};
use cortex_m::peripheral::NVIC;
use cortex_m::asm::{delay, wfi};
use cortex_m_rt::entry;
use heapless::{Vec, String, consts::*};
use stm32f1xx_hal::{prelude::*, stm32};
use stm32f1xx_hal::stm32::interrupt;
use stm32_usbd::UsbBusType;
use usb_device::prelude::*;
use usb_device::bus::UsbBusAllocator;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

mod timer_dma;
mod pulse_input;
mod sirc;
mod mutex;

use timer_dma::TimerDma;
use pulse_input::*;
use sirc::SircTransmitter;
use mutex::Mutex;

#[allow(unused)]
#[macro_export]
macro_rules! println {
    ($($arg:tt)*) => {
        critical_section(|_| {
            let itm = unsafe { &mut *cortex_m::peripheral::ITM::ptr() };
            cortex_m::iprintln!(&mut itm.stim[0], $($arg)*);
        });
    }
}

static mut DMA_BUF: Option<[vcell::VolatileCell<u16>; 70 * 2]> = None;
static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;
static USB_DEVICE: Mutex<UsbDevice<'static, UsbBusType>> = Mutex::uninit();
static SERIAL: Mutex<SerialPort<'static, UsbBusType>> = Mutex::uninit();
static PULSE_INPUT: Mutex<PulseInput<stm32::TIM3>> = Mutex::uninit();
static SIRC: Mutex<SircTransmitter> = Mutex::uninit();

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(48.mhz())
        .pclk1(24.mhz())
        .freeze(&mut flash.acr);

    assert!(clocks.usbclk_valid());

    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

    let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
    usb_dp.set_low();
    delay(clocks.sysclk().0 / 100);

    let usb_dm = gpioa.pa11;
    let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    led.set_high();

    let dma1 = dp.DMA1.split(&mut rcc.ahb);

    cortex_m::asm::delay(48 * 1000 * 100);

    let dma_buf;
    let usb_bus;

    unsafe {
        DMA_BUF = Some(core::mem::MaybeUninit::zeroed().assume_init());
        dma_buf = DMA_BUF.as_mut().unwrap();

        USB_BUS = Some(UsbBusType::new(dp.USB, (usb_dm, usb_dp)));
        usb_bus = USB_BUS.as_ref().unwrap()
    }

    let mut pulse_input = PulseInput::new(
        dp.TIM3, dma1.6, gpioa.pa6,
        48, 7000, dma_buf,
        &mut rcc.apb1);

    pulse_input.listen();

    PULSE_INPUT.init(pulse_input);

    SIRC.init(
        SircTransmitter::new(
            dp.TIM1, dma1.5, gpioa.pa8.into_alternate_push_pull(&mut gpioa.crh),
            48,
            &mut rcc.apb2));

    SERIAL.init(SerialPort::new(usb_bus));

    USB_DEVICE.init(
        UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("virkkunen.net")
            .product("RimoIO2")
            .serial_number("rimoio2")
            .device_class(USB_CLASS_CDC)
            .build());

    unsafe {
        NVIC::unmask(stm32::Interrupt::TIM3);
        NVIC::unmask(stm32::Interrupt::DMA1_CHANNEL6);
        NVIC::unmask(stm32::Interrupt::USB_LP_CAN_RX0);
    }

    loop {
        wfi();
    }
}

#[interrupt]
fn TIM3() {
    critical_section(|cs| {
        let mut pulse_input = PULSE_INPUT.lock(cs);

        if let Some(mut pulses) = pulse_input.tim_isr() {
            if let Some(cmd) = parse_pulses(&mut pulses) {
                let (ctype, id) = match cmd {
                    RemoteCommand::Keyfob(id) => ('K', id),
                    RemoteCommand::Nexa(id) => ('N', id),
                };

                let mut msg: String<U32> = String::new();
                write!(msg, "{}{:08X}\r\n", ctype, id).ok();

                let mut serial = SERIAL.lock(cs);
                serial.write(&msg.as_bytes()).ok();
            }
        }; // yes, this semicolon is required
    });
}

#[interrupt]
fn DMA1_CHANNEL6() {
    critical_section(|cs| {
        let mut pulse_input = PULSE_INPUT.lock(cs);

        pulse_input.dma_isr();
    });
}

#[interrupt]
fn USB_LP_CAN_RX0() {
    static RX_BUF: Mutex<Vec<u8, U32>> = Mutex::new(Vec(heapless::i::Vec::new())); // wat

    critical_section(|cs| {
        let mut usb_dev = USB_DEVICE.lock(cs);
        let mut serial = SERIAL.lock(cs);

        if usb_dev.poll(&mut [&mut *serial]) {
            let mut rx_buf = RX_BUF.lock(cs);

            let mut c = 0;
            while let Ok(_) = serial.read(core::slice::from_mut(&mut c)) {
                if c == b'\r' || c == b'\n' {
                    if rx_buf.len() > 0 {
                        process_command(cs, rx_buf.as_ref());
                    }

                    rx_buf.clear();
                } else {
                    // this will fail if the line is too long, but we don't care
                    rx_buf.push(c).ok();
                }
            }
        }
    });
}

fn process_command(cs: &CriticalSection, cmd: &[u8]) {
    if cmd[0] == b'S' && cmd.len() >= 5 {
        let s = unsafe { core::str::from_utf8_unchecked(cmd) };
        if let (Ok(address), Ok(command)) =
            (u8::from_str_radix(&s[1..3], 16), u8::from_str_radix(&s[3..5], 16))
        {
            let mut sirc = SIRC.lock(cs);
            sirc.send(address, command);
        }
    }
}

enum RemoteCommand {
    Keyfob(u32),
    Nexa(u32),
}

fn parse_pulses<T: TimerDma>(pulses: &mut PulseTrain<T>) -> Option<RemoteCommand> {
    let mut pulses = pulses.iter();

    if let Some(first) = pulses.next() {
        if first.0 > 1100 && first.0 < 1300 {
            parse_keyfob(&mut pulses).map(RemoteCommand::Keyfob)
        } else if first.0 > 2500 && first.0 < 3000 && first.1 > 200 && first.1 < 400 {
            parse_nexa(&mut pulses).map(RemoteCommand::Nexa)
        } else {
            None
        }
    } else {
        None
    }
}

fn parse_keyfob(pulses: &mut Pulses) -> Option<u32> {
    let mut id = 0u32;

    for pulse in pulses.take(24) {
        let bit = match pulse {
            (1100..=1300, 200..=500) => 0,
            (1100..=1300, 800..=1100) => 1,
            _ => {
                //println!("  err {:?}", pulse);
                return None;
            }
        };

        id |= bit;
        id <<= 1;
    }

    return Some(id);

    /*let button = match id & 0x1f {
        0b10000 => 'B',
        0b01000 => 'D',
        0b00100 => 'A',
        0b00010 => 'C',
        _ => return,
    };

    let unit = (id >> 5) & 0x7ffff;

    if unit == 9099 {
        led(button == 'A');
    }*/
}

fn parse_nexa(pulses: &mut Pulses) -> Option<u32> {
    let mut pulses = pulses.take(64);

    let mut id = 0u32;

    while let Some(a) = pulses.next() {
        let b = match pulses.next() {
            Some(p) => p,
            None => return None,
        };

        if !(a.1 > 200 && a.1 < 400 && b.1 > 200 && b.1 < 400) {
            return None;
        }

        let bit = match (a.0, b.0) {
            (400..=600, 1400..=1600) => 0,
            (1400..=1600, 400..=600) => 1,
            _ => {
                //println!("  err {:?}", (a, b));
                return None;
            },
        };

        id <<= 1;
        id |= bit;
    }

    return Some(id);


    /*let unit = id & 0xf;
    let on = ((id >> 4) & 1) != 0;
    let house = (id >> 6) & 0x3fffffff;

    if house == 15626102 && unit == 10 {
        led(on);
    }*/

    //println!("  nexa house {} unit {} on {}", house, unit, on);
}
