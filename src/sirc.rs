use vcell::VolatileCell;
use stm32f1xx_hal::gpio::*;
use stm32f1xx_hal::stm32;
use stm32::TIM1;

const BITS: usize = 12;
const REPEATS: usize = 5;

static mut DMA_BUF: Option<[VolatileCell<u8>; 2 * 2 * (BITS + 2) * REPEATS + 2]> = None;

pub struct SircTransmitter;

impl SircTransmitter {
    /// prescaler should set timer clock to 1µs
    pub fn new(
        tim: TIM1,
        dma: stm32f1xx_hal::dma::dma1::C5,
        pin: stm32f1xx_hal::gpio::gpioa::PA8<Alternate<PushPull>>,
        prescaler: u16,
        enr: &mut stm32f1xx_hal::rcc::APB2) -> Self
    {
        let _ = (tim, dma, pin, enr);

        // Enable timer
        let dp = unsafe { stm32::Peripherals::steal() };
        dp.RCC.apb2enr.modify(|_, w| w.tim1en().set_bit());

        let mut si = SircTransmitter;

        si.init(prescaler);

        si
    }

    fn tim(&self) -> &stm32::tim1::RegisterBlock {
        unsafe { &*stm32::TIM1::ptr() }
    }

    fn dma(&self) -> &stm32::dma1::RegisterBlock {
        unsafe { &*stm32::DMA1::ptr() }
    }

    fn dma_ch(&self) -> &stm32::dma1::CH {
        &self.dma().ch5
    }

    fn init(&mut self, prescaler: u16) {
        unsafe { DMA_BUF = Some(core::mem::MaybeUninit::zeroed().assume_init()); }

        // Memory size 8 bits, peripheral size 16 bits, memory increment, memory to peripheral
        unsafe {
            self.dma_ch().cr.modify(|_, w| w
                .msize().bits(0b00)
                .psize().bits(0b01)
                .minc().set_bit()
                .dir().set_bit());
        }

        // Peripheral address (TIM1 burst DMA)
        self.dma_ch().par.write(|w| w.pa().bits(&self.tim().dmar as *const _ as *const u32 as u32));

        // Memory address
        let dma_buf = unsafe { DMA_BUF.as_ref().unwrap() as *const _ as *const u8 as u32 };
        self.dma_ch().mar.write(|w| w.ma().bits(dma_buf));

        // Only overflow generates events
        self.tim().cr1.modify(|_, w| w.urs().set_bit());

        unsafe {
            // Prescale timer
            self.tim().psc.modify(|_, w| w.psc().bits(prescaler));

            // PWM mode 1 - high as long as CNT < CCR1, enable preloading for CCR1
            self.tim().ccmr1_output.modify(|_, w| w
                .oc1m().bits(0b110)
                .oc1pe().set_bit());

            // DMA burst length = 2, base address = RCR
            self.tim().dcr.modify(|_, w| w.dbl().bits(1).dba().bits(12));
        }

        // Enable CC1 output
        self.tim().ccer.modify(|_, w| w.cc1e().set_bit());

        // Master output enable
        self.tim().bdtr.modify(|_, w| w.moe().set_bit());

        // Enable update DMA request
        self.tim().dier.modify(|_, w| w.ude().set_bit());

        unsafe {
            // Set ARR for 40kHz frequency
            self.tim().arr.modify(|_, w| w.arr().bits(24));
        }

        // Enable timer
        self.tim().cr1.modify(|_, w| w.cen().set_bit());

        unsafe {
            self.tim().ccr1.modify(|_, w| w.ccr().bits(0));
        }
    }

    pub fn busy(&self) -> bool {
        return self.dma_ch().cr.read().en().bit_is_set()
            && !self.dma().isr.read().tcif5().bit_is_set();
    }

    pub fn send(&mut self, address: u8, command: u8) {
        if self.busy() {
            return;
        }

        // Stop DMA
        self.dma_ch().cr.modify(|_, w| w.en().clear_bit());

        // Clear DMA interrupt
        self.dma().ifcr.write(|w| w.ctcif5().set_bit());

        let dma_buf = unsafe { DMA_BUF.as_mut().unwrap() };
        encode_command(dma_buf, command, address);

        // Set DMA transfer length
        self.dma_ch().ndtr.modify(|_, w| w.ndt().bits(dma_buf.len() as u16));

        // Start DMA
        self.dma_ch().cr.modify(|_, w| w.en().set_bit());

        // Update RCR
        unsafe {
            self.tim().rcr.write(|w| w.rep().bits(0));
        }

        self.tim().egr.write(|w| w.ug().set_bit());
    }
}

fn encode_command(dma_buf: &mut [VolatileCell<u8>], command: u8, address: u8) {
    let bits = (command & 0x7f) | ((address & 0x1f) << 7);

    let mut i = 2;

    const DUTY: u8 = 12;

    for _ in 0..REPEATS {
        let mut bit = 1;

        // sync pulse

        dma_buf[i + 0].set(96 - 1);
        dma_buf[i + 1].set(DUTY);
        dma_buf[i + 2].set(24 - 1);

        i += 4;

        for _ in 0..BITS {
            dma_buf[i + 0].set(if bits & bit != 0 { 48 } else { 24 } - 1);
            dma_buf[i + 1].set(DUTY);
            dma_buf[i + 2].set(24 - 1);

            i += 4;

            bit <<= 1;
        }

        // dead time

        dma_buf[i - 2].set(255);

        dma_buf[i].set(255);
        dma_buf[i + 2].set(255);

        i += 4;
    }
}