#![allow(dead_code)]
use core::marker::PhantomData;
use vcell::VolatileCell;
use stm32f1xx_hal::stm32;
use crate::timer_dma::TimerDma;

pub type DmaBuffer = [VolatileCell<u16>];

pub struct PulseInput<TIM> {
    dma_buf: &'static mut DmaBuffer,
    synced: bool,
    _phantom: PhantomData<TIM>,
}

impl<TIM> PulseInput<TIM>
    where TIM: TimerDma
{
    pub fn new(
        tim: TIM,
        dma: TIM::HalDmaChannel,
        pin: TIM::HalPin,
        prescaler: u16,
        sync_len: u16,
        dma_buf: &'static mut DmaBuffer,
        enr: &mut TIM::HalEnr) -> Self
    {
        let _ = tim;
        let _ = dma;
        let _ = pin;

        TIM::tim_enable(enr);

        let mut pi = PulseInput {
            dma_buf,
            synced: false,
            _phantom: Default::default(),
        };

        pi.init(prescaler, sync_len);

        pi
    }

    fn tim(&self) -> &stm32::tim2::RegisterBlock {
        TIM::tim()
    }

    fn dma_ch(&self) -> &stm32::dma1::CH {
        TIM::dma_ch()
    }

    fn init(&mut self, prescaler: u16, sync_len: u16) {
        // Memory size 16 bits, peripheral size 16 bits, memory increment
        unsafe {
            self.dma_ch().cr.modify(|_, w| w
                .msize().bits(0b01)
                .psize().bits(0b01)
                .minc().set_bit());
        }

        // Peripheral address (timer burst DMA)
        self.dma_ch().par.write(|w| w.pa().bits(&self.tim().dmar as *const _ as *const u32 as u32));

        // Memory address
        self.dma_ch().mar.write(|w| w.ma().bits(self.dma_buf as *const _ as *const u16 as u32));

        // Only overflow generates events
        self.tim().cr1.modify(|_, w| w.urs().set_bit());

        unsafe {

            // Prescale timer
            self.tim().psc.modify(|_, w| w.psc().bits(prescaler));

            // Map both CC1 and CC2 to input TI1 (PA8), 8 clock filter
            /*self.tim().ccmr1_input.modify(|_, w| w
                .ic1f().bits(0b0011)
                .cc1s().bits(0b01)
                .ic2f().bits(0b0011)
                .cc2s().bits(0b10));*/

            // fuck you, cargo, this is all your fault
            let ccmr1_input: &stm32::tim2::CCMR1_INPUT = core::mem::transmute(&self.tim().ccmr1_output);
            ccmr1_input.modify(|_, w| w
                .ic1f().bits(0b0011)
                .cc1s().bits(0b01)
                .ic2f().bits(0b0011)
                .cc2s().bits(0b10));

            // Slave mode trigger = TI1FP1, slave mode selection = reset
            self.tim().smcr.modify(|_, w| w
                .ts().bits(0b101)
                .sms().bits(0b100));

            // DMA burst length = 2, base address = CCR1
            self.tim().dcr.modify(|_, w| w.dbl().bits(1).dba().bits(13));
        }

        // CC1 captures on rising edge, CC2 captures on falling edge, enable both
        self.tim().ccer.modify(|_, w| w
            .cc1p().clear_bit()
            .cc1e().set_bit()
            .cc2p().set_bit()
            .cc2e().set_bit());

        // Set ARR to sync_len worth of ticks
        self.tim().arr.modify(|_, w| w.arr().bits(sync_len));

        self.start_sync();
    }

    fn start_sync(&mut self) {
        self.synced = false;

        // Reset timer
        unsafe {
            self.tim().cnt.modify(|_, w| w.bits(0));
        }

        self.tim().egr.write(|w| w.ug().set_bit());

        self.tim().sr.modify(|_, w| w.cc1if().clear_bit().uif().clear_bit());

        // Enable timer
        self.tim().cr1.modify(|_, w| w.cen().set_bit());
    }

    fn start_capture(&mut self) {
        // Stop DMA
        self.dma_ch().cr.modify(|_, w| w.en().clear_bit());

        // Reset timer
        unsafe {
            self.tim().cnt.modify(|_, w| w.bits(0));
        }

        self.tim().egr.write(|w| w.ug().set_bit());

        // Clear interrupt bits
        self.tim().sr.modify(|_, w| w.cc1if().clear_bit().uif().clear_bit());

        TIM::dma_clear_isr_tc();

        // Enable CC1 DMA request
        self.tim().dier.modify(|_, w| w.cc1de().set_bit());

        // Set DMA transfer length
        self.dma_ch().ndtr.modify(|_, w| w.ndt().bits((self.dma_buf.len() & !1) as u16));

        // Start DMA
        self.dma_ch().cr.modify(|_, w| w.en().set_bit());

        // Enable timer
        self.tim().cr1.modify(|_, w| w.cen().set_bit());
    }

    fn get_pulse_train(&mut self) -> Option<PulseTrain<'_, TIM>> {
        // Disable timer
        self.tim().cr1.modify(|_, w| w.cen().clear_bit());

        // Stop DMA
        self.dma_ch().cr.modify(|_, w| w.en().clear_bit());

        TIM::dma_clear_isr_tc();

        // Disable CC1 DMA request
        self.tim().dier.modify(|_, w| w.cc1de().clear_bit());

        let count = (self.dma_buf.len() - self.dma_ch().ndtr.read().ndt().bits() as usize) / 2;

        if count > 1 {
            Some(PulseTrain {
                parent: self,
                count,
            })
        } else {
            self.start_sync();

            None
        }
    }

    fn resume(&mut self) {
        if self.synced {
            self.start_capture();
        } else {
            self.start_sync();
        }
    }

    pub fn tim_isr(&mut self) -> Option<PulseTrain<'_, TIM>> {
        if self.tim().sr.read().uif().bit_is_set() {
            self.tim().sr.modify(|_, w| w.uif().clear_bit());

            if self.synced {
                return self.get_pulse_train();
            } else {
                self.synced = true;

                self.start_capture();
            }
        }

        None
    }

    pub fn dma_isr(&mut self) -> Option<PulseTrain<'_, TIM>> {
        if TIM::dma_isr_tc() {
            self.synced = false;

            self.get_pulse_train()
        } else {
            None
        }
    }

    pub fn listen(&mut self) {
        self.dma_ch().cr.modify(|_, w| w.tcie().set_bit());
        self.tim().dier.modify(|_, w| w.uie().set_bit());
    }

    pub fn unlisten(&mut self) {
        self.dma_ch().cr.modify(|_, w| w.tcie().clear_bit());
        self.tim().dier.modify(|_, w| w.uie().clear_bit());
    }
}

pub struct PulseTrain<'a, TIM>
where
    TIM: TimerDma
{
    parent: &'a mut PulseInput<TIM>,
    count: usize,
}

impl<'a, TIM> PulseTrain<'a, TIM>
where
    TIM: TimerDma
{
    pub fn iter(&self) -> Pulses {
        Pulses {
            dma_buf: self.parent.dma_buf,
            count: self.count,
            i: 1, // first element is garbage
        }
    }
}

impl<'a, TIM> Drop for PulseTrain<'_ ,TIM>
where
    TIM: TimerDma
{
    fn drop(&mut self) {
        self.parent.resume()
    }
}

pub struct Pulses<'a> {
    dma_buf: &'a DmaBuffer,
    count: usize,
    i: usize,
}

impl Iterator for Pulses<'_> {
    type Item = (u16, u16);

    fn next(&mut self) -> Option<Self::Item> {
        if self.i == self.count {
            None
        } else {
            let r = Some((
                self.dma_buf[self.i * 2].get(),
                self.dma_buf[self.i * 2 + 1].get(),
            ));

            self.i += 1;

            r
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let left = self.count - self.i;

        (left, Some(left))
    }
}

impl ExactSizeIterator for Pulses<'_> { }
