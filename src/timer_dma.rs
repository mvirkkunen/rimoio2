use stm32f1xx_hal::stm32;
use stm32f1xx_hal::gpio::*;

// Timer and DMA channel abstraction for CC1 DMA
pub trait TimerDma {
    type HalDmaChannel;
    type HalPin;
    type HalEnr;

    const DMA_CHANNEL: u32;

    fn tim() -> &'static stm32::tim2::RegisterBlock;

    fn tim_enable(hal_enr: &mut Self::HalEnr);

    fn dma() -> &'static stm32::dma1::RegisterBlock;

    fn dma_ch() -> &'static stm32::dma1::CH;

    fn dma_isr_tc() -> bool {
        Self::dma().isr.read().bits() & (2 << (Self::DMA_CHANNEL - 1)) != 0
    }

    fn dma_clear_isr_tc() {
        unsafe {
            Self::dma().ifcr.write(|w| w.bits(2 << (Self::DMA_CHANNEL - 1)));
        }
    }
}

macro_rules! timer_dma_impls {
    {
        $(
            $tim:ident => (
                $hal_enr:ident,
                $enr:ident,
                $enr_bit:ident,
                $(::$hal_pin:ident)*,
                $dma_channel:literal,
                $hal_chan:ident,
                $pac_chan:ident),)+
    } => {
        $(
            impl TimerDma for stm32::$tim {
                type HalDmaChannel = stm32f1xx_hal::dma::dma1::$hal_chan;
                type HalPin = stm32f1xx_hal::gpio$(::$hal_pin)*<Input<Floating>>;
                type HalEnr = stm32f1xx_hal::rcc::$hal_enr;

                const DMA_CHANNEL: u32 = $dma_channel;

                fn tim() -> &'static stm32::tim2::RegisterBlock {
                    unsafe { &*(stm32::$tim::ptr() as *const stm32::tim2::RegisterBlock) }
                }

                fn tim_enable(hal_enr: &mut Self::HalEnr) {
                    let _ = hal_enr;

                    let dp = unsafe { stm32::Peripherals::steal() };
                    dp.RCC.$enr.modify(|_, w| w.$enr_bit().set_bit());
                }

                fn dma() -> &'static stm32::dma1::RegisterBlock {
                    unsafe { &* stm32::DMA1::ptr() }
                }

                fn dma_ch() -> &'static stm32::dma1::CH {
                    &Self::dma().$pac_chan
                }
            }
        )*
    }
}

timer_dma_impls! {
    TIM1 => (APB2, apb2enr, tim1en, ::gpioa::PA8, 2, C2, ch2),
    TIM2 => (APB1, apb1enr, tim2en, ::gpioa::PA0, 5, C5, ch5),
    TIM3 => (APB1, apb1enr, tim3en, ::gpioa::PA6, 6, C6, ch6),
    TIM4 => (APB1, apb1enr, tim4en, ::gpiob::PB6, 1, C1, ch1),
}