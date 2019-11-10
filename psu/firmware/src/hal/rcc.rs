use stm32ral::{rcc, flash, read_reg, write_reg, modify_reg};

pub struct RCC {
    rcc: rcc::Instance,
    flash: flash::Instance,
}

impl RCC {
    pub fn new(rcc: rcc::Instance, flash: flash::Instance) -> Self {
        RCC { rcc, flash }
    }

    /// Set up device clocks
    pub fn setup(&self) {
        let flash = &self.flash;
        let rcc = &self.rcc;

        // Configure flash to two wait states ready for 70MHz operation
        modify_reg!(stm32ral::flash, flash, ACR, LATENCY: 2);

        // Ensure HSI is enabled, stable, and in use
        modify_reg!(stm32ral::rcc, rcc, CR, HSION: On);
        while read_reg!(stm32ral::rcc, rcc, CR, HSIRDY != Ready) {}
        modify_reg!(stm32ral::rcc, rcc, CFGR, SW: HSI);
        while read_reg!(stm32ral::rcc, rcc, CFGR, SWS != HSI) {}

        // Enable HSE
        modify_reg!(stm32ral::rcc, rcc, CR, HSEBYP: Bypassed);
        modify_reg!(stm32ral::rcc, rcc, CR, HSEON: On);
        while read_reg!(stm32ral::rcc, rcc, CR, HSERDY != Ready) {}

        // Ensure PLL is off before configuring
        modify_reg!(stm32ral::rcc, rcc, CR, PLLON: Off);
        while read_reg!(stm32ral::rcc, rcc, CR, PLLRDY == Ready) {}

        // Configure PLL: 25MHz HSE /5 *14 = 70MHz PLL
        // APB1: 35MHz, APB2: 70MHz, AHB: 70MHz
        modify_reg!(stm32ral::rcc, rcc, CFGR2, PREDIV: Div5, ADC12PRES: Div1);
        modify_reg!(stm32ral::rcc, rcc, CFGR, PLLSRC: HSE_Div_PREDIV, PLLMUL: Mul14);
        modify_reg!(stm32ral::rcc, rcc, CFGR, PPRE2: Div1, PPRE1: Div2, HPRE: Div1);
        modify_reg!(stm32ral::rcc, rcc, CR, PLLON: On);

        // Wait for PLL to be ready and swap to it
        while read_reg!(stm32ral::rcc, rcc, CR, PLLRDY != Ready) {}
        modify_reg!(stm32ral::rcc, rcc, CFGR, SW: PLL);
        while read_reg!(stm32ral::rcc, rcc, CFGR, SWS != PLL) {}

        // Set HRTIM1 to run off PLL VCO output
        write_reg!(stm32ral::rcc, rcc, CFGR3, 1<<12);

        // Enable peripheral clocks
        modify_reg!(stm32ral::rcc, rcc, AHBENR,
                    ADC12EN: Enabled, IOPAEN: Enabled, IOPBEN: Enabled, DMA1EN: Enabled);
        modify_reg!(stm32ral::rcc, rcc, APB1ENR, DAC1EN: Enabled, DAC2EN: Enabled);
        modify_reg!(stm32ral::rcc, rcc, APB2ENR,
                    HRTIM1EN: Enabled, USART1EN: Enabled, SYSCFGEN: Enabled);

        // Enable TIM1 only for prototyping
        modify_reg!(stm32ral::rcc, rcc, APB2ENR, TIM1EN: Enabled);
    }
}
