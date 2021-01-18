use stm32ral::rcc;
use stm32ral::{write_reg, read_reg, modify_reg};

pub struct RCC {
    rcc: rcc::Instance,
}

impl RCC {
    pub fn new(rcc: rcc::Instance) -> Self {
        RCC { rcc }
    }

    pub fn setup(&self, flash: &stm32ral::flash::Instance) {
        // Set flash wait states.
        modify_reg!(stm32ral::flash, flash, ACR, LATENCY: 2);
        while read_reg!(stm32ral::flash, flash, ACR, LATENCY != 2) {}
        // PLL setup, HSI=16M, div 1, mul 8, div 2 = 64MHz sysclk.
        // Disable PLL, wait for disabled, configure PLL, enable PLL, wait for enabled.
        write_reg!(rcc, self.rcc, CR, HSION: 1, PLLON: 0);
        while read_reg!(rcc, self.rcc, CR, PLLRDY != 0) {}
        write_reg!(rcc, self.rcc, PLLSYSCFGR,
                   PLLSRC: 0b10, PLLM: 0, PLLN: 8, PLLR: 1, PLLREN: 1);
        write_reg!(rcc, self.rcc, CR, HSION: 1, PLLON: 1);
        while read_reg!(rcc, self.rcc, CR, PLLRDY == 0) {}
        // Swap to SYSCLK and configure AHB and APB to 64MHz.
        write_reg!(rcc, self.rcc, CFGR, SW: 0b010, HPRE: 0, PPRE: 0);
        while read_reg!(rcc, self.rcc, CFGR, SWS != 0b010) {}
        // Enable peripheral clocks.
        modify_reg!(rcc, self.rcc, IOPENR, IOPAEN: 1, IOPBEN: 1);
        modify_reg!(rcc, self.rcc, APBENR1, USART3EN: 1);
        modify_reg!(rcc, self.rcc, APBENR2, TIM1EN: 1);
    }
}
