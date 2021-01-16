use stm32ral::rcc;
use stm32ral::modify_reg;

pub struct RCC {
    rcc: rcc::Instance,
}

impl RCC {
    pub fn new(rcc: rcc::Instance) -> Self {
        RCC { rcc }
    }

    pub fn setup(&self) {
        modify_reg!(rcc, self.rcc, IOPENR, IOPAEN: 1, IOPBEN: 1);
        modify_reg!(rcc, self.rcc, APBENR1, USART3EN: 1);
        modify_reg!(rcc, self.rcc, APBENR2, TIM1EN: 1);
    }
}
