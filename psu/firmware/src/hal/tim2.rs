use stm32ral::{tim2, write_reg, modify_reg};

pub struct TIM2 {
    tim2: tim2::Instance,
}

impl TIM2 {
    pub fn new(tim2: tim2::Instance) -> Self {
        TIM2 { tim2 }
    }

    pub fn setup(&self) {
        write_reg!(stm32ral::tim2, self.tim2, DIER, UIE: Enabled);
        // TIM2 runs at 70MHz. Prescale by 70 to count at 1MHz
        write_reg!(stm32ral::tim2, self.tim2, PSC, 70);
        // ARR at 100 gives 10kHz interrupt generation
        write_reg!(stm32ral::tim2, self.tim2, ARR, 100);
    }

    pub fn start(&self) {
        modify_reg!(stm32ral::tim2, self.tim2, CR1, CEN: Enabled);
    }

    pub fn isr(&self) {
        modify_reg!(stm32ral::tim2, self.tim2, SR, UIF: Clear);
    }
}
