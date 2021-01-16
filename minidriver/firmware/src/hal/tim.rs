use stm32ral::tim1;
use stm32ral::{write_reg, read_reg};

pub struct TIM {
    tim1: tim1::Instance,
}

impl TIM {
    pub fn new(tim1: tim1::Instance) -> Self {
        TIM { tim1 }
    }

    /// Configure TIM1 to generate short pulses on channel 1.
    pub fn setup(&self) {
        // PWM mode 1: active when CNT<CC, else inactive.
        write_reg!(tim1, self.tim1, CCMR1, OC1M: 0b0110, OC1PE: 1);
        // Polarity active low, enabled.
        write_reg!(tim1, self.tim1, CCER, CC1P: 1, CC1E: 1);
        // ARR=250 gives 1kHz pulse per line (16kHz update rate).
        write_reg!(tim1, self.tim1, ARR, 16000);
        // CCR1=48 gives 3µs pulses.
        write_reg!(tim1, self.tim1, CCR1, 3*16);
        // Enable outputs.
        write_reg!(tim1, self.tim1, BDTR, MOE: 1);
        // Enable timer.
        write_reg!(tim1, self.tim1, CR1, ARPE: 1, CEN: 1);
        // Generate update even to load preloaded registers.
        write_reg!(tim1, self.tim1, EGR, UG: 1);
    }

    pub fn cc1if_set(&self) -> bool {
        read_reg!(tim1, self.tim1, SR, CC1IF == 1)
    }

    pub fn clear_sr(&self) {
        write_reg!(tim1, self.tim1, SR, 0);
    }
}
