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
        // Output 1 polarity active low and enabled.
        write_reg!(tim1, self.tim1, CCER, CC1P: 1, CC1E: 1);
        // ARR=1000 gives 64kHz pulse frequency, for 1kHz per column.
        write_reg!(tim1, self.tim1, ARR, 1000);
        // CCR1=64 gives 1µs pulses, scale as required.
        write_reg!(tim1, self.tim1, CCR1, 3*64);
        // Enable timer.
        write_reg!(tim1, self.tim1, CR1, ARPE: 1, CEN: 1);
        // Generate update even to load preloaded registers.
        write_reg!(tim1, self.tim1, EGR, UG: 1);
    }

    /// Set pulse duration in 1/64th µs.
    pub fn set_duration(&self, duration: u32) {
        write_reg!(tim1, self.tim1, CCR1, duration);
    }

    pub fn cc1if_set(&self) -> bool {
        read_reg!(tim1, self.tim1, SR, CC1IF == 1)
    }

    pub fn clear_sr(&self) {
        write_reg!(tim1, self.tim1, SR, 0);
    }

    pub fn wait_pulse(&self) {
        while !self.cc1if_set() { continue };
        self.clear_sr();
    }

    pub fn output_enable(&self) {
        write_reg!(tim1, self.tim1, BDTR, MOE: 1);
    }

    pub fn output_disable(&self) {
        write_reg!(tim1, self.tim1, BDTR, MOE: 0);
    }
}
