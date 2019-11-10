use stm32ral::{dac, write_reg};

pub struct DAC {
    dac: dac::Instance,
}

impl DAC {
    pub fn new(dac: dac::Instance) -> Self {
        DAC { dac }
    }

    pub fn setup(&self) {
        write_reg!(stm32ral::dac, self.dac, CR, EN2: Enabled, EN1: Enabled);
    }

    pub fn set_ch1(&self, val: u16) {
        write_reg!(stm32ral::dac, self.dac, DHR12R1, val as u32);
    }

    pub fn set_ch2(&self, val: u16) {
        write_reg!(stm32ral::dac, self.dac, DHR12R2, val as u32);
    }
}
