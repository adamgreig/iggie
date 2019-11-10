use stm32ral::{comp, write_reg};

pub struct Comp {
    comp: comp::Instance,
}

impl Comp {
    pub fn new(comp: comp::Instance) -> Self {
        Comp { comp }
    }

    pub fn setup(&self) {
        write_reg!(stm32ral::comp, self.comp, COMP2_CSR,
                   COMP2INMSEL: PA4_DAC1_CH1, COMP2EN: Enabled);
        write_reg!(stm32ral::comp, self.comp, COMP4_CSR,
                   COMP4INMSEL: DAC1_CH2, COMP4EN: Enabled);
    }
}
