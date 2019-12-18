use stm32ral::{gpio, modify_reg, write_reg, read_reg};

pub struct GPIO {
    gpioa: gpio::Instance,
    gpiob: gpio::Instance,
}

impl GPIO {
    pub fn new(gpioa: gpio::Instance, gpiob: gpio::Instance) -> Self {
        GPIO { gpioa, gpiob }
    }

    pub fn setup(&self) {
        let gpioa = &self.gpioa;
        let gpiob = &self.gpiob;

        // Set PA8 GD to HRTIM CHA1
        modify_reg!(stm32ral::gpio, gpioa, ODR, ODR8: 0);
        modify_reg!(stm32ral::gpio, gpioa, OSPEEDR, OSPEEDR8: HighSpeed);
        modify_reg!(stm32ral::gpio, gpioa, AFRH, AFRH8: AF13);
        modify_reg!(stm32ral::gpio, gpioa, MODER, MODER8: Alternate);

        // Set both LEDs to outputs
        modify_reg!(stm32ral::gpio, gpiob, MODER, MODER3: Output, MODER4: Output);
        modify_reg!(stm32ral::gpio, gpiob, ODR, ODR3: 0, ODR4: 0);

        // Set PB6 to USART Tx (AF7)
        modify_reg!(stm32ral::gpio, gpiob, AFRL, AFRL6: AF7);
        modify_reg!(stm32ral::gpio, gpiob, MODER, MODER6: Alternate);

        // Set PA0, 1, 2, 3, 6, 7 to analogue input for ADCs and COMPs
        modify_reg!(stm32ral::gpio, gpioa, MODER, MODER0: Analog, MODER1: Analog, MODER2: Analog,
                                                  MODER3: Analog, MODER6: Analog, MODER7: Analog);

        // Set PB0 to analogue input for COMP4
        modify_reg!(stm32ral::gpio, gpiob, MODER, MODER0: Analog);

        // Set PA15 to HRTIM1_FLT2 for fault detection
        modify_reg!(stm32ral::gpio, gpioa, AFRH, AFRH15: AF13);
        modify_reg!(stm32ral::gpio, gpioa, MODER, MODER15: Alternate);

        // Prototyping COMP and DAC outputs:

        // Set COMP2 output to PA12 for prototyping
        modify_reg!(stm32ral::gpio, gpioa, AFRH, AFRH12: AF8);
        modify_reg!(stm32ral::gpio, gpioa, MODER, MODER12: Alternate);

        // Set COMP4 output to PB1 for prototyping
        modify_reg!(stm32ral::gpio, gpiob, AFRL, AFRL1: AF8);
        modify_reg!(stm32ral::gpio, gpiob, MODER, MODER1: Alternate);

        // Set DAC1_OUT1 output to PA4 for prototyping
        modify_reg!(stm32ral::gpio, gpioa, MODER, MODER4: Analog);

        // Set DAC1_OUT2 output to PA5 for prototyping
        modify_reg!(stm32ral::gpio, gpioa, MODER, MODER5: Analog);
    }

    pub fn set_400v_led(&self, state: bool) {
        if state {
            write_reg!(stm32ral::gpio, self.gpiob, BSRR, BS3: Set);
        } else {
            write_reg!(stm32ral::gpio, self.gpiob, BSRR, BR3: Reset);
        }
    }

    pub fn set_err_led(&self, state: bool) {
        if state {
            write_reg!(stm32ral::gpio, self.gpiob, BSRR, BS4: Set);
        } else {
            write_reg!(stm32ral::gpio, self.gpiob, BSRR, BR4: Reset);
        }
    }

    pub fn get_run(&self) -> bool {
        read_reg!(stm32ral::gpio, self.gpioa, IDR, IDR15 == Low)
    }

    pub unsafe fn global_set_err_led() {
        write_reg!(stm32ral::gpio, GPIOB, BSRR, BS4: Set);
    }
}
