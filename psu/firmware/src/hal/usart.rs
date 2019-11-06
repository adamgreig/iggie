use stm32ral::{usart, modify_reg, write_reg, read_reg};

use super::dma::DMA;

pub struct USART {
    usart: usart::Instance,
}

impl USART {
    pub fn new(usart: usart::Instance) -> Self {
        USART { usart }
    }

    pub fn setup(&self) {
        // Configure USART. Enable DMA for transmission, enable transmitter, set to 1MBd.
        // Other settings are default: 8n1
        modify_reg!(stm32ral::usart, self.usart, CR3, DMAT: Enabled);
        write_reg!(stm32ral::usart, self.usart, BRR, 35);
        modify_reg!(stm32ral::usart, self.usart, CR1, TCIE: Enabled, TE: Enabled, UE: Enabled);
    }

    pub fn transmit(&self, dma: &DMA, data: &[u8]) {
        write_reg!(stm32ral::usart, self.usart, ICR, TCCF: Clear);
        dma.usart1_enable(data);
    }

    pub fn isr(&self, dma: &DMA) {
        if read_reg!(stm32ral::usart, self.usart, ISR, TC == 1) {
            if !dma.usart1_busy() {
                write_reg!(stm32ral::usart, self.usart, ICR, TCCF: Clear);
                dma.usart1_disable();
            }
        }
    }
}
