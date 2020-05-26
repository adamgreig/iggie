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
        // Configure USART. Enable DMA for transmission, enable transmitter, set to 3.5MBd.
        // Other settings are default: 8n1
        modify_reg!(stm32ral::usart, self.usart, CR3, DMAT: Enabled);
        modify_reg!(stm32ral::usart, self.usart, CR1, OVER8: Oversampling8);
        write_reg!(stm32ral::usart, self.usart, BRR, 18);
        modify_reg!(stm32ral::usart, self.usart, CR1, TCIE: Enabled, TE: Enabled, UE: Enabled);

    }

    /// Transmit a &[u8] via DMA
    pub fn transmit(&self, dma: &DMA, data: &[u8]) {
        write_reg!(stm32ral::usart, self.usart, ICR, TCCF: Clear);
        dma.usart1_enable(data);
    }

    /// Immediately transmit a u16 without DMA, little endian.
    pub fn transmit_u16(&self, word: u16) {
        let w1 = word & 0x00FF;
        while read_reg!(stm32ral::usart, self.usart, ISR, TXE == 0) {}
        write_reg!(stm32ral::usart, self.usart, TDR, w1 as u32);
        let w2 = (word & 0xFF00) >> 8;
        while read_reg!(stm32ral::usart, self.usart, ISR, TXE == 0) {}
        write_reg!(stm32ral::usart, self.usart, TDR, w2 as u32);
    }

    /// If TC flag is set and the DMA transfer has completed, clear TC and disable DMA.
    pub fn isr(&self, dma: &DMA) {
        if read_reg!(stm32ral::usart, self.usart, ISR, TC == 1) {
            if !dma.usart1_busy() {
                write_reg!(stm32ral::usart, self.usart, ICR, TCCF: Clear);
                dma.usart1_disable();
            }
        }
    }
}
