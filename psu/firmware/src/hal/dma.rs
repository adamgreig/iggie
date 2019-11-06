use stm32ral::dma1 as dma;
use stm32ral::{read_reg, write_reg, modify_reg};

const USART_TDR_OFFSET: u32 = 0x28;

pub struct DMA {
    dma: dma::Instance,
}

impl DMA {
    pub fn new(dma: dma::Instance) -> Self {
        DMA { dma }
    }

    pub fn setup(&self) {
        let dma = &self.dma;

        // Configure channel 4 for USART1 TX
        write_reg!(dma, dma, CR4, MEM2MEM: Disabled, PL: Low, MSIZE: Bits8, PSIZE: Bits8,
                                  MINC: Enabled, PINC: Disabled, CIRC: Disabled, DIR: FromMemory,
                                  TEIE: Disabled, HTIE: Disabled, TCIE: Disabled, EN: Disabled);
        write_reg!(dma, dma, PAR4, stm32ral::usart::USART1 as u32 + USART_TDR_OFFSET);
    }

    pub fn usart1_enable(&self, data: &[u8]) {
        let dma = &self.dma;
        write_reg!(dma, dma, IFCR, CGIF4: Clear);
        write_reg!(dma, dma, NDTR4, data.len() as u32);
        write_reg!(dma, dma, MAR4, data.as_ptr() as u32);
        modify_reg!(dma, dma, CR4, EN: Enabled);
    }

    pub fn usart1_busy(&self) -> bool {
        read_reg!(dma, self.dma, ISR, TCIF4 == NotComplete)
    }

    pub fn usart1_disable(&self) {
        modify_reg!(dma, self.dma, CR4, EN: Disabled);
    }
}
