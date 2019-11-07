use stm32ral::dma1 as dma;
use stm32ral::{read_reg, write_reg, modify_reg};

const USART_TDR_OFFSET: u32 = 0x28;
const ADC_DR_OFFSET: u32 = 0x40;

pub struct DMA {
    dma: dma::Instance,
}

impl DMA {
    pub fn new(dma: dma::Instance) -> Self {
        DMA { dma }
    }

    pub fn setup(&self) {
        let dma = &self.dma;

        // Configure channel 1 for ADC1
        write_reg!(dma, dma, CR1, MEM2MEM: Disabled, PL: Medium, MSIZE: Bits16, PSIZE: Bits16,
                                  MINC: Enabled, PINC: Disabled, CIRC: Enabled,
                                  DIR: FromPeripheral, TCIE: Disabled, EN: Disabled);
        write_reg!(dma, dma, PAR1, stm32ral::adc::ADC1 as u32 + ADC_DR_OFFSET);

        // Configure channel 2 for ADC2
        write_reg!(dma, dma, CR2, MEM2MEM: Disabled, PL: Medium, MSIZE: Bits16, PSIZE: Bits16,
                                  MINC: Enabled, PINC: Disabled, CIRC: Enabled,
                                  DIR: FromPeripheral, TCIE: Disabled, EN: Disabled);
        write_reg!(dma, dma, PAR2, stm32ral::adc::ADC2 as u32 + ADC_DR_OFFSET);

        // Configure channel 4 for USART1 TX
        write_reg!(dma, dma, CR4, MEM2MEM: Disabled, PL: Low, MSIZE: Bits8, PSIZE: Bits8,
                                  MINC: Enabled, PINC: Disabled, CIRC: Disabled, DIR: FromMemory,
                                  TEIE: Disabled, HTIE: Disabled, TCIE: Disabled, EN: Disabled);
        write_reg!(dma, dma, PAR4, stm32ral::usart::USART1 as u32 + USART_TDR_OFFSET);
    }

    pub fn usart1_enable(&self, data: &[u8]) {
        write_reg!(dma, self.dma, IFCR, CGIF4: Clear);
        write_reg!(dma, self.dma, NDTR4, data.len() as u32);
        write_reg!(dma, self.dma, MAR4, data.as_ptr() as u32);
        modify_reg!(dma, self.dma, CR4, EN: Enabled);
    }

    pub fn usart1_busy(&self) -> bool {
        read_reg!(dma, self.dma, ISR, TCIF4 == NotComplete)
    }

    pub fn usart1_disable(&self) {
        modify_reg!(dma, self.dma, CR4, EN: Disabled);
    }

    pub fn adc1_enable(&self, buf: &mut [u16]) {
        write_reg!(dma, self.dma, IFCR, CGIF1: Clear);
        write_reg!(dma, self.dma, NDTR1, buf.len() as u32);
        write_reg!(dma, self.dma, MAR1, buf.as_ptr() as u32);
        modify_reg!(dma, self.dma, CR1, EN: Enabled);
    }

    pub fn adc2_enable(&self, buf: &mut [u16]) {
        write_reg!(dma, self.dma, IFCR, CGIF2: Clear);
        write_reg!(dma, self.dma, NDTR2, buf.len() as u32);
        write_reg!(dma, self.dma, MAR2, buf.as_ptr() as u32);
        modify_reg!(dma, self.dma, CR2, EN: Enabled);
    }
}
