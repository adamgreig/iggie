use stm32ral::{adc, modify_reg, write_reg, read_reg};
use super::dma::DMA;

pub struct ADC {
    adc1: adc::Instance,
    adc2: adc::Instance,
}

impl ADC {
    pub fn new(adc1: adc::Instance, adc2: adc::Instance) -> Self {
        ADC { adc1, adc2 }
    }

    pub fn setup(&self) {
        // Configures ADCs for continuous sampling into DMA.
        // ADC1: Channels 1, 2, 3, 4
        // ADC2: Channels 3, 4

        // Enable ADC voltage regulator and wait at least 10µs
        write_reg!(stm32ral::adc, self.adc1, CR, ADVREGEN: Intermediate);
        write_reg!(stm32ral::adc, self.adc1, CR, ADVREGEN: Enabled);
        write_reg!(stm32ral::adc, self.adc2, CR, ADVREGEN: Intermediate);
        write_reg!(stm32ral::adc, self.adc2, CR, ADVREGEN: Enabled);
        cortex_m::asm::delay(700);

        // Run calibration and wait for completion
        modify_reg!(stm32ral::adc, self.adc1, CR, ADCAL: Calibration, ADCALDIF: SingleEnded);
        modify_reg!(stm32ral::adc, self.adc2, CR, ADCAL: Calibration, ADCALDIF: SingleEnded);
        while read_reg!(stm32ral::adc, self.adc1, CR, ADCAL != Complete) {}
        while read_reg!(stm32ral::adc, self.adc2, CR, ADCAL != Complete) {}

        // Enable interrupt on end of conversion sequence
        write_reg!(stm32ral::adc, self.adc1, IER, EOSIE: Enabled);
        //write_reg!(stm32ral::adc, self.adc2, IER, EOSIE: Enabled);

        // Configure for continuous DMA data
        write_reg!(stm32ral::adc, self.adc1, CFGR,
                   CONT: Continuous, DMACFG: Circular, DMAEN: Enabled);
        write_reg!(stm32ral::adc, self.adc2, CFGR,
                   CONT: Continuous, DMACFG: Circular, DMAEN: Enabled);

        // Configure sample times
        // ADC clock is 70MHz (direct from PLL, no division).
        // We set 181.5 cycles of sampling time. Additionally at 12 bits there are 12.5 cycles
        // of conversion time, for a total of 194 samples per channel.
        // With four channels, ADC1 obtains 90.2kS/s, and with two channels,
        // ADC2 obtains 180.4kS/s.
        write_reg!(stm32ral::adc, self.adc1, SMPR1,
                   SMP1: Cycles181_5, SMP2: Cycles181_5, SMP3: Cycles181_5, SMP4: Cycles181_5);
        write_reg!(stm32ral::adc, self.adc2, SMPR1,
                   SMP3: Cycles181_5, SMP4: Cycles181_5);

        // Configure sampling sequence
        write_reg!(stm32ral::adc, self.adc1, SQR1, L: 4 - 1, SQ1: 1, SQ2: 2, SQ3: 3, SQ4: 4);
        write_reg!(stm32ral::adc, self.adc2, SQR1, L: 2 - 1, SQ1: 3, SQ2: 4);

        // Enable ADC
        modify_reg!(stm32ral::adc, self.adc1, CR, ADEN: Enable);
        modify_reg!(stm32ral::adc, self.adc2, CR, ADEN: Enable);
    }

    pub fn start(&mut self, dma: &DMA, buf1: &mut [u16; 4], buf2: &mut [u16; 2]) {
        // Start DMA for each ADC
        dma.adc1_enable(buf1);
        dma.adc2_enable(buf2);

        // Start conversions
        modify_reg!(stm32ral::adc, self.adc1, CR, ADSTART: Start);
        modify_reg!(stm32ral::adc, self.adc2, CR, ADSTART: Start);
    }

    pub fn isr(&self) {
        if read_reg!(stm32ral::adc, self.adc1, ISR, EOS == 1) {
            write_reg!(stm32ral::adc, self.adc1, ISR, EOS: 1);
        }
        if read_reg!(stm32ral::adc, self.adc2, ISR, EOS == 1) {
            write_reg!(stm32ral::adc, self.adc2, ISR, EOS: 1);
        }
    }
}
