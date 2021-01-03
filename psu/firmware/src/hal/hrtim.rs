use stm32ral::{hrtim_master, hrtim_tima, hrtim_common, read_reg, write_reg, modify_reg};

pub struct HRTIM {
    master: hrtim_master::Instance,
    tima: hrtim_tima::Instance,
    common: hrtim_common::Instance,
}

impl HRTIM {
    pub fn new(
        master: hrtim_master::Instance,
        tima: hrtim_tima::Instance,
        common: hrtim_common::Instance
    ) -> Self {
        HRTIM { master, tima, common }
    }

    pub fn setup(&self) {
        // Begin DLL calibration
        write_reg!(stm32ral::hrtim_common, self.common, DLLCR, CAL: Start);
        // Wait for calibration complete
        while read_reg!(stm32ral::hrtim_common, self.common, ISR, DLLRDY != 1) {}
        // Enable periodic recalibration
        write_reg!(stm32ral::hrtim_common, self.common, DLLCR, CALRTE: Micros910, CALEN: Enabled);

        // Configure master counter

        // Set prescaler to Div64 to obtain fHRTIM=70MHz, master counter to continuous,
        // enable preloading and update on repetition (=rollover with rep=0)
        write_reg!(stm32ral::hrtim_master, self.master, MCR,
                   MREPU: Enabled, PREEN: Enabled, CONT: Continuous, CKPSC: 6);

        // Set period to ensure regular pulses at startup; during operation DCM detection on V_Q
        // triggers the next charge cycle and causes a software reset of the master timer.
        write_reg!(stm32ral::hrtim_master, self.master, MPER, 0x4000);

        // Note: no master compare units used; we start TIMA on master reset each master period

        // Configure TIMa

        // Enable preload with update on reset, prescaler same as master.
        write_reg!(stm32ral::hrtim_tima, self.tima, TIMACR,
                   PREEN: Enabled, TxRSTU: Enabled, CKPSCx: 6);

        // Configure period to 120 counts maximum.
        // In principle 100 counts gives 1.4µs, which at 5µH and 24V gives 6.7A,
        // but we give a little slack to allow lower input voltage or output overload.
        write_reg!(stm32ral::hrtim_tima, self.tima, PERAR, 120);

        // Configure CMP1 to derive blanking signal for first 280ns
        write_reg!(stm32ral::hrtim_tima, self.tima, CMP1AR, 20);

        // Configure output 1 to set on counter reset
        write_reg!(stm32ral::hrtim_tima, self.tima, SETA1R, UPDATE: SetActive);

        // Configure output 1 to reset on counter period and EEV1 (COMP2, I_Q) event
        write_reg!(stm32ral::hrtim_tima, self.tima, RSTA1R,
                   EXTEVNT1: 1, PER: SetInactive);

        // Configure EEV1 and EEV2 to be blanked by CMP1
        write_reg!(stm32ral::hrtim_tima, self.tima, EEFAR1,
                   EE1FLTR: BlankResetToCompare1, EE2FLTR: BlankResetToCompare1);

        // Configure TIMA to be reset by master period and EEV2 (COMP4, V_Q, DCM detect)
        write_reg!(stm32ral::hrtim_tima, self.tima, RSTAR,
                   MSTPER: ResetCounter, EXTEVNT2: 1);

        // Configure capture on EEV1
        write_reg!(stm32ral::hrtim_tima, self.tima, CPT1ACR, EXEV1CPT: TriggerCapture);

        // Enable FLT2 (nRUN input)
        write_reg!(stm32ral::hrtim_tima, self.tima, FLTAR, FLT2EN: Active);

        // Configure burst mode controller:
        // Continuous mode, clocked from f_HRTIM/512 (68kHz),
        // preload registers, master clock unaffected, timer A clock stopped during idle.
        write_reg!(stm32ral::hrtim_common, self.common, BMCR,
                   BME: Disabled, BMOM: Continuous, BMCLK: Clock, BMPRSC: Div512,
                   BMPREN: Enabled, MTBM: Normal, TABM: Stopped);

        // Disable all burst mode triggers; we'll trigger manually in enable().
        write_reg!(stm32ral::hrtim_common, self.common, BMTRGR, 0);

        // Set idle time to 0 initially.
        write_reg!(stm32ral::hrtim_common, self.common, BMCMPR, 0);

        // Set period to 1000 counts = 68Hz
        write_reg!(stm32ral::hrtim_common, self.common, BMPER, 1000);

        // Configure external event conditioning for EEV1 and EEV2
        write_reg!(stm32ral::hrtim_common, self.common, EECR1,
                   EE1FAST: Asynchronous, EE1SNS: Rising, EE1POL: ActiveHigh, EE1SRC: Src2,
                   EE2FAST: Resynchronized, EE2SNS: Falling, EE2POL: ActiveLow, EE2SRC: Src2);

        // Configure fault input conditioning
        write_reg!(stm32ral::hrtim_common, self.common, FLTINR1,
                   FLT2F: Disabled, FLT2SRC: Input, FLT2P: ActiveHigh, FLT2E: Enabled);

        // Configure outputs.
        // Output A1: active high, idle inactive, set inactive on fault, idle on burst
        write_reg!(stm32ral::hrtim_tima, self.tima, OUTAR,
                   FAULT1: SetInactive, IDLEM1: SetIdle, IDLES1: Inactive, POL1: ActiveHigh);

        // Enable fault interrupt
        write_reg!(stm32ral::hrtim_common, self.common, IER, FLT2IE: Enabled, SYSFLTIE: Enabled);

        // Force an update of master and TIMA preload registers
        write_reg!(stm32ral::hrtim_common, self.common, CR2, MSWU: Update, TASWU: Update);
    }

    /// Enable HRTIM and begin driving outputs.
    pub fn enable(&self) {
        // Enable fault interrupt
        write_reg!(stm32ral::hrtim_common, self.common, IER, FLT2IE: Enabled, SYSFLTIE: Enabled);

        // Enable outputs
        write_reg!(stm32ral::hrtim_common, self.common, OENR, TA1OEN: Enable);

        // Begin counting
        modify_reg!(stm32ral::hrtim_master, self.master, MCR, TACEN: Enabled, MCEN: Enabled);

        // Trigger burst mode with 0 initial idle period
        write_reg!(stm32ral::hrtim_common, self.common, BMCMPR, 0);
        modify_reg!(stm32ral::hrtim_common, self.common, BMCR, BME: Enabled);
        write_reg!(stm32ral::hrtim_common, self.common, BMTRGR, SW: Trigger);
    }

    /// Set burst mode duty cycle.
    ///
    /// `duty` may take values from 0 to 1000, where 0 is 0% duty (outputs always off)
    /// and 1000 is 100% duty (outputs never forced off).
    pub fn set_duty(&self, duty: u16) {
        let duty = if duty > 1000 { 1000u32 } else { duty as u32 };
        write_reg!(stm32ral::hrtim_common, self.common, BMCMPR, 1000-duty);
    }

    /// Disable HRTIM, stopping outputs and counters.
    pub fn disable(&self) {
        // Disable outputs
        write_reg!(stm32ral::hrtim_common, self.common, ODISR, TA1ODIS: Disable);

        // Stop burst mode
        modify_reg!(stm32ral::hrtim_common, self.common, BMCR, BME: Disabled);

        // Stop counting
        modify_reg!(stm32ral::hrtim_master, self.master, MCR, TACEN: Disabled, MCEN: Disabled);
    }

    pub fn flt_isr(&self) {
        // Clear ISR bits
        if read_reg!(stm32ral::hrtim_common, self.common, ISR, FLT2 == Event) {
            write_reg!(stm32ral::hrtim_common, self.common, ICR, FLT2C: Clear);
        }
        if read_reg!(stm32ral::hrtim_common, self.common, ISR, SYSFLT == Event) {
            write_reg!(stm32ral::hrtim_common, self.common, ICR, SYSFLTC: Clear);
        }

        // Disable fault interrupt (it is re-enabled when the HRTIM is re-enabled)
        write_reg!(stm32ral::hrtim_common, self.common, IER, FLT2IE: Disabled, SYSFLTIE: Disabled);

        // Move from FAULT state to normal disabled state
        self.disable();
    }

    pub unsafe fn global_disable() {
        write_reg!(stm32ral::hrtim_common, HRTIM_Common, ODISR, TA1ODIS: Disable);
        write_reg!(stm32ral::hrtim_master, HRTIM_Master, MCR, TACEN: Disabled, MCEN: Disabled);
    }
}
