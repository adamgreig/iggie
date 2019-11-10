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

        // Set period to maximum for prototyping at low frequency
        // XXX: Either set this to something higher-freq or dynamically adjust at runtime
        write_reg!(stm32ral::hrtim_master, self.master, MPER, 0xBFDF);

        // Note: no master compare units used; we start TIMA on master reset each master period

        // Configure TIMa

        // Enable preload with update on reset, prescaler same as master.
        write_reg!(stm32ral::hrtim_tima, self.tima, TIMACR,
                   PREEN: Enabled, TxRSTU: Enabled, CKPSCx: 6);

        // Configure period to 100 counts maximum
        write_reg!(stm32ral::hrtim_tima, self.tima, PERAR, 100);

        // Configure CMP1 to derive blanking signal for first 280ns
        write_reg!(stm32ral::hrtim_tima, self.tima, CMP1AR, 20);

        // Configure output 1 to set on counter reset
        write_reg!(stm32ral::hrtim_tima, self.tima, SETA1R, UPDATE: SetActive);

        // Configure output 1 to reset on counter period and EEV1 (COMP2, I_Q) event
        write_reg!(stm32ral::hrtim_tima, self.tima, RSTA1R,
                   EXTEVNT1: 1, PER: SetInactive);

        // Configure EEV1 to be blanked by CMP1
        write_reg!(stm32ral::hrtim_tima, self.tima, EEFAR1, EE1FLTR: BlankResetToCompare1);

        // Configure TIMA to be reset by master period
        write_reg!(stm32ral::hrtim_tima, self.tima, RSTAR, MSTPER: ResetCounter);

        // Configure capture on EEV1
        write_reg!(stm32ral::hrtim_tima, self.tima, CPT1ACR, EXEV1CPT: TriggerCapture);

        // Enable FLT2 (nRUN input)
        // Disabled for development
        //write_reg!(stm32ral::hrtim_tima, self.tima, FLTAR, FLT2EN: Active);

        // Configure external event conditioning
        write_reg!(stm32ral::hrtim_common, self.common, EECR1,
                   EE1FAST: Asynchronous, EE1SRC: Src2);

        // Configure fault input conditioning
        write_reg!(stm32ral::hrtim_common, self.common, FLTINR1,
                   FLT2F: Disabled, FLT2SRC: Input, FLT2P: ActiveHigh, FLT2E: Enabled);

        // Configure outputs
        write_reg!(stm32ral::hrtim_tima, self.tima, OUTAR,
                   FAULT1: SetInactive, IDLES1: Inactive, POL1: ActiveHigh);

        // Force an update of master and TIMA preload registers
        write_reg!(stm32ral::hrtim_common, self.common, CR2, MSWU: Update, TASWU: Update);
    }

    pub fn start(&self) {
        // Enable outputs
        write_reg!(stm32ral::hrtim_common, self.common, OENR, TA1OEN: Enable);

        // Begin counting
        modify_reg!(stm32ral::hrtim_master, self.master, MCR, TACEN: Enabled, MCEN: Enabled);
    }
}
