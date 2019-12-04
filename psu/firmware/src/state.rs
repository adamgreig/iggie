#[repr(u8)]
pub enum FaultCode {
    NoFault = 0,
    NoRun   = 1,
    VLim    = 2,
    ILim    = 3,
    NoIQ    = 4,
}

#[repr(C)]
#[repr(align(4))]
pub struct State {
    magic: u32,
    pub v_in: f32,
    pub i_in: f32,
    pub v_out: f32,
    pub i_out: f32,
    pub v_q: f32,
    pub i_q: f32,
    pub ref_i_q: u16,
    pub fault_code: FaultCode,
    _1: u8,
}

impl State {
    pub const fn new() -> State {
        State {
            magic: 0x74656c65,
            v_in: 0.0, i_in: 0.0, v_out: 0.0, i_out: 0.0, v_q: 0.0, i_q: 0.0,
            ref_i_q: 0, fault_code: FaultCode::NoFault, _1: 0,
        }
    }

    pub fn update_adc(&mut self, buf1: [u16; 4], buf2: [u16; 2]) {
        let [vout, iout, iin, vin] = buf1;
        let [vq, iq] = buf2;

        self.v_in = (vin as f32) * (3.3 * 11.0 / 4096.0);
        self.i_in = (iin as f32) * (3.3 / 4096.0);
        self.v_out = (vout as f32) * (3.3 * 200.6 / 4096.0);
        self.i_out = (iout as f32) * (3.3 * 0.04 / 4096.0);
        self.v_q = (vq as f32) * (3.3 * 21.0 / 4096.0);
        self.i_q = (iq as f32) * (3.3 * (1.0 / 0.51) / 4096.0);
    }

    pub fn update_ref_i_q(&mut self, ref_i_q: u16) {
        self.ref_i_q = ref_i_q;
    }

    pub fn set_fault(&mut self, fault: FaultCode) {
        self.fault_code = fault;
    }
}

/// Trait for structs which can be safely cast to &[u8].
pub unsafe trait ToBytes: Sized {
    fn to_bytes(&self) -> &[u8] {
        // UNSAFE: We return a non-mutable slice into this packed struct's memory at the
        // length of the struct, with a lifetime bound to &self.
        unsafe {
            core::slice::from_raw_parts(self as *const _ as *const u8,
                                        core::mem::size_of::<Self>())
        }
    }
}

unsafe impl ToBytes for State {}
