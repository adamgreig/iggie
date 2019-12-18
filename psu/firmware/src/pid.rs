//! A simple PID control loop

/// PID controller implementation.
///
/// At each timestep the current process value and its derivative are required.
///
/// Extra non-linearities are provided for integral control:
///
/// * i_min and i_max represent the smallest and largest permitted values of the integrator,
///   before gain is applied, e.g. the largest possible contribution to control output from
///   the integrator is k_i * i_max.
pub struct PID {
    dt: f32,
    k_p: f32,
    k_i: f32,
    k_d: f32,
    i: f32,
    i_min: f32,
    i_max: f32,
}

impl PID {
    pub const fn new(dt: f32, k_p: f32, k_i: f32, k_d: f32, i_min: f32, i_max: f32)
        -> Self
    {
        PID { dt, k_p, k_i, k_d, i_min, i_max, i: 0.0 }
    }

    pub fn zero(&mut self) {
        self.i = 0.0;
    }

    pub fn control_step(&mut self, setpoint: f32, x: f32, xdot: f32) -> f32 {
        // Compute error between setpoint and filtered process value
        let err = setpoint - x;

        // Accumulate integrator if error below threshold
        self.i += err * self.dt;
        if self.i > self.i_max {
            self.i = self.i_max;
        } else if self.i < self.i_min {
            self.i = self.i_min;
        }

        // Compute P, I, and D contributions
        let p = self.k_p * err;
        let i = self.k_i * self.i;
        let d = self.k_d * -xdot;

        //hprintln!("p={} i={} d={}", p, i, d);

        // Sum to get overall control action
        p + i + d
    }

    pub fn get_i(&self) -> f32 {
        self.i
    }
}
