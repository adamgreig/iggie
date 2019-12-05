//! A simple PID control loop

use cortex_m_semihosting::hprintln;

/// PID controller implementation.
///
/// At each timestep the current process value and its derivative are required.
///
/// Extra non-linearities are provided for integral control:
///
/// * i_min and i_max represent the smallest and largest permitted values of the integrator,
///   before gain is applied, e.g. the largest possible contribution to control output from
///   the integrator is k_i * i_max.
///
/// * i_thresh is a threshold; error is only accumulated into the integrator if the error is
///   below this threshold, to reduce windup when the error is very large.
pub struct PID {
    dt: f32,
    k_p: f32,
    k_i: f32,
    k_d: f32,
    i: f32,
    i_min: f32,
    i_max: f32,
    i_thresh: f32,
}

fn fabs(x: f32) -> f32 {
    if x >= 0.0 {
        x
    } else {
        -x
    }
}

impl PID {
    pub const fn new(dt: f32, k_p: f32, k_i: f32, k_d: f32, i_min: f32, i_max: f32, i_thresh: f32)
        -> Self
    {
        PID { dt, k_p, k_i, k_d, i_min, i_max, i_thresh, i: 0.0 }
    }

    pub fn control_step(&mut self, setpoint: f32, x: f32, xdot: f32) -> f32 {
        // Compute error between setpoint and filtered process value
        let err = setpoint - x;

        // Accumulate integrator if error below threshold
        if fabs(err) < self.i_thresh {
            self.i += err * self.dt;
            if self.i > self.i_max {
                self.i = self.i_max;
            } else if self.i < self.i_min {
                self.i = self.i_min;
            }
        }

        // Compute P, I, and D contributions
        let p = self.k_p * err;
        let i = self.k_i * self.i;
        let d = self.k_d * -xdot;

        //hprintln!("p={} i={} d={}", p, i, d);

        // Sum to get overall control action
        p + i + d
    }
}
