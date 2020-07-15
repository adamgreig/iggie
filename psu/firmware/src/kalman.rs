//! A simple Kalman filter for one-dimensional readings

/// Kalman filter implementation for estimating the first derivative of a 1-d variable,
/// with a fixed sample rate 1/dt for samples of that variable.
///
/// Q is set to a representative process variance; it is the variance of an unknown
/// change in second derivative over each sampling period dt.
/// R is set to the expected sensor variance.
/// x tracks the current state and xp tracks the current state prediction.
/// P tracks the state covariance matrix and Pp its prediction.
/// Q0 is the constant process covariance matrix and is computed from Q.
///
/// ## Model
///
/// For convenience, variables in this explanation are named following the Wikipedia
/// article on Kalman filters, refer there for more details.
///
/// Our state vector `𝘅_k` is:
///
///     𝘅_k = [ x_k       ]
///           [ dx_k/dt   ]
///
/// i.e., the modelled process variable `x_k` and its first derivative.
///
/// The dynamics are modelled by the implicit state transition model `𝗙_k`,
/// produced by discretising the dynamics with a velocity Verlet integrator
/// with constant second derivative:
///
///     𝗙_k = [ 1  dt ]
///           [ 0   1 ]
///
/// We model the system as undergoing an unknown change in derivative
/// a=d²x/dt² whose value a~N(0, Q), with a constant over each integration period dt.
///
/// Integrating the constant a over a time period dt leads to the process noise 𝘄_k:
///
///     𝘄_k = [ a dt²/2 ]
///           [ a dt    ]
///
/// We therefore find 𝗤_k as E[𝘄_k . 𝘄_k'], where E[j²]=Q, the provided process variance.
/// Note that 𝘄_k~N(0, 𝗤_k).
///
///     𝗤_k = Q . [ dt^4/4   dt^3/2 ]
///               [ dt^3/2   dt^2/1 ]
///
/// As 𝗤_k does not depend on k, we compute 𝗤_k once and store it as `Kalman.Q0`.
///
/// Since there is no modelled control input (𝘂_k=0), the state evolution is:
///
///     𝘅_k = 𝗙_k . 𝘅_k-1 + 𝘄_k
///
/// The implicit observation model `𝗛_k` is simple:
///
///     𝗛_k = [ 1 0 ]
///
/// In other words, we directly observe the process variable itself in noise:
///
///     z_k = 𝗛_k . 𝘅_k + v_k
///         =       x_k + v_k
///
/// We track the a posteriori state estimate 𝘅_k|k as `Kalman.x`, and the a priori state estimate
/// 𝘅_k|k-1 as `Kalman.xp`. Likewise the a posteriori error covariance 𝗣_k|k is `Kalman.P` and
/// the a priori prediction 𝗣_k|k-1 is `Kalman.Pp`
///
#[allow(non_snake_case)]
pub struct Kalman {
    R: f32,
    dt: f32,
    x: [f32; 2],
    xp: [f32; 2],
    P: [[f32; 2]; 2],
    Pp: [[f32; 2]; 2],
    Q0: [[f32; 2]; 2],
}

impl Kalman {
    /// Create a new Kalman struct initialised to a current value `z` and zero-valued derivatives.
    ///
    /// Q: process variance (variance of random change in derivative per second)
    /// R: sensor variance
    /// dt: time interval between filter updates
    /// z: initial state of process variable
    #[allow(non_snake_case)]
    pub fn new(Q: f32, R: f32, dt: f32, z: f32) -> Self {
        let mut k = Kalman {
            R, dt,
            x: [0f32; 2], xp: [0f32; 2],
            P: [[0f32; 2]; 2], Pp: [[0f32; 2]; 2],
            Q0: [[0f32; 2]; 2],
        };

        // Initialise state to (z, 0, 0) with small error covariance along diagonal
        k.x[0] = z;
        for i in 0..2 {
            for j in 0..2 {
                if i == j {
                    k.P[i][j] = 1e-3;
                }
            }
        }

        // Initialise process noise covariance from dynamic model
        k.Q0[0][0] = Q * (dt*dt*dt*dt      )/4.0;
        k.Q0[0][1] = Q * (dt*dt*dt         )/2.0;
        k.Q0[1][0] = Q * (dt*dt*dt         )/2.0;
        k.Q0[1][1] = Q * (dt*dt            )/1.0;

        k
    }

    /// Run a Kalman update step from measurement z
    ///
    /// Call this function at regular `dt` intervals for proper filtering.
    ///
    /// This function models the Kalman update equations:
    ///
    /// 𝘆_k = 𝘇_k - 𝗛_k . 𝘅_k|k-1
    ///     = z_k - x_k|k-1
    /// 𝗦_k = 𝗛_k 𝗣_k|k-1 𝗛'_k + 𝗥_k
    ///     = P00_k|k-1 + R
    /// 𝗞_k = 𝗣_k|k-1 𝗛'_k (𝗦_k)^-1
    ///     = [P00 P10]'_k|k-1 / (P00_k|k-1 + R)
    /// 𝘅_k|k = 𝘅_k|k-1 + 𝗞_k 𝘆_k
    /// 𝗣_k|k = (𝗜 - 𝗞_k 𝗛_k)𝗣_k|k-1
    ///
    /// Note that `x` is 𝘅_k|k, `xp` is 𝘅_k|k-1, `P` is 𝗣_k|k, `Pp` is 𝗣_k|k-1.
    pub fn update(&mut self, z: f32) {
        let y = z - self.xp[0];
        let k = 1.0 / (self.Pp[0][0] + self.R);
        self.x[0] = self.xp[0] + k * self.Pp[0][0] * y;
        self.x[1] = self.xp[1] + k * self.Pp[1][0] * y;

        self.P[0][0] = self.Pp[0][0] - k * self.Pp[0][0] * self.Pp[0][0];
        self.P[0][1] = self.Pp[0][1] - k * self.Pp[0][0] * self.Pp[0][1];
        self.P[1][0] = self.Pp[1][0] - k * self.Pp[1][0] * self.Pp[0][0];
        self.P[1][1] = self.Pp[1][1] - k * self.Pp[1][0] * self.Pp[0][1];
    }

    /// Run a Kalman predict step
    ///
    /// Call this function whenever an updated prediction `xp` is required.
    ///
    /// This function models the Kalman predict equations:
    ///
    /// 𝘅_k|k-1 = 𝗙_k 𝘅_k-1|k-1
    /// 𝗣_k|k-1 = 𝗙_k 𝗣_k-1|k-1 𝗙'_k + 𝗤_k
    pub fn predict(&mut self) {
        let dt = self.dt;
        self.xp[0] = self.x[0] + dt*self.x[1];
        self.xp[1] = self.x[1];

        self.Pp[0][0] = self.P[0][0] + self.P[1][0]*dt;
        self.Pp[0][1] = self.P[0][1] + self.P[1][1]*dt;
        self.Pp[1][0] = self.P[1][0] + self.P[1][1]*dt;
        self.Pp[1][1] = self.P[1][1];
        self.Pp[0][0] += self.Pp[0][1]*dt;

        for i in 0..2 {
            for j in 0..2 {
                self.Pp[i][j] += self.Q0[i][j];
            }
        }
    }

    pub fn get(&self) -> (f32, f32) {
        (self.x[0], self.x[1])
    }
}

