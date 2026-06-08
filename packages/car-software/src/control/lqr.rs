/// control/lqr.rs — LQR lateral (steering) controller
///
/// Computes a steering angle from a 4-element state vector:
///   state = [lateral_error_m, lateral_rate_m_s, heading_error_rad, yaw_rate_rad_s]
///
/// The gain matrix K is computed offline (MATLAB/Python LQR solve on your
/// linearised bicycle model) and hardcoded here as constants.
/// Tune Q (state cost) and R (input cost) to trade tracking vs. aggressiveness.
///
/// Once a wheel encoder is fitted, replace the stub state vector in main.rs
/// with real lateral error from your path planner.

/// Gain vector K = [k_lat_err, k_lat_rate, k_heading_err, k_yaw_rate]
/// Placeholder values — MUST be replaced after system identification.
const K: [f32; 4] = [0.80, 0.30, 1.20, 0.40];

/// TODO: importer la constante SERVO_MAX_RAD(main.rs) via le constructeur
const STEER_MAX_RAD: f32 = 0.5;

pub struct Lqr {
    k: [f32; 4],
}

impl Lqr {
    pub fn new() -> Self {
        Self { k: K }
    }

    /// Renvoies l'angle de virage en rad
    /// Positif = left, negatif = right
    pub fn compute_lateral(&self, state: [f32; 4]) -> f32 {
        let u: f32 = self.k.iter().zip(state.iter()).map(|(k, x)| k * x).sum();
        u.clamp(-STEER_MAX_RAD, STEER_MAX_RAD)
    }

    /// Pour tuner en realtime avec un UDP)
    pub fn set_gains(&mut self, k: [f32; 4]) {
        self.k = k;
    }
}
