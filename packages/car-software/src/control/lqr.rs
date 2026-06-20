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
use crate::control::actuate::SERVO_MAX_RAD;

pub struct LqrState {
    pub lateral_error_m: f32,
    pub lateral_rate_m_s: f32,
    pub heading_error_rad: f32,
    pub yaw_rate_rad_s: f32,
}

impl LqrState {
    pub fn as_array(&self) -> [f32; 4] {
        [
            self.lateral_error_m,
            self.lateral_rate_m_s,
            self.heading_error_rad,
            self.yaw_rate_rad_s,
        ]
    }
}
#[derive(Default)]
pub struct Lqr {
    k: [f32; 4],
}

impl Lqr {
    pub fn new() -> Self {
        Self { k: K }
    }

    pub fn new_with_gains(k: [f32; 4]) -> Self {
        Self { k }
    }

    /// Renvoies l'angle de virage en rad
    /// Positif = left, negatif = right
    pub fn compute_lateral(&self, state: &LqrState) -> f32 {
        let mat = state.as_array();
        let u: f32 = self.k.iter().zip(mat.iter()).map(|(k, x)| k * x).sum();
        u.clamp(-SERVO_MAX_RAD, SERVO_MAX_RAD)
    }

    /// Pour tuner en realtime avec un UDP)
    pub fn set_gains(&mut self, k: [f32; 4]) {
        self.k = k;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        #[test]
        fn compute_equals_dot_and_clamped(
            k in prop::collection::vec(-10f32..10f32, 4),
            state_arr in prop::collection::vec(-100f32..100f32, 4)
        ) {
            let k_arr: [f32;4] = [k[0], k[1], k[2], k[3]];
            let state = LqrState {
                lateral_error_m: state_arr[0],
                lateral_rate_m_s: state_arr[1],
                heading_error_rad: state_arr[2],
                yaw_rate_rad_s: state_arr[3],
            };
            let lqr = Lqr::new_with_gains(k_arr);
            let dot: f32 = k_arr.iter().zip(state.as_array().iter()).map(|(a,b)| a*b).sum();
            let expected = dot.clamp(-SERVO_MAX_RAD, SERVO_MAX_RAD);
            let out = lqr.compute_lateral(&state);
            prop_assert!( (out - expected).abs() <= 1e-6 );
        }
    }

    #[test]
    fn output_clamped_to_servo_range() {
        let lqr = Lqr::new();
        let extreme = LqrState {
            lateral_error_m: 100.0,
            lateral_rate_m_s: 100.0,
            heading_error_rad: 100.0,
            yaw_rate_rad_s: 100.0,
        };
        let steering = lqr.compute_lateral(&extreme);
        assert!(steering.abs() <= SERVO_MAX_RAD);
    }

    #[test]
    fn zero_state_gives_zero_steering() {
        let lqr = Lqr::new();
        let zero = LqrState {
            lateral_error_m: 0.0,
            lateral_rate_m_s: 0.0,
            heading_error_rad: 0.0,
            yaw_rate_rad_s: 0.0,
        };
        assert_eq!(lqr.compute_lateral(&zero), 0.0);
    }
}
