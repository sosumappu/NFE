use nfe_core::params::Tunable;

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize, Tunable)]
#[serde(default)]
pub struct LqrParams {
    #[param(0.0..10.0, default = 0.80)]
    pub k_lateral: f32,
    #[param(0.0..10.0, default = 0.30)]
    pub k_lateral_rate: f32,
    #[param(0.0..10.0, default = 1.20)]
    pub k_heading: f32,
    #[param(0.0..10.0, default = 0.40)]
    pub k_yaw_rate: f32,
    #[param(0.05..1.2, default = 0.70)]
    pub max_steering_rad: f32,
}

impl Default for LqrParams {
    fn default() -> Self {
        Self {
            k_lateral: 0.80,
            k_lateral_rate: 0.30,
            k_heading: 1.20,
            k_yaw_rate: 0.40,
            max_steering_rad: 0.70,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct LqrState {
    pub lateral_error_m: f32,
    pub lateral_rate_m_s: f32,
    pub heading_error_rad: f32,
    pub yaw_rate_rad_s: f32,
}

#[derive(Clone, Debug)]
pub struct Lqr {
    params: LqrParams,
}

impl Lqr {
    pub fn new(params: LqrParams) -> Self {
        Self { params }
    }

    pub fn compute(&self, state: LqrState) -> f32 {
        let u = self.params.k_lateral * state.lateral_error_m
            + self.params.k_lateral_rate * state.lateral_rate_m_s
            + self.params.k_heading * state.heading_error_rad
            + self.params.k_yaw_rate * state.yaw_rate_rad_s;
        u.clamp(-self.params.max_steering_rad, self.params.max_steering_rad)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_state_outputs_zero() {
        let lqr = Lqr::new(LqrParams::default());
        assert_eq!(lqr.compute(LqrState::default()), 0.0);
    }

    #[test]
    fn clamps_to_limit() {
        let lqr = Lqr::new(LqrParams::default());
        let out = lqr.compute(LqrState {
            lateral_error_m: 100.0,
            lateral_rate_m_s: 0.0,
            heading_error_rad: 0.0,
            yaw_rate_rad_s: 0.0,
        });
        assert!(out <= LqrParams::default().max_steering_rad);
    }
}
