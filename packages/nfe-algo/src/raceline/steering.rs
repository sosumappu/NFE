use nfe_core::params::Tunable;

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize, Tunable)]
#[serde(default)]
pub struct LowLevelSteeringParams {
    /// Effective wheelbase used by the geometric curvature feedforward [m].
    #[param(0.05..0.6, default = 0.21)]
    pub wheelbase_m: f32,
    /// Steering correction per yaw-rate error [rad / (rad/s)].
    #[param(0.0..2.0, default = 0.08)]
    pub yaw_rate_p_gain_s: f32,
    /// Yaw-rate feedback fades in below this speed and is fully active at/above it.
    #[param(0.05..3.0, default = 0.5)]
    pub min_feedback_speed_ms: f32,
    #[param(0.05..1.2, default = 0.70)]
    pub max_steering_rad: f32,
}

impl Default for LowLevelSteeringParams {
    fn default() -> Self {
        Self {
            wheelbase_m: 0.21,
            yaw_rate_p_gain_s: 0.08,
            min_feedback_speed_ms: 0.5,
            max_steering_rad: 0.70,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LowLevelSteeringInput {
    pub curvature_command_m_inv: f32,
    pub speed_ms: f32,
    pub measured_yaw_rate_rad_s: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LowLevelSteeringOutput {
    pub target_yaw_rate_rad_s: f32,
    pub yaw_rate_error_rad_s: f32,
    pub feedback_scale: f32,
    pub feedforward_steering_rad: f32,
    pub feedback_steering_rad: f32,
    pub raw_steering_rad: f32,
    pub steering_rad: f32,
    pub saturated: bool,
}

#[derive(Clone, Debug)]
pub struct LowLevelSteeringController {
    params: LowLevelSteeringParams,
}

impl LowLevelSteeringController {
    pub fn new(params: LowLevelSteeringParams) -> Self {
        Self { params }
    }

    pub fn params(&self) -> &LowLevelSteeringParams {
        &self.params
    }

    pub fn compute(&self, input: LowLevelSteeringInput) -> LowLevelSteeringOutput {
        compute_low_level_steering(&input, &self.params)
    }
}

pub fn compute_low_level_steering(
    input: &LowLevelSteeringInput,
    params: &LowLevelSteeringParams,
) -> LowLevelSteeringOutput {
    let curvature = finite_or_zero(input.curvature_command_m_inv);
    let speed = finite_or_zero(input.speed_ms).max(0.0);
    let measured_yaw_rate = finite_or_zero(input.measured_yaw_rate_rad_s);
    let wheelbase = params.wheelbase_m.max(1.0e-3);
    let target_yaw_rate = speed * curvature;
    let yaw_rate_error = target_yaw_rate - measured_yaw_rate;
    let min_speed = params.min_feedback_speed_ms.max(1.0e-3);
    let feedback_scale = (speed / min_speed).clamp(0.0, 1.0);

    let feedforward = (wheelbase * curvature).atan();
    let feedback = params.yaw_rate_p_gain_s.max(0.0) * yaw_rate_error * feedback_scale;
    let raw = feedforward + feedback;
    let limit = params.max_steering_rad.max(0.0);
    let steering = raw.clamp(-limit, limit);
    let saturated = (steering - raw).abs() > 1.0e-6;

    LowLevelSteeringOutput {
        target_yaw_rate_rad_s: target_yaw_rate,
        yaw_rate_error_rad_s: yaw_rate_error,
        feedback_scale,
        feedforward_steering_rad: feedforward,
        feedback_steering_rad: feedback,
        raw_steering_rad: raw,
        steering_rad: steering,
        saturated,
    }
}

fn finite_or_zero(value: f32) -> f32 {
    if value.is_finite() {
        value
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn geometric_feedforward_matches_bicycle_curvature() {
        let controller = LowLevelSteeringController::new(LowLevelSteeringParams {
            yaw_rate_p_gain_s: 0.0,
            wheelbase_m: 0.21,
            ..Default::default()
        });
        let out = controller.compute(LowLevelSteeringInput {
            curvature_command_m_inv: 2.0,
            speed_ms: 3.0,
            ..Default::default()
        });

        assert!((out.feedforward_steering_rad - (0.21_f32 * 2.0).atan()).abs() < 1.0e-6);
        assert_eq!(out.steering_rad, out.feedforward_steering_rad);
        assert_eq!(out.target_yaw_rate_rad_s, 6.0);
    }

    #[test]
    fn yaw_rate_p_feedback_reduces_tracking_error() {
        let controller = LowLevelSteeringController::new(LowLevelSteeringParams {
            yaw_rate_p_gain_s: 0.1,
            ..Default::default()
        });
        let nominal = controller.compute(LowLevelSteeringInput {
            curvature_command_m_inv: 1.0,
            speed_ms: 3.0,
            measured_yaw_rate_rad_s: 3.0,
        });
        let understeer = controller.compute(LowLevelSteeringInput {
            curvature_command_m_inv: 1.0,
            speed_ms: 3.0,
            measured_yaw_rate_rad_s: 2.0,
        });

        assert!(understeer.steering_rad > nominal.steering_rad);
        assert!(understeer.feedback_steering_rad > 0.0);
    }

    #[test]
    fn stopped_vehicle_keeps_feedforward_but_suppresses_yaw_feedback() {
        let controller = LowLevelSteeringController::new(LowLevelSteeringParams {
            yaw_rate_p_gain_s: 0.5,
            ..Default::default()
        });
        let out = controller.compute(LowLevelSteeringInput {
            curvature_command_m_inv: 1.0,
            speed_ms: 0.0,
            measured_yaw_rate_rad_s: -10.0,
        });

        assert_eq!(out.feedback_scale, 0.0);
        assert_eq!(out.feedback_steering_rad, 0.0);
        assert!((out.steering_rad - out.feedforward_steering_rad).abs() < 1.0e-6);
    }

    #[test]
    fn steering_limit_reports_saturation() {
        let controller = LowLevelSteeringController::new(LowLevelSteeringParams {
            max_steering_rad: 0.2,
            ..Default::default()
        });
        let out = controller.compute(LowLevelSteeringInput {
            curvature_command_m_inv: 100.0,
            speed_ms: 5.0,
            ..Default::default()
        });

        assert_eq!(out.steering_rad, 0.2);
        assert!(out.saturated);
    }
}
