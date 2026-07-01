use nfe_core::params::Tunable;
use nfe_core::wrap_angle;

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize, Tunable)]
#[serde(default)]
pub struct HighLevelLateralParams {
    /// Natural frequency of the second-order lateral-error dynamics [rad/s].
    #[param(0.1..10.0, default = 2.0)]
    pub natural_frequency_rad_s: f32,
    /// Damping ratio of the second-order lateral-error dynamics.
    #[param(0.1..2.0, default = 0.53)]
    pub damping_ratio: f32,
    /// Feedback fades in below this speed and is fully active at/above it.
    #[param(0.05..3.0, default = 0.5)]
    pub min_speed_ms: f32,
    /// Symmetric cap on feedback lateral acceleration [m/s²].
    #[param(0.1..30.0, default = 8.0)]
    pub max_feedback_accel_ms2: f32,
    /// Symmetric cap on the high-level curvature request [1/m].
    #[param(0.1..50.0, default = 20.0)]
    pub max_curvature_command_m_inv: f32,
}

impl Default for HighLevelLateralParams {
    fn default() -> Self {
        Self {
            natural_frequency_rad_s: 2.0,
            damping_ratio: 0.53,
            min_speed_ms: 0.5,
            max_feedback_accel_ms2: 8.0,
            max_curvature_command_m_inv: 20.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HighLevelLateralInput {
    /// Path-relative lateral displacement, positive left of the reference path [m].
    pub lateral_error_m: f32,
    /// Vehicle heading minus reference-path heading, positive counter-clockwise [rad].
    pub heading_error_rad: f32,
    /// Longitudinal speed used for Frenet-rate reconstruction [m/s].
    pub speed_ms: f32,
    /// Curvature feedforward from the raceline geometry [1/m].
    pub reference_curvature_m_inv: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HighLevelLateralOutput {
    /// Reconstructed lateral-error derivative using β = 0 [m/s].
    pub lateral_rate_m_s: f32,
    /// Unclamped second-order feedback acceleration [m/s²].
    pub raw_feedback_accel_ms2: f32,
    /// Clamped second-order feedback acceleration before low-speed scaling [m/s²].
    pub clamped_feedback_accel_ms2: f32,
    /// Effective feedback acceleration after low-speed scaling [m/s²].
    pub feedback_accel_ms2: f32,
    /// Quadratic fade applied to feedback below `min_speed_ms`.
    pub low_speed_feedback_scale: f32,
    /// Geometric raceline feedforward lateral acceleration [m/s²].
    pub feedforward_accel_ms2: f32,
    /// Sum of feedforward and feedback lateral acceleration [m/s²].
    pub target_lateral_accel_ms2: f32,
    /// High-level curvature request before low-level steering mapping [1/m].
    pub curvature_command_m_inv: f32,
    pub feedback_saturated: bool,
    pub curvature_saturated: bool,
}

#[derive(Clone, Debug)]
pub struct HighLevelLateralController {
    params: HighLevelLateralParams,
}

impl HighLevelLateralController {
    pub fn new(params: HighLevelLateralParams) -> Self {
        Self { params }
    }

    pub fn params(&self) -> &HighLevelLateralParams {
        &self.params
    }

    pub fn compute(&self, input: HighLevelLateralInput) -> HighLevelLateralOutput {
        compute_high_level_lateral(&input, &self.params)
    }
}

pub fn compute_high_level_lateral(
    input: &HighLevelLateralInput,
    params: &HighLevelLateralParams,
) -> HighLevelLateralOutput {
    let speed = finite_or_zero(input.speed_ms).max(0.0);
    let min_speed = params.min_speed_ms.max(1.0e-3);
    let speed_for_curvature = speed.max(min_speed);
    let heading_error = wrap_angle(finite_or_zero(input.heading_error_rad));
    let lateral_error = finite_or_zero(input.lateral_error_m);
    let reference_curvature = finite_or_zero(input.reference_curvature_m_inv);

    let omega = params.natural_frequency_rad_s.max(0.0);
    let damping = params.damping_ratio.max(0.0);
    let lateral_rate = speed * heading_error.sin();
    let raw_feedback_accel = -omega * omega * lateral_error - 2.0 * damping * omega * lateral_rate;
    let max_feedback = params.max_feedback_accel_ms2.max(0.0);
    let clamped_feedback_accel = raw_feedback_accel.clamp(-max_feedback, max_feedback);
    let feedback_saturated = (clamped_feedback_accel - raw_feedback_accel).abs() > 1.0e-6;
    let low_speed_feedback_scale = (speed / min_speed).clamp(0.0, 1.0).powi(2);
    let feedback_accel = clamped_feedback_accel * low_speed_feedback_scale;

    let feedforward_accel = speed * speed * reference_curvature;
    let target_lateral_accel = feedforward_accel + feedback_accel;
    let raw_curvature_command =
        reference_curvature + feedback_accel / (speed_for_curvature * speed_for_curvature);
    let max_curvature = params.max_curvature_command_m_inv.max(0.0);
    let curvature_command = raw_curvature_command.clamp(-max_curvature, max_curvature);
    let curvature_saturated = (curvature_command - raw_curvature_command).abs() > 1.0e-6;

    HighLevelLateralOutput {
        lateral_rate_m_s: lateral_rate,
        raw_feedback_accel_ms2: raw_feedback_accel,
        clamped_feedback_accel_ms2: clamped_feedback_accel,
        feedback_accel_ms2: feedback_accel,
        low_speed_feedback_scale,
        feedforward_accel_ms2: feedforward_accel,
        target_lateral_accel_ms2: target_lateral_accel,
        curvature_command_m_inv: curvature_command,
        feedback_saturated,
        curvature_saturated,
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

    fn params() -> HighLevelLateralParams {
        HighLevelLateralParams {
            max_feedback_accel_ms2: 100.0,
            max_curvature_command_m_inv: 100.0,
            ..HighLevelLateralParams::default()
        }
    }

    #[test]
    fn centered_vehicle_preserves_reference_curvature() {
        let controller = HighLevelLateralController::new(params());
        let out = controller.compute(HighLevelLateralInput {
            speed_ms: 3.0,
            reference_curvature_m_inv: 0.4,
            ..Default::default()
        });

        assert!(
            (out.curvature_command_m_inv - 0.4).abs() < 1.0e-6,
            "{out:?}"
        );
        assert!((out.feedforward_accel_ms2 - 3.6).abs() < 1.0e-6, "{out:?}");
        assert_eq!(out.feedback_accel_ms2, 0.0);
    }

    #[test]
    fn positive_lateral_error_commands_rightward_curvature_correction() {
        let controller = HighLevelLateralController::new(params());
        let out = controller.compute(HighLevelLateralInput {
            lateral_error_m: 0.5,
            speed_ms: 2.0,
            ..Default::default()
        });

        assert!(out.feedback_accel_ms2 < 0.0, "{out:?}");
        assert!(out.curvature_command_m_inv < 0.0, "{out:?}");
    }

    #[test]
    fn heading_error_reconstructs_lateral_rate() {
        let controller = HighLevelLateralController::new(params());
        let out = controller.compute(HighLevelLateralInput {
            heading_error_rad: 0.1,
            speed_ms: 4.0,
            ..Default::default()
        });

        assert!(
            (out.lateral_rate_m_s - 4.0 * 0.1_f32.sin()).abs() < 1.0e-6,
            "{out:?}"
        );
        assert!(out.feedback_accel_ms2 < 0.0, "{out:?}");
    }

    #[test]
    fn gain_scheduling_reduces_curvature_correction_at_higher_speed() {
        let controller = HighLevelLateralController::new(params());
        let slow = controller.compute(HighLevelLateralInput {
            lateral_error_m: 0.2,
            speed_ms: 1.0,
            ..Default::default()
        });
        let fast = controller.compute(HighLevelLateralInput {
            lateral_error_m: 0.2,
            speed_ms: 4.0,
            ..Default::default()
        });

        assert!(
            fast.curvature_command_m_inv.abs() < slow.curvature_command_m_inv.abs(),
            "slow={slow:?} fast={fast:?}"
        );
        assert!((fast.feedback_accel_ms2 - slow.feedback_accel_ms2).abs() < 1.0e-6);
    }

    #[test]
    fn feedback_and_curvature_limits_are_reported() {
        let controller = HighLevelLateralController::new(HighLevelLateralParams {
            max_feedback_accel_ms2: 1.0,
            max_curvature_command_m_inv: 0.5,
            ..HighLevelLateralParams::default()
        });
        let out = controller.compute(HighLevelLateralInput {
            lateral_error_m: 10.0,
            speed_ms: 1.0,
            ..Default::default()
        });

        assert_eq!(out.clamped_feedback_accel_ms2, -1.0);
        assert_eq!(out.feedback_accel_ms2, -1.0);
        assert_eq!(out.curvature_command_m_inv, -0.5);
        assert!(out.feedback_saturated);
        assert!(out.curvature_saturated);
    }

    #[test]
    fn stopped_vehicle_suppresses_lateral_feedback_curvature() {
        let controller = HighLevelLateralController::new(params());
        let out = controller.compute(HighLevelLateralInput {
            lateral_error_m: 0.5,
            speed_ms: 0.0,
            ..Default::default()
        });

        assert!(out.raw_feedback_accel_ms2 < 0.0, "{out:?}");
        assert!(out.clamped_feedback_accel_ms2 < 0.0, "{out:?}");
        assert_eq!(out.low_speed_feedback_scale, 0.0);
        assert_eq!(out.feedback_accel_ms2, 0.0);
        assert_eq!(out.curvature_command_m_inv, 0.0);
    }

    #[test]
    fn closed_loop_step_response_matches_second_order_shape() {
        let params = params();
        let controller = HighLevelLateralController::new(params.clone());
        let dt = 0.001_f32;
        let speed = 5.0_f32;
        let initial_d = 0.5_f32;
        let mut d = initial_d;
        let mut d_dot = 0.0;
        let mut samples = Vec::new();
        let sample_times = [0.5, 1.0, 2.0, 3.0, 4.0];
        let mut next_sample = 0;
        let steps = (4.0_f32 / dt) as usize;
        for step in 0..=steps {
            let t = step as f32 * dt;
            if next_sample < sample_times.len() && t >= sample_times[next_sample] {
                samples.push((t, d));
                next_sample += 1;
            }
            let heading_error = (d_dot / speed).clamp(-0.99, 0.99).asin();
            let out = controller.compute(HighLevelLateralInput {
                lateral_error_m: d,
                heading_error_rad: heading_error,
                speed_ms: speed,
                ..Default::default()
            });
            d_dot += out.feedback_accel_ms2 * dt;
            d += d_dot * dt;
        }

        for (t, actual) in samples {
            let expected = second_order_step_error(
                initial_d,
                params.natural_frequency_rad_s,
                params.damping_ratio,
                t,
            );
            assert!(
                (actual - expected).abs() < 0.01,
                "t={t} actual={actual} expected={expected}"
            );
        }
        assert!(d.abs() < 0.03, "d={d} d_dot={d_dot}");
    }

    fn second_order_step_error(d0: f32, omega: f32, damping: f32, t: f32) -> f32 {
        let omega_d = omega * (1.0 - damping * damping).sqrt();
        let envelope = (-damping * omega * t).exp();
        let phase = omega_d * t;
        d0 * envelope * (phase.cos() + damping / (1.0 - damping * damping).sqrt() * phase.sin())
    }
}
