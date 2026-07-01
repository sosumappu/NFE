use nfe_core::params::Tunable;

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize, Tunable)]
#[serde(default)]
pub struct LongitudinalForceParams {
    /// Vehicle mass used for force diagnostics [kg].
    #[param(0.2..10.0, default = 1.52)]
    pub mass_kg: f32,
    /// Normalized positive throttle to longitudinal acceleration mapping [m/s²].
    #[param(0.1..50.0, default = 20.0)]
    pub motor_gain_ms2: f32,
    /// Negative throttle gain multiplier.
    #[param(0.1..5.0, default = 1.4)]
    pub brake_gain: f32,
    /// Quadratic drag coefficient [m/s² per (m/s)²].
    #[param(0.0..5.0, default = 0.5)]
    pub drag_k: f32,
    /// Proportional speed-error feedback as acceleration [m/s² per m/s].
    #[param(0.0..20.0, default = 4.0)]
    pub k_speed_ms2_per_ms: f32,
    #[param(0.0..30.0, default = 12.0)]
    pub max_accel_ms2: f32,
    #[param(0.0..30.0, default = 16.0)]
    pub max_brake_ms2: f32,
    #[param(0.0..0.5, default = 0.05)]
    pub motor_deadband: f32,
    #[param(0.0..1.0, default = 1.0)]
    pub max_throttle: f32,
}

impl Default for LongitudinalForceParams {
    fn default() -> Self {
        Self {
            mass_kg: 1.52,
            motor_gain_ms2: 20.0,
            brake_gain: 1.4,
            drag_k: 0.5,
            k_speed_ms2_per_ms: 4.0,
            max_accel_ms2: 12.0,
            max_brake_ms2: 16.0,
            motor_deadband: 0.05,
            max_throttle: 1.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LongitudinalForceInput {
    pub target_speed_ms: f32,
    pub current_speed_ms: f32,
    /// Feedforward acceleration from the velocity profile [m/s²].
    pub feedforward_accel_ms2: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LongitudinalForceOutput {
    pub speed_error_ms: f32,
    pub feedback_accel_ms2: f32,
    pub raw_target_accel_ms2: f32,
    pub target_accel_ms2: f32,
    pub drag_comp_accel_ms2: f32,
    pub tractive_accel_ms2: f32,
    pub target_force_n: f32,
    pub throttle: f32,
    pub accel_saturated: bool,
    pub throttle_saturated: bool,
}

#[derive(Clone, Debug)]
pub struct LongitudinalForceController {
    params: LongitudinalForceParams,
}

impl LongitudinalForceController {
    pub fn new(params: LongitudinalForceParams) -> Self {
        Self { params }
    }

    pub fn params(&self) -> &LongitudinalForceParams {
        &self.params
    }

    pub fn compute(&self, input: LongitudinalForceInput) -> LongitudinalForceOutput {
        compute_longitudinal_force(&input, &self.params)
    }
}

pub fn compute_longitudinal_force(
    input: &LongitudinalForceInput,
    params: &LongitudinalForceParams,
) -> LongitudinalForceOutput {
    let target_speed = finite_or_zero(input.target_speed_ms).max(0.0);
    let current_speed = finite_or_zero(input.current_speed_ms).max(0.0);
    let feedforward_accel = finite_or_zero(input.feedforward_accel_ms2);
    let speed_error = target_speed - current_speed;
    let feedback_accel = params.k_speed_ms2_per_ms.max(0.0) * speed_error;
    let raw_target_accel = feedforward_accel + feedback_accel;
    let target_accel = raw_target_accel.clamp(
        -params.max_brake_ms2.max(0.0),
        params.max_accel_ms2.max(0.0),
    );
    let accel_saturated = (target_accel - raw_target_accel).abs() > 1.0e-6;
    let drag_comp = params.drag_k.max(0.0) * current_speed * current_speed;
    let tractive_accel = target_accel + drag_comp;
    let effective_throttle = if tractive_accel >= 0.0 {
        tractive_accel / params.motor_gain_ms2.max(1.0e-3)
    } else {
        tractive_accel / (params.motor_gain_ms2.max(1.0e-3) * params.brake_gain.max(1.0e-3))
    };
    let raw_throttle = deadband_inverse(effective_throttle, params.motor_deadband);
    let max_throttle = params.max_throttle.clamp(0.0, 1.0);
    let throttle = raw_throttle.clamp(-max_throttle, max_throttle);
    let target_force = params.mass_kg.max(0.0) * target_accel;
    let throttle_saturated = (throttle - raw_throttle).abs() > 1.0e-6;

    LongitudinalForceOutput {
        speed_error_ms: speed_error,
        feedback_accel_ms2: feedback_accel,
        raw_target_accel_ms2: raw_target_accel,
        target_accel_ms2: target_accel,
        drag_comp_accel_ms2: drag_comp,
        tractive_accel_ms2: tractive_accel,
        target_force_n: target_force,
        throttle,
        accel_saturated,
        throttle_saturated,
    }
}

fn deadband_inverse(effective: f32, deadband: f32) -> f32 {
    if effective.abs() <= 1.0e-6 {
        return 0.0;
    }
    let deadband = deadband.clamp(0.0, 0.99);
    effective.signum() * (deadband + (1.0 - deadband) * effective.abs())
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
    fn zero_error_and_zero_feedforward_coasts_at_zero_speed() {
        let controller = LongitudinalForceController::new(LongitudinalForceParams::default());
        let out = controller.compute(LongitudinalForceInput::default());

        assert_eq!(out.throttle, 0.0);
        assert_eq!(out.target_force_n, 0.0);
    }

    #[test]
    fn acceleration_feedforward_maps_to_positive_throttle() {
        let controller = LongitudinalForceController::new(LongitudinalForceParams {
            motor_gain_ms2: 20.0,
            motor_deadband: 0.0,
            drag_k: 0.0,
            k_speed_ms2_per_ms: 0.0,
            ..Default::default()
        });
        let out = controller.compute(LongitudinalForceInput {
            target_speed_ms: 2.0,
            current_speed_ms: 2.0,
            feedforward_accel_ms2: 4.0,
        });

        assert!((out.throttle - 0.2).abs() < 1.0e-6, "{out:?}");
        assert!((out.target_force_n - 1.52 * 4.0).abs() < 1.0e-6, "{out:?}");
    }

    #[test]
    fn steady_speed_compensates_drag() {
        let controller = LongitudinalForceController::new(LongitudinalForceParams {
            motor_gain_ms2: 20.0,
            motor_deadband: 0.0,
            drag_k: 0.5,
            k_speed_ms2_per_ms: 0.0,
            ..Default::default()
        });
        let out = controller.compute(LongitudinalForceInput {
            target_speed_ms: 4.0,
            current_speed_ms: 4.0,
            feedforward_accel_ms2: 0.0,
        });

        assert!((out.drag_comp_accel_ms2 - 8.0).abs() < 1.0e-6, "{out:?}");
        assert!((out.throttle - 0.4).abs() < 1.0e-6, "{out:?}");
    }

    #[test]
    fn overspeed_commands_braking() {
        let controller = LongitudinalForceController::new(LongitudinalForceParams {
            motor_gain_ms2: 20.0,
            brake_gain: 2.0,
            motor_deadband: 0.0,
            drag_k: 0.0,
            k_speed_ms2_per_ms: 4.0,
            ..Default::default()
        });
        let out = controller.compute(LongitudinalForceInput {
            target_speed_ms: 2.0,
            current_speed_ms: 3.0,
            feedforward_accel_ms2: 0.0,
        });

        assert!(out.target_accel_ms2 < 0.0, "{out:?}");
        assert!(out.throttle < 0.0, "{out:?}");
        assert!((out.throttle + 0.1).abs() < 1.0e-6, "{out:?}");
    }

    #[test]
    fn acceleration_limit_reports_saturation() {
        let controller = LongitudinalForceController::new(LongitudinalForceParams {
            max_accel_ms2: 1.0,
            motor_deadband: 0.0,
            ..Default::default()
        });
        let out = controller.compute(LongitudinalForceInput {
            target_speed_ms: 10.0,
            current_speed_ms: 0.0,
            feedforward_accel_ms2: 20.0,
        });

        assert_eq!(out.target_accel_ms2, 1.0);
        assert!(out.accel_saturated);
    }
}
