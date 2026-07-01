use nfe_core::control::{ControlInput, ControlOutput, Controller, ControllerStatus};
use nfe_core::params::Tunable;
use nfe_core::wrap_angle;

use crate::raceline::lateral::{
    HighLevelLateralController, HighLevelLateralInput, HighLevelLateralParams,
};
use crate::raceline::longitudinal::{
    LongitudinalForceController, LongitudinalForceInput, LongitudinalForceParams,
};
use crate::raceline::steering::{
    LowLevelSteeringController, LowLevelSteeringInput, LowLevelSteeringParams,
};

#[derive(Clone, Debug, Default, serde::Deserialize, serde::Serialize, Tunable)]
#[serde(default)]
pub struct RaceLineControllerParams {
    #[tunable(nested)]
    pub lateral: HighLevelLateralParams,
    #[tunable(nested)]
    pub steering: LowLevelSteeringParams,
    #[tunable(nested)]
    pub longitudinal: LongitudinalForceParams,
}

pub struct RaceLineController {
    lateral: HighLevelLateralController,
    steering: LowLevelSteeringController,
    longitudinal: LongitudinalForceController,
}

impl RaceLineController {
    pub fn new(params: RaceLineControllerParams) -> Self {
        Self {
            lateral: HighLevelLateralController::new(params.lateral),
            steering: LowLevelSteeringController::new(params.steering),
            longitudinal: LongitudinalForceController::new(params.longitudinal),
        }
    }

    pub fn lateral_controller(&self) -> &HighLevelLateralController {
        &self.lateral
    }

    pub fn steering_controller(&self) -> &LowLevelSteeringController {
        &self.steering
    }

    pub fn longitudinal_controller(&self) -> &LongitudinalForceController {
        &self.longitudinal
    }
}

impl Controller for RaceLineController {
    fn reset(&mut self) {}

    fn compute(&mut self, input: &ControlInput<'_>) -> ControlOutput {
        let Some(reference) = input.race_reference else {
            return ControlOutput {
                status: ControllerStatus::Unavailable,
                ..Default::default()
            };
        };
        let lateral = self.lateral.compute(HighLevelLateralInput {
            lateral_error_m: reference.lateral_error_m,
            heading_error_rad: wrap_angle(-reference.heading_error_rad),
            speed_ms: input.motion.speed_ms,
            reference_curvature_m_inv: reference.target.curvature,
        });
        let steering = self.steering.compute(LowLevelSteeringInput {
            curvature_command_m_inv: lateral.curvature_command_m_inv,
            speed_ms: input.motion.speed_ms,
            measured_yaw_rate_rad_s: input.motion.yaw_rate_rad_s,
        });
        let longitudinal = self.longitudinal.compute(LongitudinalForceInput {
            target_speed_ms: reference.target.speed_ms,
            current_speed_ms: input.motion.speed_ms,
            feedforward_accel_ms2: reference.target.accel_x_ms2,
        });
        ControlOutput {
            steering_rad: steering.steering_rad,
            throttle: longitudinal.throttle,
            target_speed_ms: reference.target.speed_ms,
            status: if reference.confidence > 0.5 {
                ControllerStatus::Nominal
            } else {
                ControllerStatus::Degraded
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nfe_core::control::ControlInput;
    use nfe_core::estimation::StateEstimate;
    use nfe_core::raceline::{RaceLinePoint, RaceReference};
    use nfe_core::{MotionState, Point2, Pose2};

    fn input(reference: RaceReference, speed_ms: f32) -> ControlInput<'static> {
        let estimate = Box::leak(Box::new(StateEstimate::default()));
        ControlInput {
            dt_s: 0.01,
            pose: Pose2::default(),
            motion: MotionState {
                speed_ms,
                yaw_rate_rad_s: 0.0,
            },
            estimate,
            corridor: None,
            race_reference: Some(Box::leak(Box::new(reference))),
        }
    }

    fn reference(lateral_error_m: f32, heading_error_rad: f32, curvature: f32) -> RaceReference {
        RaceReference {
            target: RaceLinePoint {
                p: Point2::new(0.0, 0.0),
                yaw: 0.0,
                curvature,
                speed_ms: 2.0,
                accel_x_ms2: 0.0,
                s_m: 0.0,
            },
            lateral_error_m,
            heading_error_rad,
            confidence: 1.0,
            ..Default::default()
        }
    }

    #[test]
    fn missing_reference_is_unavailable() {
        let estimate = StateEstimate::default();
        let mut controller = RaceLineController::new(RaceLineControllerParams::default());
        let out = controller.compute(&ControlInput {
            dt_s: 0.01,
            pose: Pose2::default(),
            motion: MotionState::default(),
            estimate: &estimate,
            corridor: None,
            race_reference: None,
        });

        assert_eq!(out.status, ControllerStatus::Unavailable);
    }

    #[test]
    fn positive_path_relative_lateral_error_steers_right() {
        let mut controller = RaceLineController::new(RaceLineControllerParams::default());
        let out = controller.compute(&input(reference(0.5, 0.0, 0.0), 2.0));

        assert!(out.steering_rad < 0.0, "{out:?}");
    }

    #[test]
    fn reference_curvature_feeds_through_when_centered() {
        let mut controller = RaceLineController::new(RaceLineControllerParams::default());
        let out = controller.compute(&input(reference(0.0, 0.0, 0.5), 2.0));

        assert!(out.steering_rad > 0.0, "{out:?}");
    }

    #[test]
    fn velocity_profile_acceleration_feeds_throttle() {
        let mut controller = RaceLineController::new(RaceLineControllerParams::default());
        let mut r = reference(0.0, 0.0, 0.0);
        r.target.speed_ms = 2.0;
        r.target.accel_x_ms2 = 4.0;
        let out = controller.compute(&input(r, 2.0));

        assert!(out.throttle > 0.0, "{out:?}");
    }
}
