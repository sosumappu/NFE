use nfe_core::control::{ControlInput, ControlOutput, Controller, ControllerStatus};
use nfe_core::params::Tunable;
use nfe_core::wrap_angle;

use crate::control::pid::{Pid, PidParams};

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize, Tunable)]
#[serde(default)]
pub struct RaceLineControllerParams {
    #[param(0.0..10.0, default = 1.2)]
    pub k_lateral: f32,
    #[param(0.0..10.0, default = 1.4)]
    pub k_heading: f32,
    #[param(0.0..2.0, default = 0.6)]
    pub k_curvature_ff: f32,
    #[param(0.05..1.2, default = 0.70)]
    pub max_steering_rad: f32,
    #[tunable(nested)]
    pub pid: PidParams,
}

impl Default for RaceLineControllerParams {
    fn default() -> Self {
        Self {
            k_lateral: 1.2,
            k_heading: 1.4,
            k_curvature_ff: 0.6,
            max_steering_rad: 0.70,
            pid: PidParams::default(),
        }
    }
}

pub struct RaceLineController {
    params: RaceLineControllerParams,
    pid: Pid,
}

impl RaceLineController {
    pub fn new(params: RaceLineControllerParams) -> Self {
        let pid = Pid::new(params.pid.clone());
        Self { params, pid }
    }
}

impl Controller for RaceLineController {
    fn reset(&mut self) {
        self.pid.reset();
    }

    fn compute(&mut self, input: &ControlInput<'_>) -> ControlOutput {
        let Some(reference) = input.race_reference else {
            return ControlOutput {
                status: ControllerStatus::Unavailable,
                ..Default::default()
            };
        };
        let heading_error = wrap_angle(reference.heading_error_rad);
        let steering = (self.params.k_lateral * reference.lateral_error_m
            + self.params.k_heading * heading_error
            + self.params.k_curvature_ff * reference.target.curvature)
            .clamp(-self.params.max_steering_rad, self.params.max_steering_rad);
        let target_speed = reference.target.speed_ms;
        let throttle = self
            .pid
            .compute(target_speed - input.motion.speed_ms, input.dt_s);
        ControlOutput {
            steering_rad: steering,
            throttle,
            target_speed_ms: target_speed,
            status: if reference.confidence > 0.5 {
                ControllerStatus::Nominal
            } else {
                ControllerStatus::Degraded
            },
        }
    }
}
