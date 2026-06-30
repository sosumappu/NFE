use nfe_core::control::{ControlInput, ControlOutput, Controller, ControllerStatus};
use nfe_core::params::Tunable;

use super::pid::{Pid, PidParams};
use super::speed::{SpeedParams, SpeedPlanner};
use super::stanley::{Stanley, StanleyParams};

#[derive(Clone, Debug, Default, serde::Deserialize, serde::Serialize, Tunable)]
#[serde(default)]
pub struct ReactiveStanleyControllerParams {
    #[tunable(nested)]
    pub stanley: StanleyParams,
    #[tunable(nested)]
    pub pid: PidParams,
    #[tunable(nested)]
    pub speed: SpeedParams,
}

pub struct ReactiveStanleyController {
    stanley: Stanley,
    pid: Pid,
    speed: SpeedPlanner,
}

impl ReactiveStanleyController {
    pub fn new(params: ReactiveStanleyControllerParams) -> Self {
        Self {
            stanley: Stanley::new(params.stanley),
            pid: Pid::new(params.pid),
            speed: SpeedPlanner::new(params.speed),
        }
    }
}

impl Controller for ReactiveStanleyController {
    fn reset(&mut self) {
        self.pid.reset();
        self.speed.reset();
    }

    fn compute(&mut self, input: &ControlInput<'_>) -> ControlOutput {
        let Some(corridor) = input.corridor else {
            return ControlOutput {
                status: ControllerStatus::Unavailable,
                ..Default::default()
            };
        };

        let steering = self.stanley.compute(
            input.motion.speed_ms,
            corridor.lateral_error_m,
            corridor.heading_error_rad,
        );
        let target_speed = self.speed.compute(Some(corridor), input.dt_s);
        let throttle = self
            .pid
            .compute(target_speed - input.motion.speed_ms, input.dt_s);

        ControlOutput {
            steering_rad: steering,
            throttle,
            target_speed_ms: target_speed,
            status: if corridor.confidence > 0.3 {
                ControllerStatus::Nominal
            } else {
                ControllerStatus::Degraded
            },
        }
    }
}
