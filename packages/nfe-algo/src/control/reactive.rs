use nfe_core::control::{ControlInput, ControlOutput, Controller, ControllerStatus};
use nfe_core::params::Tunable;

use super::lqr::{Lqr, LqrParams, LqrState};
use super::pid::{Pid, PidParams};
use super::speed::{SpeedParams, SpeedPlanner};

#[derive(Clone, Debug, Default, serde::Deserialize, serde::Serialize, Tunable)]
#[serde(default)]
pub struct ReactiveControllerParams {
    #[tunable(nested)]
    pub lqr: LqrParams,
    #[tunable(nested)]
    pub pid: PidParams,
    #[tunable(nested)]
    pub speed: SpeedParams,
}

pub struct ReactiveController {
    lqr: Lqr,
    pid: Pid,
    speed: SpeedPlanner,
}

impl ReactiveController {
    pub fn new(params: ReactiveControllerParams) -> Self {
        Self {
            lqr: Lqr::new(params.lqr),
            pid: Pid::new(params.pid),
            speed: SpeedPlanner::new(params.speed),
        }
    }
}

impl Controller for ReactiveController {
    fn reset(&mut self) {
        self.pid.reset();
    }

    fn compute(&mut self, input: &ControlInput<'_>) -> ControlOutput {
        let Some(corridor) = input.corridor else {
            return ControlOutput {
                status: ControllerStatus::Unavailable,
                ..Default::default()
            };
        };

        let steering = self.lqr.compute(LqrState {
            lateral_error_m: corridor.lateral_error_m,
            lateral_rate_m_s: corridor.lateral_rate_m_s,
            heading_error_rad: corridor.heading_error_rad,
            yaw_rate_rad_s: input.motion.yaw_rate_rad_s,
        });
        let target_speed = self.speed.compute(Some(corridor));
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
