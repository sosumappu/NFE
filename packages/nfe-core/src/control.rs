//! Controller trait boundary.
//!
//! A controller is pure control law: it consumes estimator/perception/reference
//! state and returns desired commands. It never reads sensors, never writes
//! actuators, and never decides drive mode.

use crate::estimation::StateEstimate;
use crate::raceline::RaceReference;
use crate::{MotionState, Pose2, WallLine};

/// Reactive corridor estimate consumed by wall-following controllers.
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct CorridorEstimate {
    pub lateral_error_m: f32,
    pub lateral_rate_m_s: f32,
    pub heading_error_rad: f32,
    pub nearest_obstacle_m: f32,
    /// Perception confidence in [0,1].
    pub confidence: f32,
    /// Fitted walls in the current frame; optional consumers can inspect them.
    pub walls: Vec<WallLine>,
}

/// One control-law input at a deterministic pipeline tick.
#[derive(Clone, Debug)]
pub struct ControlInput<'a> {
    pub dt_s: f32,
    pub pose: Pose2,
    pub motion: MotionState,
    pub estimate: &'a StateEstimate,
    pub corridor: Option<&'a CorridorEstimate>,
    pub race_reference: Option<&'a RaceReference>,
}

/// Controller health/status independent of the supervisor.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ControllerStatus {
    #[default]
    Nominal,
    Degraded,
    Unavailable,
}

/// Command produced by a controller before runtime safety clamping/actuation.
#[derive(Clone, Copy, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct ControlOutput {
    /// Steering angle [rad], positive = left.
    pub steering_rad: f32,
    /// Normalized throttle [-1,1]. Coupled controllers may set this directly.
    pub throttle: f32,
    /// Target speed [m/s] used for telemetry/tuning even if throttle is direct.
    pub target_speed_ms: f32,
    pub status: ControllerStatus,
}

pub trait Controller {
    fn reset(&mut self);
    fn compute(&mut self, input: &ControlInput<'_>) -> ControlOutput;
}
