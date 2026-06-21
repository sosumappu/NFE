//! State estimation trait boundary.

use crate::{MotionState, Pose2};

/// Raw IMU sample in car body frame.
#[derive(Clone, Copy, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct ImuSample {
    pub ax: f32,
    pub ay: f32,
    pub az: f32,
    pub gx: f32,
    pub gy: f32,
    pub gz: f32,
    pub timestamp_us: u64,
}

/// World-frame pose measurement from scan-matching/localization.
#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct PoseMeasurement {
    pub pose: Pose2,
    /// Measurement quality in [0,1]; lower values inflate measurement noise.
    pub quality: f32,
}

/// Estimator output consumed by controllers, supervisor, mapper, and telemetry.
#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct StateEstimate {
    pub pose: Pose2,
    pub motion: MotionState,
    /// Localization / estimator confidence in [0,1].
    pub confidence: f32,
    /// Last normalized innovation squared or equivalent consistency score.
    pub consistency: f32,
    pub diverged: bool,
    pub timestamp_us: u64,
}

impl Default for StateEstimate {
    fn default() -> Self {
        Self {
            pose: Pose2::default(),
            motion: MotionState::default(),
            confidence: 0.0,
            consistency: 0.0,
            diverged: false,
            timestamp_us: 0,
        }
    }
}

pub trait StateEstimator {
    fn reset(&mut self, pose: Pose2, timestamp_us: u64);
    fn predict_imu(&mut self, sample: ImuSample);
    fn correct_pose(&mut self, measurement: PoseMeasurement) -> bool;
    fn estimate(&self) -> StateEstimate;
}
