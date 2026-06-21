//! Cheap estimator for reactive-only operation.
//!
//! This intentionally does not map, localize, allocate per tick, or maintain a
//! covariance. It exists so the reactive fallback can run with the same
//! `StateEstimator` boundary as the EKF path without paying SLAM overhead.

use nfe_core::estimation::{ImuSample, PoseMeasurement, StateEstimate, StateEstimator};
use nfe_core::{wrap_angle, MotionState, Pose2};

#[derive(Clone, Debug, Default)]
pub struct DeadReckonEstimator {
    pose: Pose2,
    vx: f32,
    vy: f32,
    yaw_rate: f32,
    last_timestamp_us: u64,
    confidence: f32,
}

impl DeadReckonEstimator {
    pub fn new() -> Self {
        Self::default()
    }
}

impl StateEstimator for DeadReckonEstimator {
    fn reset(&mut self, pose: Pose2, timestamp_us: u64) {
        self.pose = pose;
        self.vx = 0.0;
        self.vy = 0.0;
        self.yaw_rate = 0.0;
        self.last_timestamp_us = timestamp_us;
        self.confidence = 1.0;
    }

    fn predict_imu(&mut self, sample: ImuSample) {
        let dt = if self.last_timestamp_us == 0 {
            0.0
        } else {
            sample.timestamp_us.saturating_sub(self.last_timestamp_us) as f32 * 1e-6
        };
        self.last_timestamp_us = sample.timestamp_us;
        if !(0.0..=0.25).contains(&dt) {
            return;
        }
        self.yaw_rate = sample.gz;
        let (s, c) = self.pose.yaw.sin_cos();
        self.pose.x += (self.vx * c - self.vy * s) * dt;
        self.pose.y += (self.vx * s + self.vy * c) * dt;
        self.pose.yaw = wrap_angle(self.pose.yaw + sample.gz * dt);
        self.vx += sample.ax * dt;
        self.vy += sample.ay * dt;
        self.confidence = 1.0;
    }

    fn correct_pose(&mut self, measurement: PoseMeasurement) -> bool {
        // Reactive fallback accepts external pose corrections if supplied, but
        // normally no one calls this. Keeping it lets replay tests seed pose.
        self.pose = measurement.pose;
        self.confidence = measurement.quality.clamp(0.0, 1.0);
        true
    }

    fn estimate(&self) -> StateEstimate {
        StateEstimate {
            pose: self.pose,
            motion: MotionState {
                speed_ms: self.vx.hypot(self.vy),
                yaw_rate_rad_s: self.yaw_rate,
            },
            confidence: self.confidence,
            consistency: 0.0,
            diverged: false,
            timestamp_us: self.last_timestamp_us,
        }
    }
}
