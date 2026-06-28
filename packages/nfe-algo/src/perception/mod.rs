//! Perception: turn a LIDAR cloud into a track error signal.
//!
//! `ransac` is the shared wall-extraction path used by both the reactive
//! controller and the mapping task. Reactive perception consumes fitted walls
//! internally to produce the crosstrack/heading error the controller tracks;
//! optional observers can inspect intermediate geometry without making it part
//! of the control output.

use nfe_core::sensors::LidarPoint;
use nfe_core::{Point2, WallLine};

pub mod apex;
pub mod corridor;
pub mod ransac;

pub struct RansacWallsObservation<'a> {
    pub timestamp_us: u64,
    pub points: &'a [Point2],
    pub walls: &'a [WallLine],
    pub confidence: f32,
}

pub struct ApexObservation<'a> {
    pub timestamp_us: u64,
    pub apex: &'a LidarPoint,
    pub opposite: &'a LidarPoint,
    pub target: &'a LidarPoint,
    pub cartesian_midpoint: &'a LidarPoint,
    pub filtered_points: &'a [LidarPoint],
    pub range_jump_m: f32,
    pub derivative_score: f32,
    pub confidence: f32,
}

pub trait PerceptionObserver {
    fn wants_ransac_walls(&self) -> bool {
        false
    }

    fn ransac_walls(&mut self, _event: RansacWallsObservation<'_>) {}

    fn wants_apex(&self) -> bool {
        false
    }

    fn apex(&mut self, _event: ApexObservation<'_>) {}
}

pub struct NoopPerceptionObserver;

impl PerceptionObserver for NoopPerceptionObserver {}
