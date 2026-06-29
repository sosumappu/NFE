//! Apex-based reactive corridor perception.
//!
//! This path looks for the strongest range discontinuity in the scan. The
//! closest endpoint of that discontinuity is treated as the visible apex; the
//! opposite wall is selected from the other angular side of the LiDAR scan. The
//! reactive target is the polar midpoint of the apex/opposite gap.

use nfe_core::control::CorridorEstimate;
use nfe_core::params::Tunable;
use nfe_core::sensors::LidarCloud;
use std::f32::consts::PI;

use self::geometry::{ApexGeometry, OppositeParams};
use self::scan::{nearest_front_obstacle_m, ApexScan};
use self::tracking::ApexTracker;
use super::{ApexObservation, NoopPerceptionObserver, PerceptionObserver};

mod geometry;
mod scan;
mod tracking;

const APEX_HOLD_MIN_CONFIDENCE: f32 = 0.5;
const APEX_REMEMBER_MIN_CONFIDENCE: f32 = 0.3;

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize, Tunable)]
#[serde(default)]
pub struct ApexParams {
    #[param(int, 1..15, default = 3)]
    pub median_window: usize,

    #[param(int, 4..128, default = 4)]
    pub min_points: usize,

    #[param(0.0..1.0, default = 0.5)]
    pub min_forward_m: f32,

    #[param(0.01..2.0, default = 0.15)]
    pub min_range_jump_m: f32,

    #[param(0.05..2.0, default = 0.05)]
    pub max_opposite_dist_error_m: f32,

    #[tunable(skip)]
    pub prefer_nearer_opposite: bool,

    #[param(0.0..1.0, default = 0.25)]
    pub wall_clearance_m: f32,

    #[param(0.0..PI, default = 0.1)]
    pub apex_switch_threshold_rad: f32,

    #[param(1.0..10.0, default = 2.8)]
    pub apex_switch_hysteresis_factor: f32,

    #[param(1.0..20.0, default = 8.0)]
    pub max_lookahead_m: f32,

    #[param(0.1..5.0, default = 0.5)]
    pub min_lookahead_m: f32,

    #[param(0.0..15.0, default = 5.0)]
    pub lookahead_sensitivity: f32,

    #[param(1.0..360.0, default = 30.0)]
    pub side_lookahead_fov_deg: f32,

    #[param(1.0..360.0, default = 90.0)]
    pub side_lookahead_center_deg: f32,
}

impl Default for ApexParams {
    fn default() -> Self {
        Self {
            median_window: 5,
            min_points: 4,
            min_forward_m: 0.3,
            min_range_jump_m: 0.25,
            max_opposite_dist_error_m: 0.08,
            prefer_nearer_opposite: true,
            wall_clearance_m: 0.15,
            apex_switch_threshold_rad: 0.35,
            apex_switch_hysteresis_factor: 1.8,
            max_lookahead_m: 8.0,
            min_lookahead_m: 0.5,
            lookahead_sensitivity: 5.0,
            side_lookahead_fov_deg: 60.0,
            side_lookahead_center_deg: 90.0,
        }
    }
}

pub trait ApexPerception {
    fn estimate(&mut self, cloud: &LidarCloud, timestamp_us: u64) -> CorridorEstimate {
        let mut observer = NoopPerceptionObserver;
        self.estimate_observed(cloud, timestamp_us, &mut observer)
    }

    fn estimate_observed<O: PerceptionObserver + ?Sized>(
        &mut self,
        cloud: &LidarCloud,
        timestamp_us: u64,
        observer: &mut O,
    ) -> CorridorEstimate;
}

#[derive(Clone, Debug)]
pub struct ApexCorridorPerception {
    params: ApexParams,
    tracker: ApexTracker,
}

impl ApexCorridorPerception {
    pub fn new(params: ApexParams) -> Self {
        Self {
            params,
            tracker: ApexTracker::default(),
        }
    }

    pub fn reset(&mut self) {
        self.tracker.reset();
    }
}

impl ApexPerception for ApexCorridorPerception {
    fn estimate_observed<O: PerceptionObserver + ?Sized>(
        &mut self,
        cloud: &LidarCloud,
        timestamp_us: u64,
        observer: &mut O,
    ) -> CorridorEstimate {
        let safe_lookahead_m = self.calculate_dynamic_lookahead(cloud);
        let filtered = ApexScan::preprocess(
            cloud,
            self.params.min_forward_m,
            safe_lookahead_m,
            self.params.median_window,
        );

        if filtered.len() < self.params.min_points {
            self.tracker.clear_apex_hysteresis();
            return self.tracker.insufficient_points_estimate(cloud);
        }

        let Some(discontinuity) = filtered.find_discontinuity(self.params.min_range_jump_m) else {
            self.tracker.clear_apex_hysteresis();
            return self.tracker.insufficient_points_estimate(cloud);
        };

        let confidence = filtered.confidence_stats(self.params.min_range_jump_m);
        if confidence.confidence < APEX_HOLD_MIN_CONFIDENCE {
            self.tracker.clear_apex_hysteresis();
        }

        if confidence.confidence >= APEX_HOLD_MIN_CONFIDENCE
            && self.tracker.should_hold_previous_apex(
                discontinuity.breakpoint.angle_rad,
                discontinuity.score_m,
                self.params.apex_switch_threshold_rad,
                self.params.apex_switch_hysteresis_factor,
            )
        {
            let confidence = self.tracker.hold_confidence(confidence.confidence);
            return self
                .tracker
                .previous_estimate_with_confidence(cloud, confidence);
        }

        let breakpoint = *discontinuity.breakpoint;
        let opposite_wall = filtered.opposite_wall(&discontinuity);
        let Some(opposite_point) = ApexGeometry::opposite_point(
            opposite_wall,
            &breakpoint,
            timestamp_us,
            self.opposite_params(),
        ) else {
            self.tracker.clear_apex_hysteresis();
            return self.tracker.previous_estimate_with_confidence(cloud, 0.0);
        };

        let target = ApexGeometry::polar_midpoint(breakpoint, opposite_point, timestamp_us);
        if observer.wants_apex() {
            let cartesian_midpoint =
                ApexGeometry::cartesian_midpoint(breakpoint, opposite_point, timestamp_us);
            observer.apex(ApexObservation {
                timestamp_us,
                apex: &breakpoint,
                opposite: &opposite_point,
                target: &target,
                cartesian_midpoint: &cartesian_midpoint,
                filtered_points: filtered.points(),
                range_jump_m: confidence.range_jump_m,
                derivative_score: confidence.derivative_score,
                confidence: confidence.confidence,
            });
        }

        let lateral_error_m = target.y;
        let lateral_rate_m_s = self.tracker.lateral_rate_m_s(lateral_error_m, timestamp_us);
        let estimate = CorridorEstimate {
            lateral_error_m,
            lateral_rate_m_s,
            heading_error_rad: target.angle_rad,
            nearest_obstacle_m: nearest_front_obstacle_m(cloud),
            confidence: confidence.confidence,
        };

        if confidence.confidence >= APEX_REMEMBER_MIN_CONFIDENCE {
            self.tracker
                .remember_apex(breakpoint.angle_rad, discontinuity.score_m);
        } else {
            self.tracker.clear_apex_hysteresis();
        }
        self.tracker.remember_estimate(estimate.clone());

        estimate
    }
}

impl ApexCorridorPerception {
    fn opposite_params(&self) -> OppositeParams {
        OppositeParams {
            max_dist_error_m: self.params.max_opposite_dist_error_m,
            prefer_nearer: self.params.prefer_nearer_opposite,
            wall_clearance_m: self.params.wall_clearance_m,
        }
    }

    fn calculate_dynamic_lookahead(&self, cloud: &LidarCloud) -> f32 {
        let fov = self.params.side_lookahead_fov_deg.to_radians();
        let center = self.params.side_lookahead_center_deg.to_radians();

        // Use a conservative side distance when one side has no return.
        let dist = |angle| cloud.nearest_in_arc(angle, fov).map_or(0.5, |p| p.dist_m);
        let side_diff = (dist(center) - dist(-center)).abs();

        (self.params.max_lookahead_m - side_diff * self.params.lookahead_sensitivity)
            .clamp(self.params.min_lookahead_m, self.params.max_lookahead_m)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nfe_core::sensors::LidarPoint;

    #[derive(Default)]
    struct CaptureObserver {
        apex: Option<LidarPoint>,
        opposite: Option<LidarPoint>,
        target: Option<LidarPoint>,
    }

    impl PerceptionObserver for CaptureObserver {
        fn wants_apex(&self) -> bool {
            true
        }

        fn apex(&mut self, event: ApexObservation<'_>) {
            self.apex = Some(*event.apex);
            self.opposite = Some(*event.opposite);
            self.target = Some(*event.target);
        }
    }

    fn point(angle_rad: f32, dist_m: f32) -> LidarPoint {
        LidarPoint::from_polar(dist_m, angle_rad, 0)
    }

    fn cloud(points: Vec<LidarPoint>) -> LidarCloud {
        LidarCloud {
            points,
            timestamp_us: 0,
        }
    }

    #[test]
    fn discontinuity_threshold_is_not_scaled_by_lookahead() {
        let params = ApexParams {
            median_window: 1,
            min_points: 5,
            min_forward_m: 0.0,
            min_range_jump_m: 0.25,
            max_lookahead_m: 8.0,
            min_lookahead_m: 8.0,
            lookahead_sensitivity: 0.0,
            wall_clearance_m: 0.0,
            ..Default::default()
        };
        let mut perception = ApexCorridorPerception::new(params);
        let scan = cloud(vec![
            point(-PI / 2.0, 8.0),
            point(-1.0, 1.0),
            point(-0.999, 1.2),
            point(-0.5, 1.2),
            point(0.0, 1.55),
            point(0.5, 1.55),
            point(PI / 2.0, 8.0),
        ]);
        let mut observer = CaptureObserver::default();

        let estimate = perception.estimate_observed(&scan, 10, &mut observer);

        assert!(estimate.confidence > 0.0);
        let apex = observer.apex.expect("apex observation");
        assert!((apex.angle_rad + 0.5).abs() < 1e-6, "apex={apex:?}");
    }
}
