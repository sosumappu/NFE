//! Apex-based reactive corridor perception.
//!
//! This path looks for the strongest range discontinuity in the scan. The
//! closest endpoint of that discontinuity is treated as the visible apex; the
//! opposite wall is selected from the other angular side of the LiDAR scan and
//! sorted by `angle_rad`. A cubic Hermite curve over that wall is used to find
//! the perpendicular foot corresponding to the apex. The reactive target is the
//! polar midpoint of the apex/opposite gap.

use nfe_core::control::CorridorEstimate;
use nfe_core::params::Tunable;
use nfe_core::sensors::{HermiteBounds, LidarCloud, LidarPoint};
use nfe_core::wrap_angle;
use std::f32::consts::PI;

use super::{ApexObservation, NoopPerceptionObserver, PerceptionObserver};

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

    #[param(0.0..1.0, default = 0.15)]
    pub wall_clearance_m: f32,

    #[param(0.0..3.1415927, default = 0.35)]
    pub apex_switch_threshold_rad: f32,

    #[param(1.0..10.0, default = 1.8)]
    pub apex_switch_hysteresis_factor: f32,

    #[param(1.0..20.0, default = 7.0)]
    pub max_lookahead_m: f32,

    #[param(0.1..5.0, default = 0.5)]
    pub min_lookahead_m: f32,

    #[param(0.0..15.0, default = 5.0)]
    pub lookahead_sensitivity: f32,

    #[param(1.0..360.0, default = 30.0)]
    pub side_lookahead_fov_deg: f32,

    #[param(1.0..360.0, default = 80.0)]
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
    prev_lateral_error_m: Option<(f32, u64)>,
    prev_apex_angle_rad: Option<f32>,
    prev_apex_score: Option<f32>,
    prev_corridor_estimate: Option<CorridorEstimate>,
}

impl ApexCorridorPerception {
    pub fn new(params: ApexParams) -> Self {
        Self {
            params,
            prev_lateral_error_m: None,
            prev_apex_angle_rad: None,
            prev_apex_score: None,
            prev_corridor_estimate: None,
        }
    }

    pub fn reset(&mut self) {
        self.prev_lateral_error_m = None;
        self.clear_apex_hysteresis();
        self.prev_corridor_estimate = None;
    }
}

impl ApexPerception for ApexCorridorPerception {
    fn estimate_observed<O: PerceptionObserver + ?Sized>(
        &mut self,
        cloud: &LidarCloud,
        timestamp_us: u64,
        observer: &mut O,
    ) -> CorridorEstimate {
        let safe_lookahead_m = self.calculate_dynamic_lookahead(cloud, &self.params);

        let cropped = cloud
            .crop_to_front_arc(4.0 * std::f32::consts::PI / 5.0)
            .crop_distance(self.params.min_forward_m, safe_lookahead_m);

        let filtered = cropped.median_filtered(self.params.median_window);

        if filtered.points.len() < self.params.min_points {
            self.clear_apex_hysteresis();
            return CorridorEstimate {
                lateral_error_m: self.prev_lateral_error_m.map(|(err, _)| err).unwrap_or(0.0),
                lateral_rate_m_s: 0.0,
                heading_error_rad: 0.0,
                nearest_obstacle_m: cloud
                    .points
                    .iter()
                    .map(|p| p.dist_m)
                    .fold(f32::MAX, f32::min),
                confidence: 0.0,
            };
        }

        let (breakpoint_ref, gap_idx, is_in_wall_1) =
            filtered.find_discontinuity(self.params.min_range_jump_m);
        let candidate_score = discontinuity_score(&filtered, gap_idx);
        let (range_jump_m, derivative_score, confidence) = self.confidence_stats(&filtered);

        if confidence < 0.3 {
            self.clear_apex_hysteresis();
        }

        if confidence >= 0.3
            && self.should_hold_previous_apex(breakpoint_ref.angle_rad, candidate_score)
        {
            let confidence = self
                .prev_corridor_estimate
                .as_ref()
                .map_or(confidence, |estimate| estimate.confidence);
            return self.previous_estimate_with_confidence(cloud, confidence);
        }

        let breakpoint = *breakpoint_ref;

        let (wall_1, wall_2) = filtered.split_walls(gap_idx);

        let opposite_wall = if is_in_wall_1 { &wall_2 } else { &wall_1 };
        let bounds_opt = opposite_wall.find_bounding_points(&breakpoint);

        let raw_opposite_point = if let Some(bounds) = bounds_opt {
            opposite_from_bounds(bounds, &breakpoint, timestamp_us, &self.params)
        } else if let Some(fallback) = scan_based_opposite_fallback(opposite_wall, timestamp_us) {
            fallback
        } else {
            self.clear_apex_hysteresis();
            return self.previous_estimate_with_confidence(cloud, 0.0);
        };
        let opposite_point = pull_toward_origin(raw_opposite_point, self.params.wall_clearance_m);

        let target = polar_midpoint(breakpoint, opposite_point, timestamp_us);
        if observer.wants_apex() {
            let cartesian_midpoint = cartesian_midpoint(breakpoint, opposite_point, timestamp_us);
            observer.apex(ApexObservation {
                timestamp_us,
                apex: &breakpoint,
                opposite: &opposite_point,
                target: &target,
                cartesian_midpoint: &cartesian_midpoint,
                filtered_points: &filtered.points,
                range_jump_m,
                derivative_score,
                confidence,
            });
        }

        let lateral_error_m = target.y;
        let dt_s = if let Some((_, prev_ts)) = self.prev_lateral_error_m {
            (timestamp_us.saturating_sub(prev_ts)) as f32 / 1_000_000.0
        } else {
            0.1
        };

        let lateral_rate_m_s = if let Some((prev_err, _)) = self.prev_lateral_error_m {
            if dt_s > 0.0 {
                (lateral_error_m - prev_err) / dt_s
            } else {
                0.0
            }
        } else {
            0.0
        };

        self.prev_lateral_error_m = Some((lateral_error_m, timestamp_us));
        let estimate = CorridorEstimate {
            lateral_error_m,
            lateral_rate_m_s,
            heading_error_rad: target.angle_rad,
            nearest_obstacle_m: nearest_front_obstacle_m(cloud),
            confidence,
        };

        if confidence >= 0.3 {
            self.prev_apex_angle_rad = Some(breakpoint.angle_rad);
            self.prev_apex_score = Some(candidate_score);
        } else {
            self.clear_apex_hysteresis();
        }
        self.prev_corridor_estimate = Some(estimate.clone());

        estimate
    }
}

fn polar_midpoint(apex: LidarPoint, opposite: LidarPoint, timestamp_us: u64) -> LidarPoint {
    let target_dist = (apex.dist_m + opposite.dist_m) / 2.0;
    let angle_diff = opposite.angle_diff(&apex);
    let target_angle = wrap_angle(apex.angle_rad + angle_diff / 2.0);

    LidarPoint {
        x: target_dist * target_angle.cos(),
        y: target_dist * target_angle.sin(),
        dist_m: target_dist,
        angle_rad: target_angle,
        timestamp_us,
    }
}

fn opposite_from_bounds(
    bounds: HermiteBounds<'_>,
    breakpoint: &LidarPoint,
    timestamp_us: u64,
    params: &ApexParams,
) -> LidarPoint {
    if (bounds.a.dist_m - bounds.b.dist_m).abs() >= params.max_opposite_dist_error_m {
        *select_opposite_bound(bounds, breakpoint, params.prefer_nearer_opposite)
    } else {
        let angle =
            bounds
                .a
                .hermit_interpolation(bounds.b, bounds.prev, bounds.next, breakpoint.dist_m);
        LidarPoint {
            x: breakpoint.dist_m * angle.cos(),
            y: breakpoint.dist_m * angle.sin(),
            dist_m: breakpoint.dist_m,
            angle_rad: angle,
            timestamp_us,
        }
    }
}

fn select_opposite_bound<'a>(
    bounds: HermiteBounds<'a>,
    breakpoint: &LidarPoint,
    prefer_nearer_opposite: bool,
) -> &'a LidarPoint {
    if prefer_nearer_opposite {
        let a_err = (bounds.a.dist_m - breakpoint.dist_m).abs();
        let b_err = (bounds.b.dist_m - breakpoint.dist_m).abs();
        if a_err <= b_err {
            bounds.a
        } else {
            bounds.b
        }
    } else if bounds.a.dist_m > bounds.b.dist_m {
        bounds.a
    } else {
        bounds.b
    }
}

fn scan_based_opposite_fallback(
    opposite_wall: &LidarCloud,
    timestamp_us: u64,
) -> Option<LidarPoint> {
    if opposite_wall.points.is_empty() {
        return None;
    }

    let dist_m = median_range_m(&opposite_wall.points);
    let angle_rad = angular_centroid_rad(&opposite_wall.points);
    Some(LidarPoint {
        x: dist_m * angle_rad.cos(),
        y: dist_m * angle_rad.sin(),
        dist_m,
        angle_rad,
        timestamp_us,
    })
}

fn median_range_m(points: &[LidarPoint]) -> f32 {
    let mut ranges: Vec<_> = points.iter().map(|p| p.dist_m).collect();
    ranges.sort_by(f32::total_cmp);
    let mid = ranges.len() / 2;
    if ranges.len() % 2 == 0 {
        (ranges[mid - 1] + ranges[mid]) / 2.0
    } else {
        ranges[mid]
    }
}

fn angular_centroid_rad(points: &[LidarPoint]) -> f32 {
    let (sin_sum, cos_sum) = points.iter().fold((0.0_f32, 0.0_f32), |(sin, cos), p| {
        (sin + p.angle_rad.sin(), cos + p.angle_rad.cos())
    });
    if sin_sum.hypot(cos_sum) <= f32::EPSILON {
        wrap_angle(points.iter().map(|p| p.angle_rad).sum::<f32>() / points.len() as f32)
    } else {
        sin_sum.atan2(cos_sum)
    }
}

fn pull_toward_origin(point: LidarPoint, margin_m: f32) -> LidarPoint {
    let dist_m = (point.dist_m - margin_m.max(0.0)).max(0.0);
    LidarPoint {
        x: dist_m * point.angle_rad.cos(),
        y: dist_m * point.angle_rad.sin(),
        dist_m,
        ..point
    }
}

fn discontinuity_score(cloud: &LidarCloud, gap_idx: usize) -> f32 {
    cloud
        .points
        .get(gap_idx)
        .zip(cloud.points.get(gap_idx + 1))
        .map_or(0.0, |(a, b)| (b.dist_m - a.dist_m).abs())
}

fn angle_distance_rad(lhs: f32, rhs: f32) -> f32 {
    wrap_angle(lhs - rhs).abs()
}

fn nearest_front_obstacle_m(cloud: &LidarCloud) -> f32 {
    cloud
        .nearest_in_arc(0.0, PI / 12.0)
        .map(|p| p.dist_m)
        .unwrap_or(0.0)
}

fn cartesian_midpoint(apex: LidarPoint, opposite: LidarPoint, timestamp_us: u64) -> LidarPoint {
    let cx = (apex.x + opposite.x) / 2.0;
    let cy = (apex.y + opposite.y) / 2.0;
    LidarPoint {
        x: cx,
        y: cy,
        dist_m: cx.hypot(cy),
        angle_rad: cy.atan2(cx),
        timestamp_us,
    }
}

impl ApexCorridorPerception {
    fn clear_apex_hysteresis(&mut self) {
        self.prev_apex_angle_rad = None;
        self.prev_apex_score = None;
    }

    fn should_hold_previous_apex(&self, candidate_angle_rad: f32, candidate_score: f32) -> bool {
        if let (Some(prev_angle_rad), Some(prev_score)) =
            (self.prev_apex_angle_rad, self.prev_apex_score)
        {
            let angle_accepts = angle_distance_rad(candidate_angle_rad, prev_angle_rad)
                > self.params.apex_switch_threshold_rad;
            let score_accepts =
                candidate_score > prev_score * self.params.apex_switch_hysteresis_factor;
            !(angle_accepts || score_accepts)
        } else {
            false
        }
    }

    fn previous_estimate_with_confidence(
        &self,
        cloud: &LidarCloud,
        confidence: f32,
    ) -> CorridorEstimate {
        let mut estimate =
            self.prev_corridor_estimate
                .clone()
                .unwrap_or_else(|| CorridorEstimate {
                    lateral_error_m: self.prev_lateral_error_m.map(|(err, _)| err).unwrap_or(0.0),
                    lateral_rate_m_s: 0.0,
                    heading_error_rad: 0.0,
                    nearest_obstacle_m: nearest_front_obstacle_m(cloud),
                    confidence,
                });
        estimate.nearest_obstacle_m = nearest_front_obstacle_m(cloud);
        estimate.confidence = confidence;
        estimate
    }

    fn confidence_stats(&self, filtered_cloud: &LidarCloud) -> (f32, f32, f32) {
        let mut max_deriv = 0.0;
        let mut range_jump_m = 0.0;
        for w in filtered_cloud.points.windows(2) {
            let deriv = w[1].derivative(&w[0]).abs();
            if deriv > max_deriv {
                max_deriv = deriv;
                range_jump_m = (w[1].dist_m - w[0].dist_m).abs();
            }
        }

        let confidence = (range_jump_m / self.params.min_range_jump_m).clamp(0.0, 1.0);
        (range_jump_m, max_deriv, confidence)
    }

    fn calculate_dynamic_lookahead(&self, cloud: &LidarCloud, params: &ApexParams) -> f32 {
        let fov = params.side_lookahead_fov_deg.to_radians();
        let center = params.side_lookahead_center_deg.to_radians();

        // Lookahead fallback to 0.5 side distance if empty cloud
        let dist = |angle| cloud.nearest_in_arc(angle, fov).map_or(0.5, |p| p.dist_m);

        let side_diff = (dist(center) - dist(-center)).abs();

        // max - side_diff * sensitivity clamped to min and max
        (params.max_lookahead_m - side_diff * params.lookahead_sensitivity)
            .clamp(params.min_lookahead_m, params.max_lookahead_m)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
        LidarPoint {
            x: dist_m * angle_rad.cos(),
            y: dist_m * angle_rad.sin(),
            dist_m,
            angle_rad,
            timestamp_us: 0,
        }
    }

    fn cloud(points: Vec<LidarPoint>) -> LidarCloud {
        LidarCloud {
            points,
            timestamp_us: 0,
        }
    }

    #[test]
    fn polar_midpoint_between_sixty_degree_bounds_points_forward() {
        let apex = point(PI / 3.0, 2.0);
        let opposite = point(-PI / 3.0, 2.0);

        let target = polar_midpoint(apex, opposite, 7);

        assert!(target.angle_rad.abs() < 1e-6, "target={target:?}");
        assert!(target.y.abs() < 1e-6, "target={target:?}");
        assert!((target.x - 2.0).abs() < 1e-6, "target={target:?}");
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

    #[test]
    fn opposite_bound_selection_can_prefer_breakpoint_distance() {
        let prev = point(-0.4, 2.0);
        let a = point(-0.2, 4.0);
        let b = point(0.2, 1.2);
        let next = point(0.4, 1.0);
        let breakpoint = point(0.0, 1.0);
        let bounds = HermiteBounds {
            prev: &prev,
            a: &a,
            b: &b,
            next: &next,
        };

        let selected = select_opposite_bound(bounds, &breakpoint, true);
        assert_eq!(selected.angle_rad, b.angle_rad);

        let selected = select_opposite_bound(bounds, &breakpoint, false);
        assert_eq!(selected.angle_rad, a.angle_rad);
    }

    #[test]
    fn wall_clearance_pulls_opposite_toward_origin() {
        let original = point(0.25, 1.0);

        let pulled = pull_toward_origin(original, 0.15);

        assert!((pulled.dist_m - 0.85).abs() < 1e-6, "pulled={pulled:?}");
        assert!((pulled.angle_rad - original.angle_rad).abs() < 1e-6);
        assert!((pulled.x - 0.85 * original.angle_rad.cos()).abs() < 1e-6);
        assert!((pulled.y - 0.85 * original.angle_rad.sin()).abs() < 1e-6);
    }

    #[test]
    fn scan_fallback_uses_wall_median_range_and_angular_centroid() {
        let wall = cloud(vec![point(0.0, 1.0), point(0.2, 5.0), point(0.4, 3.0)]);

        let fallback = scan_based_opposite_fallback(&wall, 42).expect("fallback point");

        assert!(
            (fallback.dist_m - 3.0).abs() < 1e-6,
            "fallback={fallback:?}"
        );
        assert!(
            (fallback.angle_rad - 0.2).abs() < 1e-6,
            "fallback={fallback:?}"
        );
        assert_eq!(fallback.timestamp_us, 42);
    }

    #[test]
    fn hysteresis_holds_previous_apex_until_angle_or_score_accepts() {
        let mut perception = ApexCorridorPerception::new(ApexParams {
            apex_switch_threshold_rad: 0.35,
            apex_switch_hysteresis_factor: 1.8,
            ..Default::default()
        });
        perception.prev_apex_angle_rad = Some(0.0);
        perception.prev_apex_score = Some(1.0);

        assert!(perception.should_hold_previous_apex(0.1, 1.2));
        assert!(!perception.should_hold_previous_apex(0.1, 2.0));
        assert!(!perception.should_hold_previous_apex(0.5, 1.0));
    }
}
