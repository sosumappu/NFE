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
use nfe_core::sensors::{LidarCloud, LidarPoint};
use std::f32::consts::{PI, TAU};

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
}

impl ApexCorridorPerception {
    pub fn new(params: ApexParams) -> Self {
        Self {
            params,
            prev_lateral_error_m: None,
        }
    }

    pub fn reset(&mut self) {
        self.prev_lateral_error_m = None;
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
            filtered.find_discontinuity(self.params.min_range_jump_m * safe_lookahead_m);
        let breakpoint = *breakpoint_ref;

        let (wall_1, wall_2) = filtered.split_walls(gap_idx);

        let opposite_wall = if is_in_wall_1 { &wall_2 } else { &wall_1 };
        let bounds_opt = opposite_wall.find_bounding_points(&breakpoint);

        let opposite_point = if let Some(bounds) = bounds_opt {
            if (bounds.a.dist_m - bounds.b.dist_m).abs() >= self.params.max_opposite_dist_error_m {
                if bounds.a.dist_m > bounds.b.dist_m {
                    *bounds.a
                } else {
                    *bounds.b
                }
            } else {
                let angle = bounds.a.hermit_interpolation(
                    bounds.b,
                    bounds.prev,
                    bounds.next,
                    breakpoint.dist_m,
                );
                LidarPoint {
                    x: breakpoint.dist_m * angle.cos(),
                    y: breakpoint.dist_m * angle.sin(),
                    dist_m: breakpoint.dist_m,
                    angle_rad: angle,
                    timestamp_us,
                }
            }
        } else {
            let opp_angle = breakpoint.opposite();
            LidarPoint {
                x: breakpoint.dist_m * opp_angle.cos(),
                y: breakpoint.dist_m * opp_angle.sin(),
                dist_m: breakpoint.dist_m,
                angle_rad: opp_angle,
                timestamp_us,
            }
        };

        let target = polar_midpoint(breakpoint, opposite_point, timestamp_us);
        let (range_jump_m, derivative_score, confidence) = self.confidence_stats(&filtered);
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
        let nearest = cloud
            .nearest_in_arc(0.0, PI / 12.0)
            .map(|p| p.dist_m)
            .unwrap_or(0.0);
        CorridorEstimate {
            lateral_error_m,
            lateral_rate_m_s,
            heading_error_rad: target.angle_rad,
            nearest_obstacle_m: nearest,
            confidence,
        }
    }
}

fn polar_midpoint(apex: LidarPoint, opposite: LidarPoint, timestamp_us: u64) -> LidarPoint {
    let target_dist = (apex.dist_m + opposite.dist_m) / 2.0;
    let angle_diff = opposite.angle_diff(&apex);
    let target_angle = (apex.angle_rad + angle_diff / 2.0 + PI).rem_euclid(TAU) - PI;

    LidarPoint {
        x: target_dist * target_angle.cos(),
        y: target_dist * target_angle.sin(),
        dist_m: target_dist,
        angle_rad: target_angle,
        timestamp_us,
    }
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
