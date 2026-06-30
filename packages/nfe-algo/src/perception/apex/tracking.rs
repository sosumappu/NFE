use nfe_core::control::CorridorEstimate;
use nfe_core::sensors::LidarCloud;
use nfe_core::wrap_angle;

use super::scan::{nearest_front_obstacle_m, nearest_obstacle_m};

#[derive(Clone, Debug, Default)]
pub(super) struct ApexTracker {
    prev_lateral_error_m: Option<(f32, u64)>,
    prev_apex_angle_rad: Option<f32>,
    prev_apex_score: Option<f32>,
    prev_corridor_estimate: Option<CorridorEstimate>,
    last_curvature_m_inv: f32,
}

impl ApexTracker {
    pub(super) fn reset(&mut self) {
        self.prev_lateral_error_m = None;
        self.clear_apex_hysteresis();
        self.prev_corridor_estimate = None;
        self.last_curvature_m_inv = 0.0;
    }

    pub(super) fn clear_apex_hysteresis(&mut self) {
        self.prev_apex_angle_rad = None;
        self.prev_apex_score = None;
    }

    pub(super) fn should_hold_previous_apex(
        &self,
        candidate_angle_rad: f32,
        candidate_score: f32,
        switch_threshold_rad: f32,
        hysteresis_factor: f32,
    ) -> bool {
        if let (Some(prev_angle_rad), Some(prev_score)) =
            (self.prev_apex_angle_rad, self.prev_apex_score)
        {
            let angle_accepts =
                angle_distance_rad(candidate_angle_rad, prev_angle_rad) > switch_threshold_rad;
            let score_accepts = candidate_score > prev_score * hysteresis_factor;
            !(angle_accepts || score_accepts)
        } else {
            false
        }
    }

    pub(super) fn insufficient_points_estimate(&self, cloud: &LidarCloud) -> CorridorEstimate {
        let lateral_error_m = self.prev_lateral_error_m.map(|(err, _)| err).unwrap_or(0.0);
        CorridorEstimate {
            lateral_error_m,
            lateral_rate_m_s: 0.0,
            heading_error_rad: 0.0,
            target_x_m: 0.0,
            target_y_m: lateral_error_m,
            curvature_m_inv: 0.0,
            nearest_obstacle_m: nearest_obstacle_m(cloud),
            confidence: 0.0,
        }
    }

    pub(super) fn hold_confidence(&self, candidate_confidence: f32) -> f32 {
        self.prev_corridor_estimate
            .as_ref()
            .map_or(candidate_confidence, |estimate| estimate.confidence)
    }

    pub(super) fn previous_estimate_with_confidence(
        &self,
        cloud: &LidarCloud,
        confidence: f32,
    ) -> CorridorEstimate {
        let mut estimate = self.prev_corridor_estimate.clone().unwrap_or_else(|| {
            let lateral_error_m = self.prev_lateral_error_m.map(|(err, _)| err).unwrap_or(0.0);
            CorridorEstimate {
                lateral_error_m,
                lateral_rate_m_s: 0.0,
                heading_error_rad: 0.0,
                target_x_m: 0.0,
                target_y_m: lateral_error_m,
                curvature_m_inv: self.last_curvature_m_inv,
                nearest_obstacle_m: nearest_front_obstacle_m(cloud),
                confidence,
            }
        });
        estimate.nearest_obstacle_m = nearest_front_obstacle_m(cloud);
        estimate.confidence = confidence;
        estimate
    }

    pub(super) fn held_estimate_with_confidence(
        &mut self,
        cloud: &LidarCloud,
        confidence: f32,
        timestamp_us: u64,
    ) -> CorridorEstimate {
        let mut estimate = self.previous_estimate_with_confidence(cloud, confidence);
        estimate.lateral_rate_m_s = self.lateral_rate_m_s(estimate.lateral_error_m, timestamp_us);
        self.remember_estimate(estimate.clone());
        estimate
    }

    pub(super) fn lateral_rate_m_s(&mut self, lateral_error_m: f32, timestamp_us: u64) -> f32 {
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
        lateral_rate_m_s
    }

    pub(super) fn remember_apex(&mut self, angle_rad: f32, score: f32) {
        self.prev_apex_angle_rad = Some(angle_rad);
        self.prev_apex_score = Some(score);
    }

    pub(super) fn tracked_apex_angle_rad(&self) -> Option<f32> {
        self.prev_apex_angle_rad
    }

    pub(super) fn remember_curvature_m_inv(&mut self, curvature_m_inv: f32) {
        if curvature_m_inv.is_finite() && curvature_m_inv.abs() > f32::EPSILON {
            self.last_curvature_m_inv = curvature_m_inv;
        }
    }

    pub(super) fn last_curvature_m_inv(&self) -> f32 {
        self.last_curvature_m_inv
    }

    pub(super) fn remember_estimate(&mut self, estimate: CorridorEstimate) {
        self.prev_corridor_estimate = Some(estimate);
    }
}

fn angle_distance_rad(lhs: f32, rhs: f32) -> f32 {
    wrap_angle(lhs - rhs).abs()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hysteresis_holds_previous_apex_until_angle_or_score_accepts() {
        let mut tracker = ApexTracker::default();
        tracker.remember_apex(0.0, 1.0);

        assert!(tracker.should_hold_previous_apex(0.1, 1.2, 0.35, 1.8));
        assert!(!tracker.should_hold_previous_apex(0.1, 2.0, 0.35, 1.8));
        assert!(!tracker.should_hold_previous_apex(0.5, 1.0, 0.35, 1.8));
    }

    #[test]
    fn held_estimate_recomputes_lateral_rate_for_current_timestamp() {
        let mut tracker = ApexTracker::default();
        tracker.lateral_rate_m_s(0.4, 1_000_000);
        tracker.remember_estimate(CorridorEstimate {
            lateral_error_m: 0.4,
            lateral_rate_m_s: 42.0,
            confidence: 0.8,
            ..Default::default()
        });

        let held = tracker.held_estimate_with_confidence(&LidarCloud::default(), 0.7, 1_100_000);

        assert_eq!(held.lateral_rate_m_s, 0.0);
        assert_eq!(held.confidence, 0.7);
    }
}
