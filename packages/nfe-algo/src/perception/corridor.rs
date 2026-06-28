//! RANSAC-backed reactive corridor perception.

use nfe_core::control::CorridorEstimate;
use nfe_core::params::Tunable;
use nfe_core::{wrap_angle, Point2, WallLine};

use super::ransac::{fit_walls, RansacParams};
use super::{NoopPerceptionObserver, PerceptionObserver, RansacWallsObservation};

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize, Tunable)]
#[serde(default)]
pub struct CorridorParams {
    #[tunable(nested)]
    pub ransac: RansacParams,
    #[param(0.05..2.0, default = 0.12)]
    pub min_wall_length_m: f32,
}

impl Default for CorridorParams {
    fn default() -> Self {
        Self {
            ransac: RansacParams::default(),
            min_wall_length_m: 0.12,
        }
    }
}

pub trait CorridorPerception {
    fn estimate(&mut self, points: &[Point2], timestamp_us: u64) -> CorridorEstimate {
        let mut observer = NoopPerceptionObserver;
        self.estimate_observed(points, timestamp_us, &mut observer)
    }

    fn estimate_observed<O: PerceptionObserver + ?Sized>(
        &mut self,
        points: &[Point2],
        timestamp_us: u64,
        observer: &mut O,
    ) -> CorridorEstimate;
}

#[derive(Clone, Debug)]
pub struct RansacCorridorPerception {
    params: CorridorParams,
    seed: u64,
    prev_lateral_error_m: Option<(f32, u64)>,
}

impl RansacCorridorPerception {
    pub fn new(params: CorridorParams, seed: u64) -> Self {
        Self {
            params,
            seed,
            prev_lateral_error_m: None,
        }
    }
}

impl CorridorPerception for RansacCorridorPerception {
    fn estimate_observed<O: PerceptionObserver + ?Sized>(
        &mut self,
        points: &[Point2],
        timestamp_us: u64,
        observer: &mut O,
    ) -> CorridorEstimate {
        let mut walls = fit_walls(points, &self.params.ransac, self.seed ^ timestamp_us);
        walls.retain(|w| w.p0.dist(&w.p1) >= self.params.min_wall_length_m);

        let nearest = points
            .iter()
            .map(|p| p.x.hypot(p.y))
            .fold(f32::INFINITY, f32::min);

        let (lateral_error_m, heading_error_rad, confidence) = estimate_from_walls(&walls);
        if observer.wants_ransac_walls() {
            observer.ransac_walls(RansacWallsObservation {
                timestamp_us,
                points,
                walls: &walls,
                confidence,
            });
        }

        let lateral_rate_m_s = match self
            .prev_lateral_error_m
            .replace((lateral_error_m, timestamp_us))
        {
            Some((prev, prev_ts)) => {
                let dt = timestamp_us.saturating_sub(prev_ts) as f32 * 1e-6;
                if dt > 0.0 {
                    (lateral_error_m - prev) / dt
                } else {
                    0.0
                }
            }
            None => 0.0,
        };

        CorridorEstimate {
            lateral_error_m,
            lateral_rate_m_s,
            heading_error_rad,
            nearest_obstacle_m: nearest,
            confidence,
        }
    }
}

fn estimate_from_walls(walls: &[WallLine]) -> (f32, f32, f32) {
    if walls.is_empty() {
        return (0.0, 0.0, 0.0);
    }

    let mut left: Option<&WallLine> = None;
    let mut right: Option<&WallLine> = None;
    for w in walls {
        let mid_y = 0.5 * (w.p0.y + w.p1.y);
        if mid_y >= 0.0 {
            if left.is_none_or(|old| w.support > old.support) {
                left = Some(w);
            }
        } else if right.is_none_or(|old| w.support > old.support) {
            right = Some(w);
        }
    }

    let selected: Vec<&WallLine> = [left, right].into_iter().flatten().collect();
    if selected.is_empty() {
        return (0.0, 0.0, 0.0);
    }

    let mut center_y = 0.0;
    let mut heading = 0.0;
    let mut support = 0.0;
    for w in &selected {
        center_y += 0.5 * (w.p0.y + w.p1.y);
        // Direction vector perpendicular to normal. Flip to point generally forward.
        let mut dx = w.ny;
        let mut dy = -w.nx;
        if dx < 0.0 {
            dx = -dx;
            dy = -dy;
        }
        heading += dy.atan2(dx);
        support += w.support;
    }

    center_y /= selected.len() as f32;
    heading /= selected.len() as f32;
    support /= selected.len() as f32;

    // If both walls are present, the corridor center is the midpoint of their
    // wall midpoints. Positive error means the desired centerline is to the
    // left of the car, matching positive steering in the vehicle model.
    (center_y, wrap_angle(heading), support.clamp(0.0, 1.0))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn symmetric_corridor_estimates_near_zero() {
        let mut pts = Vec::new();
        for i in 0..40 {
            let x = i as f32 * 0.05;
            pts.push(Point2::new(x, 0.5));
            pts.push(Point2::new(x, -0.5));
        }
        let mut p = RansacCorridorPerception::new(CorridorParams::default(), 1);
        let e = p.estimate(&pts, 10_000);
        assert!(e.confidence > 0.2, "confidence={}", e.confidence);
        assert!(e.lateral_error_m.abs() < 0.05, "lat={}", e.lateral_error_m);
        assert!(
            e.heading_error_rad.abs() < 0.1,
            "head={}",
            e.heading_error_rad
        );
    }

    #[test]
    fn corridor_left_of_car_yields_positive_error() {
        let mut pts = Vec::new();
        for i in 0..40 {
            let x = i as f32 * 0.05;
            pts.push(Point2::new(x, 1.0));
            pts.push(Point2::new(x, -0.5));
        }
        let mut p = RansacCorridorPerception::new(CorridorParams::default(), 2);
        let e = p.estimate(&pts, 10_000);
        assert!(e.confidence > 0.2, "confidence={}", e.confidence);
        assert!(e.lateral_error_m > 0.1, "lat={}", e.lateral_error_m);
    }

    #[test]
    fn corridor_right_of_car_yields_negative_error() {
        let mut pts = Vec::new();
        for i in 0..40 {
            let x = i as f32 * 0.05;
            pts.push(Point2::new(x, 0.5));
            pts.push(Point2::new(x, -1.0));
        }
        let mut p = RansacCorridorPerception::new(CorridorParams::default(), 3);
        let e = p.estimate(&pts, 10_000);
        assert!(e.confidence > 0.2, "confidence={}", e.confidence);
        assert!(e.lateral_error_m < -0.1, "lat={}", e.lateral_error_m);
    }
}
