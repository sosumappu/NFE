//! Sensor-frame types used by pure runtime pipeline tests.

use crate::Point2;
use std::f32::consts::{PI, TAU};

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct LidarPoint {
    pub x: f32,
    pub y: f32,
    pub dist_m: f32,
    pub angle_rad: f32,
    pub timestamp_us: u64,
}

impl LidarPoint {
    pub fn point2(&self) -> Point2 {
        Point2::new(self.x, self.y)
    }

    pub fn angle_diff(&self, rhs: &Self) -> f32 {
        (self.angle_rad - rhs.angle_rad + PI).rem_euclid(TAU) - PI
    }
    pub fn derivative(&self, rhs: &Self) -> f32 {
        let d_theta = self.angle_diff(rhs);
        if d_theta.abs() < f32::EPSILON {
            0.0
        } else {
            (self.dist_m - rhs.dist_m) / d_theta
        }
    }

    pub fn opposite(&self) -> f32 {
        (self.angle_rad + PI).rem_euclid(TAU) - PI
    }

    pub fn hermit_interpolation(&self, rhs: &Self, prev: &Self, next: &Self, rho: f32) -> f32 {
        let d_rho = rhs.dist_m - self.dist_m;
        if d_rho.abs() < f32::EPSILON {
            return self.angle_rad;
        }

        let prev_rho = self.dist_m - prev.dist_m;
        let next_rho = next.dist_m - rhs.dist_m;
        let dot_theta_a = if prev_rho.abs() < f32::EPSILON {
            0.0
        } else {
            self.angle_diff(prev) / prev_rho
        };
        let dot_theta_b = if next_rho.abs() < f32::EPSILON {
            0.0
        } else {
            next.angle_diff(rhs) / next_rho
        };

        let t = ((rho - self.dist_m) / d_rho).clamp(0.0, 1.0);

        // Calculate power for performance
        let t2 = t * t;
        let t3 = t2 * t;

        let h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
        let h10 = -2.0 * t3 + 3.0 * t2;
        let h01 = (t3 - 2.0 * t2 + t) * d_rho;
        let h11 = (t3 - t2) * d_rho;

        let angle = h00 * self.angle_rad
            + h10 * (self.angle_rad + rhs.angle_diff(self))
            + h01 * dot_theta_a
            + h11 * dot_theta_b;
        (angle + PI).rem_euclid(TAU) - PI
    }
}

#[derive(Clone, Copy, Debug)]
pub struct HermiteBounds<'a> {
    pub prev: &'a LidarPoint,
    pub a: &'a LidarPoint,
    pub b: &'a LidarPoint,
    pub next: &'a LidarPoint,
}

#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct LidarCloud {
    pub points: Vec<LidarPoint>,
    pub timestamp_us: u64,
}

impl LidarCloud {
    pub fn as_points2(&self) -> Vec<Point2> {
        self.points.iter().map(LidarPoint::point2).collect()
    }

    pub fn nearest_in_arc(&self, center_angle_rad: f32, fov_rad: f32) -> Option<&LidarPoint> {
        let half_fov = fov_rad / 2.0;
        let mut nearest_point = None;
        let mut min_dist = f32::MAX;

        for p in &self.points {
            let angle_diff = (p.angle_rad - center_angle_rad + std::f32::consts::PI)
                .rem_euclid(std::f32::consts::TAU)
                - std::f32::consts::PI;

            if angle_diff.abs() <= half_fov && p.dist_m < min_dist {
                min_dist = p.dist_m;
                nearest_point = Some(p);
            }
        }

        nearest_point
    }

    pub fn crop_to_front_arc(&self, fov_rad: f32) -> LidarCloud {
        let half_fov = fov_rad / 2.0;

        let cropped_points: Vec<LidarPoint> = self
            .points
            .iter()
            .copied()
            .filter(|p| p.angle_rad.abs() <= half_fov)
            .collect();

        LidarCloud {
            points: cropped_points,
            timestamp_us: self.timestamp_us,
        }
    }
    pub fn crop_distance(&self, min_dist_m: f32, max_dist_m: f32) -> LidarCloud {
        let cropped_points: Vec<LidarPoint> = self
            .points
            .iter()
            .copied()
            .filter(|p| (min_dist_m..=max_dist_m).contains(&p.dist_m))
            .collect();

        LidarCloud {
            points: cropped_points,
            timestamp_us: self.timestamp_us,
        }
    }

    pub fn median_filtered(&self, window: usize) -> LidarCloud {
        let n = self.points.len();

        let window = window.min(256).min(n);
        let half_width = window / 2;
        let mut buf = Vec::with_capacity(n);

        // Scratch space — reused each iteration, no per-point allocation
        let mut scratch = [0.0f32; 256];

        for i in 0..n {
            // Collect distances from the window, wrapping circularly
            for j in 0..window {
                let idx = (i + n + j - half_width) % n;
                scratch[j] = self.points[idx].dist_m;
            }

            // Partial sort — only need the median, not full sort
            let mid = window / 2;
            let median_dist = *scratch[..window]
                .select_nth_unstable_by(mid, |a, b| {
                    a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal)
                })
                .1;

            // Preserve original point geometry, replace distance
            let p = &self.points[i];
            let angle_rad = p.angle_rad;

            buf.push(LidarPoint {
                dist_m: median_dist,
                x: median_dist * angle_rad.cos(),
                y: median_dist * angle_rad.sin(),
                ..*p
            });
        }

        LidarCloud {
            points: buf,
            timestamp_us: self.timestamp_us,
        }
    }

    pub fn find_discontinuity(&self, min_range_jump_m: f32) -> (&LidarPoint, usize, bool) {
        let windows = self.points.windows(2).enumerate();

        // 1. Try to find the closest gap that meets the jump threshold
        let best_gap = windows
            .clone()
            .filter(|(_, w)| (w[1].dist_m - w[0].dist_m).abs() >= min_range_jump_m)
            .min_by(|(_, a), (_, b)| {
                a[0].dist_m
                    .min(a[1].dist_m)
                    .total_cmp(&b[0].dist_m.min(b[1].dist_m))
            })
            // 2. Fallback to the strongest derivative if no jump is big enough
            .unwrap_or_else(|| {
                windows
                    .max_by(|(_, a), (_, b)| {
                        a[1].derivative(&a[0])
                            .abs()
                            .total_cmp(&b[1].derivative(&b[0]).abs())
                    })
                    .expect("Cloud must have at least 2 points")
            });

        let (idx, w) = best_gap;
        let is_w0 = w[0].dist_m < w[1].dist_m;

        (if is_w0 { &w[0] } else { &w[1] }, idx, is_w0)
    }

    pub fn split_walls(&self, gap_idx: usize) -> (LidarCloud, LidarCloud) {
        let wall_1 = self.points[..=gap_idx].to_vec();
        let wall_2 = self.points[gap_idx + 1..].to_vec();

        (
            LidarCloud {
                points: wall_1,
                timestamp_us: self.timestamp_us,
            },
            LidarCloud {
                points: wall_2,
                timestamp_us: self.timestamp_us,
            },
        )
    }

    pub fn find_bounding_points(&self, breakpoint: &LidarPoint) -> Option<HermiteBounds<'_>> {
        if self.points.len() < 2 {
            return None;
        }

        let target = breakpoint.dist_m;

        for (i, w) in self.points.windows(2).enumerate() {
            let p1_dist = w[0].dist_m;
            let p2_dist = w[1].dist_m;

            if (p1_dist..=p2_dist).contains(&target) || (p2_dist..=p1_dist).contains(&target) {
                let prev = i.saturating_sub(1);
                let next = (i + 2).min(self.points.len() - 1);
                return Some(HermiteBounds {
                    prev: &self.points[prev],
                    a: &self.points[i],
                    b: &self.points[i + 1],
                    next: &self.points[next],
                });
            }
        }

        None
    }
}
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct SensorSnapshot {
    pub lidar: LidarCloud,
    pub imu: crate::estimation::ImuSample,
    pub sensor_fault: bool,
    /// Distances from [front, left, right] sonars; may be f32::MAX if unused.
    pub sonar_m: [f32; 3],
    /// Optional physical start-line crossing edge from timing gates.
    pub start_line_crossed: bool,
}
