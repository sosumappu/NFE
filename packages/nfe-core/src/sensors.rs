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
    pub fn from_polar(dist_m: f32, angle_rad: f32, timestamp_us: u64) -> Self {
        Self {
            x: dist_m * angle_rad.cos(),
            y: dist_m * angle_rad.sin(),
            dist_m,
            angle_rad,
            timestamp_us,
        }
    }

    pub fn point2(&self) -> Point2 {
        Point2::new(self.x, self.y)
    }

    pub fn with_distance(self, dist_m: f32) -> Self {
        Self::from_polar(dist_m, self.angle_rad, self.timestamp_us)
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

            // Preserve the original angle/timestamp, replace distance.
            buf.push(self.points[i].with_distance(median_dist));
        }

        LidarCloud {
            points: buf,
            timestamp_us: self.timestamp_us,
        }
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
