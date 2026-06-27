use anyhow::{anyhow, Result};
use nfe_core::wrap_angle;

#[derive(Clone, Debug)]
pub struct TrackProgress {
    segments: Vec<ProgressSegment>,
    length_m: f32,
    last_s_m: Option<f32>,
    unwrapped_s_m: f32,
}

#[derive(Clone, Copy, Debug)]
struct ProgressSegment {
    ax: f32,
    ay: f32,
    dx: f32,
    dy: f32,
    len_m: f32,
    start_s_m: f32,
    heading_rad: f32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ProgressSample {
    pub s_m: f32,
    pub unwrapped_s_m: f32,
    pub lap: u32,
    pub lateral_error_m: f32,
    pub heading_error_rad: f32,
    pub segment_index: usize,
}

impl TrackProgress {
    pub fn from_waypoints(waypoints: &[(f32, f32)]) -> Result<Self> {
        if waypoints.len() < 2 {
            return Err(anyhow!(
                "track progress requires at least two world waypoints"
            ));
        }

        let mut segments = Vec::with_capacity(waypoints.len());
        let mut start_s_m = 0.0;
        for i in 0..waypoints.len() {
            let (ax, ay) = waypoints[i];
            let (bx, by) = waypoints[(i + 1) % waypoints.len()];
            let dx = bx - ax;
            let dy = by - ay;
            let len_m = dx.hypot(dy);
            if len_m <= f32::EPSILON {
                continue;
            }
            segments.push(ProgressSegment {
                ax,
                ay,
                dx,
                dy,
                len_m,
                start_s_m,
                heading_rad: dy.atan2(dx),
            });
            start_s_m += len_m;
        }

        if segments.is_empty() {
            return Err(anyhow!("track progress requires non-degenerate waypoints"));
        }

        Ok(Self {
            segments,
            length_m: start_s_m,
            last_s_m: None,
            unwrapped_s_m: 0.0,
        })
    }

    pub fn length_m(&self) -> f32 {
        self.length_m
    }

    pub fn update(&mut self, x: f32, y: f32, yaw_rad: f32) -> ProgressSample {
        let projection = self.project(x, y);
        if let Some(last_s_m) = self.last_s_m {
            let mut delta_s_m = projection.s_m - last_s_m;
            if delta_s_m < -0.5 * self.length_m {
                delta_s_m += self.length_m;
            } else if delta_s_m > 0.5 * self.length_m {
                delta_s_m -= self.length_m;
            }
            self.unwrapped_s_m += delta_s_m;
        }
        self.last_s_m = Some(projection.s_m);

        let lap = (self.unwrapped_s_m.max(0.0) / self.length_m).floor() as u32;
        ProgressSample {
            s_m: projection.s_m,
            unwrapped_s_m: self.unwrapped_s_m,
            lap,
            lateral_error_m: projection.lateral_error_m,
            heading_error_rad: wrap_angle(yaw_rad - projection.heading_rad),
            segment_index: projection.segment_index,
        }
    }

    fn project(&self, x: f32, y: f32) -> Projection {
        self.segments
            .iter()
            .enumerate()
            .map(|(idx, segment)| segment.project(idx, x, y))
            .min_by(|a, b| a.dist2.partial_cmp(&b.dist2).unwrap())
            .expect("TrackProgress always has at least one segment")
    }
}

#[derive(Clone, Copy, Debug)]
struct Projection {
    s_m: f32,
    lateral_error_m: f32,
    heading_rad: f32,
    segment_index: usize,
    dist2: f32,
}

impl ProgressSegment {
    fn project(&self, segment_index: usize, x: f32, y: f32) -> Projection {
        let px = x - self.ax;
        let py = y - self.ay;
        let len2 = self.len_m * self.len_m;
        let u = ((px * self.dx + py * self.dy) / len2).clamp(0.0, 1.0);
        let qx = self.ax + u * self.dx;
        let qy = self.ay + u * self.dy;
        let ex = x - qx;
        let ey = y - qy;
        let dist = ex.hypot(ey);
        let cross = self.dx * ey - self.dy * ex;
        let lateral_error_m = if cross >= 0.0 { dist } else { -dist };

        Projection {
            s_m: self.start_s_m + u * self.len_m,
            lateral_error_m,
            heading_rad: self.heading_rad,
            segment_index,
            dist2: ex * ex + ey * ey,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn projects_onto_straight_segment_with_signed_lateral_error() {
        let mut progress = TrackProgress::from_waypoints(&[(0.0, 0.0), (10.0, 0.0)]).unwrap();
        let left = progress.update(2.0, 1.0, 0.0);
        assert!((left.s_m - 2.0).abs() < 1e-6);
        assert!((left.lateral_error_m - 1.0).abs() < 1e-6);

        let right = progress.update(3.0, -0.5, 0.0);
        assert!((right.lateral_error_m + 0.5).abs() < 1e-6);
    }

    #[test]
    fn unwraps_forward_progress_across_start_line() {
        let mut progress =
            TrackProgress::from_waypoints(&[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)])
                .unwrap();

        for p in [
            (0.1, 0.0, 0.0),
            (0.9, 0.0, 0.0),
            (1.0, 0.9, std::f32::consts::FRAC_PI_2),
            (0.1, 1.0, std::f32::consts::PI),
            (0.0, 0.1, -std::f32::consts::FRAC_PI_2),
            (0.2, 0.0, 0.0),
        ] {
            progress.update(p.0, p.1, p.2);
        }

        let sample = progress.update(0.9, 0.0, 0.0);
        assert_eq!(sample.lap, 1);
        assert!(sample.unwrapped_s_m > progress.length_m());
    }

    #[test]
    fn reverse_start_crossing_does_not_create_lap() {
        let mut progress =
            TrackProgress::from_waypoints(&[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)])
                .unwrap();

        progress.update(0.1, 0.0, 0.0);
        let sample = progress.update(0.0, 0.1, -std::f32::consts::FRAC_PI_2);
        assert_eq!(sample.lap, 0);
        assert!(sample.unwrapped_s_m < 0.0);
    }
}
