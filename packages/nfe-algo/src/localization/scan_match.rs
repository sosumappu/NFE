use nfe_core::estimation::PoseMeasurement;
use nfe_core::localization::{LocalizationResult, Localizer};
use nfe_core::mapping::TrackMap;
use nfe_core::params::Tunable;
use nfe_core::sensors::LidarCloud;
use nfe_core::{wrap_angle, Point2, Pose2, WallLine};

use crate::perception::ransac::{fit_walls, RansacParams};

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize, Tunable)]
#[serde(default)]
pub struct ScanMatchParams {
    #[tunable(nested)]
    pub ransac: RansacParams,
    #[param(0.02..1.0, default = 0.20)]
    pub max_match_dist_m: f32,
    #[param(0.05..1.57, default = 0.35)]
    pub max_normal_angle_rad: f32,
    #[param(0.0..1.0, default = 0.6)]
    pub translation_gain: f32,
    #[param(0.0..1.0, default = 0.5)]
    pub yaw_gain: f32,
    #[param(int, 1..20, default = 3)]
    pub min_matches: usize,
}

impl Default for ScanMatchParams {
    fn default() -> Self {
        Self {
            ransac: RansacParams::default(),
            max_match_dist_m: 0.20,
            max_normal_angle_rad: 0.35,
            translation_gain: 0.6,
            yaw_gain: 0.5,
            min_matches: 3,
        }
    }
}

pub struct ScanMatchLocalizer {
    params: ScanMatchParams,
    seed: u64,
}

impl ScanMatchLocalizer {
    pub fn new(params: ScanMatchParams, seed: u64) -> Self {
        Self { params, seed }
    }
}

impl Localizer for ScanMatchLocalizer {
    fn localize(&mut self, cloud: &LidarCloud, prior: Pose2, map: &TrackMap) -> LocalizationResult {
        if map.boundaries.walls.is_empty() || cloud.points.len() < self.params.ransac.min_inliers {
            return LocalizationResult::default();
        }
        let points = cloud.as_points2();
        let local_walls = fit_walls(&points, &self.params.ransac, self.seed ^ cloud.timestamp_us);
        if local_walls.is_empty() {
            return LocalizationResult::default();
        }

        let mut n_match = 0usize;
        let mut residual_sum = 0.0f32;
        let mut dx = 0.0f32;
        let mut dy = 0.0f32;
        let mut dyaw = 0.0f32;

        for local in &local_walls {
            let world = transform_wall(prior, local);
            if let Some((mw, dist, yaw_err)) =
                nearest_wall(&world, &map.boundaries.walls, &self.params)
            {
                n_match += 1;
                residual_sum += dist.abs();
                // Correct translation along the matched map normal. If the
                // transformed scan wall has positive signed distance from the
                // map line, move pose against that normal.
                let signed = mw.signed_distance(&midpoint(&world));
                dx += -signed * mw.nx;
                dy += -signed * mw.ny;
                dyaw += yaw_err;
            }
        }

        if n_match < self.params.min_matches {
            return LocalizationResult::default();
        }

        let inv = 1.0 / n_match as f32;
        let residual = residual_sum * inv;
        let pose = Pose2::new(
            prior.x + self.params.translation_gain * dx * inv,
            prior.y + self.params.translation_gain * dy * inv,
            wrap_angle(prior.yaw + self.params.yaw_gain * dyaw * inv),
        );
        let confidence = (1.0 - residual / self.params.max_match_dist_m).clamp(0.0, 1.0);
        LocalizationResult {
            measurement: Some(PoseMeasurement {
                pose,
                quality: confidence,
            }),
            confidence,
            residual_m: residual,
            used_fallback: false,
        }
    }
}

fn midpoint(w: &WallLine) -> Point2 {
    Point2::new(0.5 * (w.p0.x + w.p1.x), 0.5 * (w.p0.y + w.p1.y))
}

fn transform_wall(pose: Pose2, w: &WallLine) -> WallLine {
    let p0 = pose.transform_point(&w.p0);
    let p1 = pose.transform_point(&w.p1);
    let dx = p1.x - p0.x;
    let dy = p1.y - p0.y;
    let len = dx.hypot(dy).max(1e-6);
    let nx = -dy / len;
    let ny = dx / len;
    let d = nx * p0.x + ny * p0.y;
    WallLine {
        nx,
        ny,
        d,
        p0,
        p1,
        support: w.support,
    }
}

fn wall_heading(w: &WallLine) -> f32 {
    let mut dx = w.ny;
    let mut dy = -w.nx;
    if dx < 0.0 {
        dx = -dx;
        dy = -dy;
    }
    dy.atan2(dx)
}

fn nearest_wall<'a>(
    w: &WallLine,
    map: &'a [WallLine],
    p: &ScanMatchParams,
) -> Option<(&'a WallLine, f32, f32)> {
    let m = midpoint(w);
    map.iter()
        .filter_map(|mw| {
            let normal_dot = (w.nx * mw.nx + w.ny * mw.ny).abs().clamp(0.0, 1.0);
            let normal_angle = normal_dot.acos();
            if normal_angle > p.max_normal_angle_rad {
                return None;
            }
            let dist = mw.signed_distance(&m).abs();
            if dist > p.max_match_dist_m {
                return None;
            }
            let yaw_err = wrap_angle(wall_heading(mw) - wall_heading(w));
            Some((mw, dist, yaw_err))
        })
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
}

#[cfg(test)]
mod tests {
    use super::*;
    use nfe_core::mapping::{BoundarySet, TrackMap};
    use nfe_core::sensors::LidarPoint;

    #[test]
    fn localizes_against_two_walls() {
        let map = TrackMap {
            boundaries: BoundarySet {
                walls: vec![
                    WallLine {
                        nx: 0.0,
                        ny: 1.0,
                        d: 0.5,
                        p0: Point2::new(0.0, 0.5),
                        p1: Point2::new(2.0, 0.5),
                        support: 1.0,
                    },
                    WallLine {
                        nx: 0.0,
                        ny: 1.0,
                        d: -0.5,
                        p0: Point2::new(0.0, -0.5),
                        p1: Point2::new(2.0, -0.5),
                        support: 1.0,
                    },
                ],
            },
            complete: true,
            revision: 1,
        };
        let mut cloud = LidarCloud {
            points: Vec::new(),
            timestamp_us: 1,
        };
        for i in 0..40 {
            let x = i as f32 * 0.05;
            cloud.points.push(LidarPoint {
                x,
                y: 0.5,
                dist_m: x.hypot(0.5),
                angle_rad: 0.0,
                timestamp_us: 1,
            });
            cloud.points.push(LidarPoint {
                x,
                y: -0.5,
                dist_m: x.hypot(0.5),
                angle_rad: 0.0,
                timestamp_us: 1,
            });
        }
        let mut loc = ScanMatchLocalizer::new(
            ScanMatchParams {
                min_matches: 2,
                ..Default::default()
            },
            1,
        );
        let out = loc.localize(&cloud, Pose2::default(), &map);
        assert!(out.measurement.is_some());
        assert!(out.confidence > 0.5, "confidence={}", out.confidence);
    }
}
