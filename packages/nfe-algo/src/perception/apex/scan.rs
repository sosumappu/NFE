use nfe_core::sensors::{LidarCloud, LidarPoint};
use std::f32::consts::PI;

const FRONT_FOV_RAD: f32 = 4.0 * PI / 5.0;

#[derive(Clone, Debug)]
pub(super) struct ApexScan {
    cloud: LidarCloud,
}

impl ApexScan {
    pub(super) fn preprocess(
        cloud: &LidarCloud,
        min_forward_m: f32,
        max_lookahead_m: f32,
        median_window: usize,
    ) -> Self {
        let cloud = cloud
            .crop_to_front_arc(FRONT_FOV_RAD)
            .crop_distance(min_forward_m, max_lookahead_m)
            .median_filtered(median_window);

        Self { cloud }
    }

    pub(super) fn len(&self) -> usize {
        self.cloud.points.len()
    }

    pub(super) fn points(&self) -> &[LidarPoint] {
        &self.cloud.points
    }

    pub(super) fn find_discontinuity(
        &self,
        min_range_jump_m: f32,
    ) -> Option<ApexDiscontinuity<'_>> {
        let windows = self.cloud.points.windows(2).enumerate();

        // Prefer the closest gap that clears the configured jump threshold;
        // if none does, keep a best-effort derivative so confidence can fall.
        let best_gap = windows
            .clone()
            .filter(|(_, w)| pair_range_jump_m(w) >= min_range_jump_m)
            .min_by(|(_, a), (_, b)| closest_range_m(a).total_cmp(&closest_range_m(b)))
            .or_else(|| {
                windows.max_by(|(_, a), (_, b)| {
                    a[1].derivative(&a[0])
                        .abs()
                        .total_cmp(&b[1].derivative(&b[0]).abs())
                })
            })?;

        let (gap_idx, pair) = best_gap;
        let breakpoint_in_first_wall = pair[0].dist_m < pair[1].dist_m;
        let breakpoint = if breakpoint_in_first_wall {
            &pair[0]
        } else {
            &pair[1]
        };

        Some(ApexDiscontinuity {
            breakpoint,
            gap_idx,
            breakpoint_in_first_wall,
            score_m: pair_range_jump_m(pair),
        })
    }

    pub(super) fn confidence_stats(&self, min_range_jump_m: f32) -> ApexConfidence {
        let mut derivative_score = 0.0;
        let mut range_jump_m = 0.0;
        for pair in self.cloud.points.windows(2) {
            let derivative = pair[1].derivative(&pair[0]).abs();
            if derivative > derivative_score {
                derivative_score = derivative;
                range_jump_m = pair_range_jump_m(pair);
            }
        }

        let confidence = if min_range_jump_m > f32::EPSILON {
            (range_jump_m / min_range_jump_m).clamp(0.0, 1.0)
        } else if range_jump_m > 0.0 {
            1.0
        } else {
            0.0
        };

        ApexConfidence {
            range_jump_m,
            derivative_score,
            confidence,
        }
    }

    pub(super) fn opposite_wall(&self, discontinuity: &ApexDiscontinuity<'_>) -> ApexWall<'_> {
        let points = if discontinuity.breakpoint_in_first_wall {
            &self.cloud.points[discontinuity.gap_idx + 1..]
        } else {
            &self.cloud.points[..=discontinuity.gap_idx]
        };

        ApexWall { points }
    }
}

#[derive(Clone, Copy, Debug)]
pub(super) struct ApexDiscontinuity<'a> {
    pub(super) breakpoint: &'a LidarPoint,
    pub(super) gap_idx: usize,
    pub(super) breakpoint_in_first_wall: bool,
    pub(super) score_m: f32,
}

#[derive(Clone, Copy, Debug)]
pub(super) struct ApexConfidence {
    pub(super) range_jump_m: f32,
    pub(super) derivative_score: f32,
    pub(super) confidence: f32,
}

#[derive(Clone, Copy, Debug)]
pub(super) struct ApexWall<'a> {
    pub(super) points: &'a [LidarPoint],
}

impl<'a> ApexWall<'a> {
    pub(super) fn points(&self) -> &'a [LidarPoint] {
        self.points
    }

    pub(super) fn bounding_points(&self, breakpoint: &LidarPoint) -> Option<HermiteBounds<'a>> {
        if self.points.len() < 2 {
            return None;
        }

        let target = breakpoint.dist_m;
        for (i, pair) in self.points.windows(2).enumerate() {
            let a_dist = pair[0].dist_m;
            let b_dist = pair[1].dist_m;

            if (a_dist..=b_dist).contains(&target) || (b_dist..=a_dist).contains(&target) {
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

#[derive(Clone, Copy, Debug)]
pub(super) struct HermiteBounds<'a> {
    pub(super) prev: &'a LidarPoint,
    pub(super) a: &'a LidarPoint,
    pub(super) b: &'a LidarPoint,
    pub(super) next: &'a LidarPoint,
}

pub(super) fn nearest_front_obstacle_m(cloud: &LidarCloud) -> f32 {
    cloud
        .nearest_in_arc(0.0, PI / 12.0)
        .map(|p| p.dist_m)
        .unwrap_or(0.0)
}

pub(super) fn nearest_obstacle_m(cloud: &LidarCloud) -> f32 {
    cloud
        .points
        .iter()
        .map(|p| p.dist_m)
        .fold(f32::MAX, f32::min)
}

fn pair_range_jump_m(pair: &[LidarPoint]) -> f32 {
    (pair[1].dist_m - pair[0].dist_m).abs()
}

fn closest_range_m(pair: &[LidarPoint]) -> f32 {
    pair[0].dist_m.min(pair[1].dist_m)
}
