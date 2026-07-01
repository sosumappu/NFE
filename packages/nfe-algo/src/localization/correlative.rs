use rayon::prelude::*;

use nfe_core::estimation::PoseMeasurement;
use nfe_core::localization::{LocalizationResult, Localizer};
use nfe_core::mapping::{DistanceField, TrackMap};
use nfe_core::params::Tunable;
use nfe_core::sensors::LidarCloud;
use nfe_core::{wrap_angle, Pose2};

use crate::mapping::distance_at;

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize, Tunable)]
#[serde(default)]
pub struct CorrelativeParams {
    #[param(0.02..1.0, default = 0.25)]
    pub search_window_xy_m: f32,
    #[param(0.01..1.57, default = 0.25)]
    pub search_window_yaw_rad: f32,
    #[param(0.02..0.25, default = 0.05)]
    pub coarse_xy_step_m: f32,
    #[param(0.01..0.25, default = 0.05)]
    pub coarse_yaw_step_rad: f32,
    #[param(0.005..0.10, default = 0.01)]
    pub fine_xy_step_m: f32,
    #[param(0.002..0.10, default = 0.01)]
    pub fine_yaw_step_rad: f32,
    #[param(int, 4..256, default = 12)]
    pub min_points: usize,
    #[param(0.02..1.0, default = 0.12)]
    pub score_sigma_m: f32,
    #[param(0.0..1.0, default = 0.25)]
    pub min_confidence: f32,
    #[tunable(skip)]
    pub top_coarse_candidates: usize,
}

impl Default for CorrelativeParams {
    fn default() -> Self {
        Self {
            search_window_xy_m: 0.25,
            search_window_yaw_rad: 0.25,
            coarse_xy_step_m: 0.05,
            coarse_yaw_step_rad: 0.05,
            fine_xy_step_m: 0.01,
            fine_yaw_step_rad: 0.01,
            min_points: 12,
            score_sigma_m: 0.12,
            min_confidence: 0.25,
            top_coarse_candidates: 16,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CandidateScore {
    pub pose: Pose2,
    pub score: f32,
    pub residual_m: f32,
    pub matched_points: u32,
}

pub trait ScanScorer: Send + Sync {
    fn score_candidates(
        &self,
        field: &DistanceField,
        cloud: &LidarCloud,
        candidates: &[Pose2],
        params: &CorrelativeParams,
    ) -> Vec<CandidateScore>;
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CpuSerialScanScorer;

impl ScanScorer for CpuSerialScanScorer {
    fn score_candidates(
        &self,
        field: &DistanceField,
        cloud: &LidarCloud,
        candidates: &[Pose2],
        params: &CorrelativeParams,
    ) -> Vec<CandidateScore> {
        candidates
            .iter()
            .copied()
            .map(|pose| score_pose(field, cloud, pose, params))
            .collect()
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CpuParallelScanScorer;

impl ScanScorer for CpuParallelScanScorer {
    fn score_candidates(
        &self,
        field: &DistanceField,
        cloud: &LidarCloud,
        candidates: &[Pose2],
        params: &CorrelativeParams,
    ) -> Vec<CandidateScore> {
        candidates
            .par_iter()
            .copied()
            .map(|pose| score_pose(field, cloud, pose, params))
            .collect()
    }
}

pub struct CorrelativeLocalizer<S = CpuParallelScanScorer> {
    params: CorrelativeParams,
    scorer: S,
}

impl CorrelativeLocalizer<CpuParallelScanScorer> {
    pub fn new(params: CorrelativeParams) -> Self {
        Self {
            params,
            scorer: CpuParallelScanScorer,
        }
    }
}

impl<S: ScanScorer> CorrelativeLocalizer<S> {
    pub fn with_scorer(params: CorrelativeParams, scorer: S) -> Self {
        Self { params, scorer }
    }
}

impl<S: ScanScorer> Localizer for CorrelativeLocalizer<S> {
    fn localize(&mut self, cloud: &LidarCloud, prior: Pose2, map: &TrackMap) -> LocalizationResult {
        let Some(field) = map.distance_field.as_ref() else {
            return LocalizationResult::default();
        };
        let Some(best) = coarse_to_fine_match(field, cloud, prior, &self.params, &self.scorer)
        else {
            return LocalizationResult::default();
        };
        if best.score < self.params.min_confidence {
            return LocalizationResult {
                confidence: best.score,
                residual_m: best.residual_m,
                ..Default::default()
            };
        }
        LocalizationResult {
            measurement: Some(PoseMeasurement {
                pose: best.pose,
                quality: best.score,
            }),
            confidence: best.score,
            residual_m: best.residual_m,
            used_fallback: false,
        }
    }
}

pub fn coarse_to_fine_match<S: ScanScorer>(
    field: &DistanceField,
    cloud: &LidarCloud,
    prior: Pose2,
    params: &CorrelativeParams,
    scorer: &S,
) -> Option<CandidateScore> {
    if cloud.points.len() < params.min_points {
        return None;
    }

    let coarse = candidate_grid(
        prior,
        params.search_window_xy_m,
        params.search_window_yaw_rad,
        params.coarse_xy_step_m,
        params.coarse_yaw_step_rad,
    );
    let mut coarse_scores = scorer.score_candidates(field, cloud, &coarse, params);
    coarse_scores.sort_by(|a, b| score_order(b, a));
    let keep = params.top_coarse_candidates.max(1).min(coarse_scores.len());

    let mut fine_candidates = Vec::new();
    for candidate in coarse_scores.iter().take(keep) {
        fine_candidates.extend(candidate_grid(
            candidate.pose,
            params.coarse_xy_step_m,
            params.coarse_yaw_step_rad,
            params.fine_xy_step_m,
            params.fine_yaw_step_rad,
        ));
    }
    if fine_candidates.is_empty() {
        return coarse_scores.into_iter().next();
    }
    scorer
        .score_candidates(field, cloud, &fine_candidates, params)
        .into_iter()
        .max_by(score_order)
}

pub fn brute_force_match<S: ScanScorer>(
    field: &DistanceField,
    cloud: &LidarCloud,
    prior: Pose2,
    params: &CorrelativeParams,
    scorer: &S,
) -> Option<CandidateScore> {
    if cloud.points.len() < params.min_points {
        return None;
    }
    let candidates = candidate_grid(
        prior,
        params.search_window_xy_m,
        params.search_window_yaw_rad,
        params.fine_xy_step_m,
        params.fine_yaw_step_rad,
    );
    scorer
        .score_candidates(field, cloud, &candidates, params)
        .into_iter()
        .max_by(score_order)
}

pub fn candidate_grid(
    center: Pose2,
    window_xy_m: f32,
    window_yaw_rad: f32,
    step_xy_m: f32,
    step_yaw_rad: f32,
) -> Vec<Pose2> {
    let xs = offsets(window_xy_m, step_xy_m);
    let ys = offsets(window_xy_m, step_xy_m);
    let yaws = offsets(window_yaw_rad, step_yaw_rad);
    let mut out = Vec::with_capacity(xs.len() * ys.len() * yaws.len());
    for dx in &xs {
        for dy in &ys {
            for dyaw in &yaws {
                out.push(Pose2::new(
                    center.x + dx,
                    center.y + dy,
                    wrap_angle(center.yaw + dyaw),
                ));
            }
        }
    }
    out
}

fn offsets(window: f32, step: f32) -> Vec<f32> {
    let step = step.max(1.0e-6);
    let n = (window.max(0.0) / step).floor() as i32;
    (-n..=n).map(|i| i as f32 * step).collect()
}

fn score_pose(
    field: &DistanceField,
    cloud: &LidarCloud,
    pose: Pose2,
    params: &CorrelativeParams,
) -> CandidateScore {
    let sigma2 = (params.score_sigma_m * params.score_sigma_m).max(1.0e-9);
    let mut likelihood_sum = 0.0f32;
    let mut residual_sum = 0.0f32;
    let mut matched = 0u32;

    for point in &cloud.points {
        let world = pose.transform_point(&point.point2());
        if let Some(distance) = distance_at(field, world) {
            likelihood_sum += (-0.5 * distance * distance / sigma2).exp();
            residual_sum += distance;
            matched += 1;
        }
    }

    if matched < params.min_points as u32 || cloud.points.is_empty() {
        return CandidateScore {
            pose,
            score: f32::NEG_INFINITY,
            residual_m: f32::INFINITY,
            matched_points: matched,
        };
    }

    CandidateScore {
        pose,
        score: likelihood_sum / cloud.points.len() as f32,
        residual_m: residual_sum / matched as f32,
        matched_points: matched,
    }
}

fn score_order(a: &CandidateScore, b: &CandidateScore) -> std::cmp::Ordering {
    a.score
        .total_cmp(&b.score)
        .then_with(|| b.residual_m.total_cmp(&a.residual_m))
        .then_with(|| a.pose.x.total_cmp(&b.pose.x))
        .then_with(|| a.pose.y.total_cmp(&b.pose.y))
        .then_with(|| a.pose.yaw.total_cmp(&b.pose.yaw))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mapping::compute_distance_field;
    use nfe_core::mapping::OccupancyGrid;
    use nfe_core::sensors::LidarPoint;
    use nfe_core::Point2;

    const POSITION_TOLERANCE_M: f32 = 0.04;
    const YAW_TOLERANCE_RAD: f32 = 3.0_f32.to_radians();

    fn test_params() -> CorrelativeParams {
        CorrelativeParams {
            search_window_xy_m: 0.12,
            search_window_yaw_rad: 0.10,
            coarse_xy_step_m: 0.04,
            coarse_yaw_step_rad: 0.04,
            fine_xy_step_m: 0.01,
            fine_yaw_step_rad: 0.01,
            min_points: 8,
            score_sigma_m: 0.08,
            min_confidence: 0.10,
            top_coarse_candidates: 64,
        }
    }

    fn l_shape_world_points() -> Vec<Point2> {
        let mut points = Vec::new();
        for i in 0..=30 {
            let t = -1.5 + i as f32 * 0.10;
            points.push(Point2::new(1.8, t));
        }
        for i in 0..=25 {
            let t = -0.7 + i as f32 * 0.10;
            points.push(Point2::new(t, 1.2));
        }
        points
    }

    fn l_shape_map() -> TrackMap {
        let resolution = 0.05;
        let width = 120u32;
        let height = 120u32;
        let origin = Point2::new(-3.0, -3.0);
        let mut grid = OccupancyGrid {
            origin,
            resolution_m: resolution,
            width,
            height,
            cells: vec![0.0; width as usize * height as usize],
        };
        for p in l_shape_world_points() {
            let ix = ((p.x - origin.x) / resolution).floor() as i32;
            let iy = ((p.y - origin.y) / resolution).floor() as i32;
            if ix >= 0 && iy >= 0 && ix < width as i32 && iy < height as i32 {
                grid.cells[iy as usize * width as usize + ix as usize] = 4.0;
            }
        }
        let distance_field = compute_distance_field(&grid, 1.0);
        TrackMap {
            occupancy: Some(grid),
            distance_field: Some(distance_field),
            complete: true,
            revision: 1,
            ..Default::default()
        }
    }

    fn scan_from_pose(pose: Pose2) -> LidarCloud {
        let (s, c) = pose.yaw.sin_cos();
        let mut points = Vec::new();
        for world in l_shape_world_points() {
            let dx = world.x - pose.x;
            let dy = world.y - pose.y;
            let x = c * dx + s * dy;
            let y = -s * dx + c * dy;
            let dist_m = x.hypot(y);
            points.push(LidarPoint {
                x,
                y,
                dist_m,
                angle_rad: y.atan2(x),
                timestamp_us: 1,
            });
        }
        LidarCloud {
            points,
            timestamp_us: 1,
        }
    }

    fn assert_pose_close(actual: Pose2, expected: Pose2) {
        let pos_err = Point2::new(actual.x, actual.y).dist(&Point2::new(expected.x, expected.y));
        let yaw_err = wrap_angle(actual.yaw - expected.yaw).abs();
        assert!(
            pos_err <= POSITION_TOLERANCE_M,
            "pos_err={pos_err} actual={actual:?} expected={expected:?}"
        );
        assert!(
            yaw_err <= YAW_TOLERANCE_RAD,
            "yaw_err={yaw_err} actual={actual:?} expected={expected:?}"
        );
    }

    #[test]
    fn synthetic_scan_known_transform_recovers_pose_within_four_cm_and_three_deg() {
        let map = l_shape_map();
        let field = map.distance_field.as_ref().unwrap();
        let true_pose = Pose2::new(0.31, -0.24, 0.13);
        let prior = Pose2::new(0.37, -0.29, 0.18);
        let scan = scan_from_pose(true_pose);
        let params = test_params();

        let best = coarse_to_fine_match(field, &scan, prior, &params, &CpuSerialScanScorer)
            .expect("match should succeed");

        assert_pose_close(best.pose, true_pose);
        assert!(best.score > 0.80, "best={best:?}");
    }

    #[test]
    fn coarse_to_fine_matches_brute_force_on_small_grid() {
        let map = l_shape_map();
        let field = map.distance_field.as_ref().unwrap();
        let true_pose = Pose2::new(0.31, -0.24, 0.13);
        let prior = Pose2::new(0.37, -0.29, 0.18);
        let scan = scan_from_pose(true_pose);
        let params = test_params();

        let coarse = coarse_to_fine_match(field, &scan, prior, &params, &CpuSerialScanScorer)
            .expect("coarse-to-fine match");
        let brute = brute_force_match(field, &scan, prior, &params, &CpuSerialScanScorer)
            .expect("brute-force match");

        assert_eq!(coarse.pose, brute.pose);
        assert_eq!(coarse.score.to_bits(), brute.score.to_bits());
        assert_eq!(coarse.residual_m.to_bits(), brute.residual_m.to_bits());
    }

    #[test]
    fn serial_and_parallel_scorers_are_identical() {
        let map = l_shape_map();
        let field = map.distance_field.as_ref().unwrap();
        let scan = scan_from_pose(Pose2::new(0.31, -0.24, 0.13));
        let params = test_params();
        let candidates = candidate_grid(Pose2::new(0.37, -0.29, 0.18), 0.04, 0.04, 0.02, 0.02);

        let serial = CpuSerialScanScorer.score_candidates(field, &scan, &candidates, &params);
        let parallel = CpuParallelScanScorer.score_candidates(field, &scan, &candidates, &params);

        assert_eq!(serial.len(), parallel.len());
        for (a, b) in serial.iter().zip(parallel.iter()) {
            assert_eq!(a.pose, b.pose);
            assert_eq!(a.score.to_bits(), b.score.to_bits());
            assert_eq!(a.residual_m.to_bits(), b.residual_m.to_bits());
            assert_eq!(a.matched_points, b.matched_points);
        }
    }

    #[test]
    fn replayed_synthetic_scans_match_ground_truth_pose() {
        let map = l_shape_map();
        let params = test_params();
        let mut localizer = CorrelativeLocalizer::with_scorer(params, CpuSerialScanScorer);
        let replay = [
            Pose2::new(0.25, -0.30, 0.08),
            Pose2::new(0.31, -0.24, 0.13),
            Pose2::new(0.37, -0.18, 0.18),
        ];

        for truth in replay {
            let scan = scan_from_pose(truth);
            let prior = Pose2::new(truth.x + 0.05, truth.y - 0.04, truth.yaw + 0.04);
            let out = localizer.localize(&scan, prior, &map);
            let measured = out.measurement.expect("localization measurement").pose;
            assert_pose_close(measured, truth);
        }
    }
}
