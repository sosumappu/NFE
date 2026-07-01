use nfe_core::estimation::PoseMeasurement;
use nfe_core::localization::{LocalizationResult, Localizer};
use nfe_core::mapping::{DistanceField, TrackMap};
use nfe_core::params::Tunable;
use nfe_core::sensors::LidarCloud;
use nfe_core::{wrap_angle, Pose2};

use crate::mapping::distance_at;

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize, Tunable)]
#[serde(default)]
pub struct ParticleParams {
    #[param(int, 16..512, default = 96)]
    pub particles: usize,
    #[param(0.01..1.0, default = 0.20)]
    pub sigma_xy_m: f32,
    #[param(0.005..1.0, default = 0.15)]
    pub sigma_yaw_rad: f32,
    #[param(int, 4..80, default = 24)]
    pub max_points_scored: usize,
    #[param(0.02..1.0, default = 0.25)]
    pub score_sigma_m: f32,
    #[param(0.1..1.0, default = 0.5)]
    pub resample_ess_fraction: f32,
    #[param(0.0..1.0, default = 0.15)]
    pub recovery_fraction: f32,
    #[param(0.0..1.0, default = 0.15)]
    pub min_confidence: f32,
}

impl Default for ParticleParams {
    fn default() -> Self {
        Self {
            particles: 96,
            sigma_xy_m: 0.20,
            sigma_yaw_rad: 0.15,
            max_points_scored: 24,
            score_sigma_m: 0.25,
            resample_ess_fraction: 0.5,
            recovery_fraction: 0.15,
            min_confidence: 0.15,
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct Particle {
    pose: Pose2,
    weight: f32,
}

#[derive(Clone, Copy, Debug)]
struct PoseScore {
    log_likelihood: f32,
    residual_m: f32,
}

pub struct ParticleLocalizer {
    params: ParticleParams,
    rng: Rng,
    particles: Vec<Particle>,
    last_prior: Option<Pose2>,
    last_ess: f32,
    resample_count: u64,
    update_count: u64,
    global_sample_index: u64,
}

impl ParticleLocalizer {
    pub fn new(params: ParticleParams, seed: u64) -> Self {
        Self {
            params,
            rng: Rng::new(seed),
            particles: Vec::new(),
            last_prior: None,
            last_ess: 0.0,
            resample_count: 0,
            update_count: 0,
            global_sample_index: 0,
        }
    }

    pub fn update_count(&self) -> u64 {
        self.update_count
    }

    pub fn resample_count(&self) -> u64 {
        self.resample_count
    }

    pub fn last_ess(&self) -> f32 {
        self.last_ess
    }

    fn ensure_particles(&mut self, prior: Pose2, field: &DistanceField) {
        let n = self.params.particles.max(1);
        if self.particles.len() == n {
            return;
        }
        self.particles.clear();
        self.particles.reserve(n);
        let global = self.recovery_count(n);
        let local = n.saturating_sub(global);
        let weight = 1.0 / n as f32;
        for _ in 0..local {
            let pose = self.sample_around(prior);
            self.particles.push(Particle { pose, weight });
        }
        for _ in 0..global {
            let pose = self.sample_global(field);
            self.particles.push(Particle { pose, weight });
        }
        self.last_prior = Some(prior);
    }

    fn predict_from_prior_delta(&mut self, prior: Pose2) {
        let Some(last) = self.last_prior.replace(prior) else {
            return;
        };
        let dx = prior.x - last.x;
        let dy = prior.y - last.y;
        let dyaw = wrap_angle(prior.yaw - last.yaw);
        let sigma_xy = self.params.sigma_xy_m * 0.25;
        let sigma_yaw = self.params.sigma_yaw_rad * 0.25;
        let len = self.particles.len();
        for idx in 0..len {
            let nx = self.rng.normal() * sigma_xy;
            let ny = self.rng.normal() * sigma_xy;
            let nyaw = self.rng.normal() * sigma_yaw;
            let p = &mut self.particles[idx];
            p.pose.x += dx + nx;
            p.pose.y += dy + ny;
            p.pose.yaw = wrap_angle(p.pose.yaw + dyaw + nyaw);
        }
    }

    fn update_weights(&mut self, cloud: &LidarCloud, map: &TrackMap) -> Option<(usize, PoseScore)> {
        let mut scores = Vec::with_capacity(self.particles.len());
        let mut best_idx = 0usize;
        let mut best_score = PoseScore {
            log_likelihood: f32::NEG_INFINITY,
            residual_m: f32::INFINITY,
        };
        for (idx, particle) in self.particles.iter().enumerate() {
            let score = score_pose(
                particle.pose,
                cloud,
                map,
                self.params.max_points_scored,
                self.params.score_sigma_m,
            );
            if score_order(&score, &best_score).is_gt() {
                best_idx = idx;
                best_score = score;
            }
            scores.push(score);
        }
        if !best_score.log_likelihood.is_finite() {
            return None;
        }

        let mut weight_sum = 0.0f32;
        for (particle, score) in self.particles.iter_mut().zip(scores.iter()) {
            particle.weight = (score.log_likelihood - best_score.log_likelihood).exp();
            weight_sum += particle.weight;
        }
        if weight_sum <= 0.0 || !weight_sum.is_finite() {
            return None;
        }
        for particle in &mut self.particles {
            particle.weight /= weight_sum;
        }
        self.last_ess = effective_sample_size(&self.particles);
        Some((best_idx, best_score))
    }

    fn maybe_resample(&mut self, confidence: f32, field: &DistanceField) {
        let n = self.particles.len();
        if n == 0 {
            return;
        }
        let threshold = self.params.resample_ess_fraction.clamp(0.0, 1.0) * n as f32;
        if self.last_ess < threshold {
            self.systematic_resample();
            self.resample_count = self.resample_count.saturating_add(1);
        }
        if confidence < self.params.min_confidence {
            self.inject_recovery(field);
        }
    }

    fn systematic_resample(&mut self) {
        let n = self.particles.len();
        if n == 0 {
            return;
        }
        let mut out = Vec::with_capacity(n);
        let step = 1.0 / n as f32;
        let mut target = self.rng.uniform() * step;
        let mut cumulative = self.particles[0].weight;
        let mut idx = 0usize;
        for _ in 0..n {
            while target > cumulative && idx + 1 < n {
                idx += 1;
                cumulative += self.particles[idx].weight;
            }
            let pose = self.jitter_pose(self.particles[idx].pose);
            out.push(Particle { pose, weight: step });
            target += step;
        }
        self.particles = out;
    }

    fn inject_recovery(&mut self, field: &DistanceField) {
        let n = self.recovery_count(self.particles.len());
        if n == 0 {
            return;
        }
        let start = self.particles.len().saturating_sub(n);
        let weight = if self.particles.is_empty() {
            1.0
        } else {
            1.0 / self.particles.len() as f32
        };
        for idx in start..self.particles.len() {
            self.particles[idx] = Particle {
                pose: self.sample_global(field),
                weight,
            };
        }
        for particle in &mut self.particles[..start] {
            particle.weight = weight;
        }
    }

    fn recovery_count(&self, len: usize) -> usize {
        ((len as f32) * self.params.recovery_fraction.clamp(0.0, 1.0)).round() as usize
    }

    fn sample_around(&mut self, center: Pose2) -> Pose2 {
        Pose2::new(
            center.x + self.rng.normal() * self.params.sigma_xy_m,
            center.y + self.rng.normal() * self.params.sigma_xy_m,
            wrap_angle(center.yaw + self.rng.normal() * self.params.sigma_yaw_rad),
        )
    }

    fn jitter_pose(&mut self, pose: Pose2) -> Pose2 {
        Pose2::new(
            pose.x + self.rng.normal() * self.params.sigma_xy_m * 0.2,
            pose.y + self.rng.normal() * self.params.sigma_xy_m * 0.2,
            wrap_angle(pose.yaw + self.rng.normal() * self.params.sigma_yaw_rad * 0.2),
        )
    }

    fn sample_global(&mut self, field: &DistanceField) -> Pose2 {
        let idx = self.global_sample_index;
        self.global_sample_index = self.global_sample_index.saturating_add(1);
        let width_m = field.width as f32 * field.resolution_m;
        let height_m = field.height as f32 * field.resolution_m;
        let x = field.origin.x + halton(idx + 1, 2) * width_m;
        let y = field.origin.y + halton(idx + 1, 3) * height_m;
        let yaw = if idx.is_multiple_of(7) {
            0.0
        } else {
            -std::f32::consts::PI + halton(idx + 1, 5) * std::f32::consts::TAU
        };
        Pose2::new(x, y, wrap_angle(yaw))
    }
}

impl Localizer for ParticleLocalizer {
    fn localize(&mut self, cloud: &LidarCloud, prior: Pose2, map: &TrackMap) -> LocalizationResult {
        let Some(field) = map.distance_field.as_ref() else {
            return LocalizationResult::default();
        };
        if cloud.points.is_empty() {
            return LocalizationResult::default();
        }
        self.update_count = self.update_count.saturating_add(1);
        self.ensure_particles(prior, field);
        self.predict_from_prior_delta(prior);
        let Some((best_idx, best_score)) = self.update_weights(cloud, map) else {
            self.inject_recovery(field);
            return LocalizationResult::default();
        };
        let best_pose = self.particles[best_idx].pose;
        let confidence = best_score.log_likelihood.exp().clamp(0.0, 1.0);
        self.maybe_resample(confidence, field);
        if confidence < self.params.min_confidence {
            return LocalizationResult {
                confidence,
                residual_m: best_score.residual_m,
                used_fallback: true,
                ..Default::default()
            };
        }
        LocalizationResult {
            measurement: Some(PoseMeasurement {
                pose: best_pose,
                quality: confidence,
            }),
            confidence,
            residual_m: best_score.residual_m,
            used_fallback: true,
        }
    }
}

fn score_pose(
    pose: Pose2,
    cloud: &LidarCloud,
    map: &TrackMap,
    max_points: usize,
    sigma: f32,
) -> PoseScore {
    let Some(field) = map.distance_field.as_ref() else {
        return PoseScore {
            log_likelihood: f32::NEG_INFINITY,
            residual_m: f32::INFINITY,
        };
    };
    let stride = (cloud.points.len() / max_points.max(1)).max(1);
    let mut n = 0usize;
    let mut sum2 = 0.0f32;
    let mut residual_sum = 0.0f32;
    for lp in cloud.points.iter().step_by(stride).take(max_points) {
        let wp = pose.transform_point(&lp.point2());
        if let Some(dist) = distance_at(field, wp) {
            sum2 += dist * dist;
            residual_sum += dist;
            n += 1;
        }
    }
    if n == 0 {
        return PoseScore {
            log_likelihood: f32::NEG_INFINITY,
            residual_m: f32::INFINITY,
        };
    }
    PoseScore {
        log_likelihood: -0.5 * (sum2 / n as f32) / (sigma * sigma).max(1e-6),
        residual_m: residual_sum / n as f32,
    }
}

fn effective_sample_size(particles: &[Particle]) -> f32 {
    let sum2: f32 = particles.iter().map(|p| p.weight * p.weight).sum();
    if sum2 <= 0.0 {
        0.0
    } else {
        1.0 / sum2
    }
}

fn score_order(lhs: &PoseScore, rhs: &PoseScore) -> std::cmp::Ordering {
    lhs.log_likelihood
        .total_cmp(&rhs.log_likelihood)
        .then_with(|| rhs.residual_m.total_cmp(&lhs.residual_m))
}

fn halton(mut index: u64, base: u64) -> f32 {
    let mut f = 1.0f32;
    let mut r = 0.0f32;
    while index > 0 {
        f /= base as f32;
        r += f * (index % base) as f32;
        index /= base;
    }
    r
}

struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Self(seed | 1)
    }
    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545F4914F6CDD1D)
    }
    fn uniform(&mut self) -> f32 {
        let v = self.next_u64() >> 40;
        (v as f32) / ((1u64 << 24) as f32)
    }
    /// Box-Muller normal; deterministic and adequate for fallback sampling.
    fn normal(&mut self) -> f32 {
        let u1 = self.uniform().clamp(1e-6, 1.0);
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (std::f32::consts::TAU * u2).cos()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mapping::compute_distance_field;
    use nfe_core::mapping::OccupancyGrid;
    use nfe_core::sensors::LidarPoint;
    use nfe_core::Point2;

    fn l_shape_world_points() -> Vec<Point2> {
        let mut points = Vec::new();
        for i in 0..=20 {
            let t = -1.0 + i as f32 * 0.10;
            points.push(Point2::new(1.0, t));
        }
        for i in 0..=20 {
            let t = -1.0 + i as f32 * 0.10;
            points.push(Point2::new(t, 1.0));
        }
        points
    }

    fn l_shape_map() -> TrackMap {
        let resolution = 0.05;
        let width = 80u32;
        let height = 80u32;
        let origin = Point2::new(-2.0, -2.0);
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
        let points = l_shape_world_points()
            .into_iter()
            .map(|world| {
                let dx = world.x - pose.x;
                let dy = world.y - pose.y;
                let x = c * dx + s * dy;
                let y = -s * dx + c * dy;
                LidarPoint {
                    x,
                    y,
                    dist_m: x.hypot(y),
                    angle_rad: y.atan2(x),
                    timestamp_us: 1,
                }
            })
            .collect();
        LidarCloud {
            points,
            timestamp_us: 1,
        }
    }

    fn test_params() -> ParticleParams {
        ParticleParams {
            particles: 256,
            sigma_xy_m: 0.20,
            sigma_yaw_rad: 0.15,
            max_points_scored: 48,
            score_sigma_m: 0.15,
            resample_ess_fraction: 0.9,
            recovery_fraction: 0.35,
            min_confidence: 0.20,
        }
    }

    #[test]
    fn ess_below_threshold_triggers_resampling() {
        let map = l_shape_map();
        let scan = scan_from_pose(Pose2::default());
        let mut loc = ParticleLocalizer::new(
            ParticleParams {
                resample_ess_fraction: 1.0,
                recovery_fraction: 0.0,
                min_confidence: 0.0,
                ..test_params()
            },
            7,
        );

        let out = loc.localize(&scan, Pose2::new(0.05, -0.05, 0.03), &map);

        assert!(out.used_fallback);
        assert!(loc.last_ess() < loc.params.particles as f32);
        assert!(loc.resample_count() > 0);
    }

    #[test]
    fn kidnapped_robot_recovers_with_recovery_particles() {
        let map = l_shape_map();
        let truth = Pose2::default();
        let scan = scan_from_pose(truth);
        let kidnapped_prior = Pose2::new(1.4, -1.2, 0.9);
        let mut loc = ParticleLocalizer::new(test_params(), 0xBEEF);
        let mut best = LocalizationResult::default();

        for _ in 0..40 {
            best = loc.localize(&scan, kidnapped_prior, &map);
            if best.confidence > 0.65 {
                break;
            }
        }

        let pose = best.measurement.expect("MCL should recover").pose;
        let pos_err = Point2::new(pose.x, pose.y).dist(&Point2::new(truth.x, truth.y));
        let yaw_err = wrap_angle(pose.yaw - truth.yaw).abs();
        assert!(pos_err < 0.25, "pose={pose:?} pos_err={pos_err}");
        assert!(yaw_err < 0.25, "pose={pose:?} yaw_err={yaw_err}");
        assert!(best.used_fallback);
    }
}
