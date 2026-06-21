use nfe_core::estimation::PoseMeasurement;
use nfe_core::localization::{LocalizationResult, Localizer};
use nfe_core::mapping::TrackMap;
use nfe_core::params::Tunable;
use nfe_core::sensors::LidarCloud;
use nfe_core::{wrap_angle, Pose2};

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
}

impl Default for ParticleParams {
    fn default() -> Self {
        Self {
            particles: 96,
            sigma_xy_m: 0.20,
            sigma_yaw_rad: 0.15,
            max_points_scored: 24,
            score_sigma_m: 0.25,
        }
    }
}

pub struct ParticleLocalizer {
    params: ParticleParams,
    rng: Rng,
}

impl ParticleLocalizer {
    pub fn new(params: ParticleParams, seed: u64) -> Self {
        Self {
            params,
            rng: Rng::new(seed),
        }
    }
}

impl Localizer for ParticleLocalizer {
    fn localize(&mut self, cloud: &LidarCloud, prior: Pose2, map: &TrackMap) -> LocalizationResult {
        if map.boundaries.walls.is_empty() || cloud.points.is_empty() {
            return LocalizationResult::default();
        }
        let mut best = prior;
        let mut best_score = f32::NEG_INFINITY;
        for _ in 0..self.params.particles {
            let p = Pose2::new(
                prior.x + self.rng.normal() * self.params.sigma_xy_m,
                prior.y + self.rng.normal() * self.params.sigma_xy_m,
                wrap_angle(prior.yaw + self.rng.normal() * self.params.sigma_yaw_rad),
            );
            let score = score_pose(
                p,
                cloud,
                map,
                self.params.max_points_scored,
                self.params.score_sigma_m,
            );
            if score > best_score {
                best_score = score;
                best = p;
            }
        }
        let confidence = best_score.exp().clamp(0.0, 1.0);
        LocalizationResult {
            measurement: Some(PoseMeasurement {
                pose: best,
                quality: confidence,
            }),
            confidence,
            residual_m: (-best_score).sqrt().min(f32::MAX),
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
) -> f32 {
    let stride = (cloud.points.len() / max_points.max(1)).max(1);
    let mut n = 0usize;
    let mut sum2 = 0.0f32;
    for lp in cloud.points.iter().step_by(stride).take(max_points) {
        let wp = pose.transform_point(&lp.point2());
        let dist = map
            .boundaries
            .walls
            .iter()
            .map(|w| w.signed_distance(&wp).abs())
            .fold(f32::INFINITY, f32::min);
        if dist.is_finite() {
            sum2 += dist * dist;
            n += 1;
        }
    }
    if n == 0 {
        return f32::NEG_INFINITY;
    }
    -0.5 * (sum2 / n as f32) / (sigma * sigma).max(1e-6)
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
