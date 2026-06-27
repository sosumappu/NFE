use anyhow::Result;
use nfe_core::control::ControllerStatus;
use nfe_core::io::{ActuatorSink, SensorSource};
use nfe_core::Pose2;
use nfe_runtime::config::RuntimeConfig;
use nfe_runtime::pipeline::{EstimatorMode, Pipeline, StepOutput};
use nfe_sim::{
    LatencyParams, SimulatorSource, TrackProgress, VehicleFootprintParams, VehicleModel,
    VehicleState, World,
};

#[derive(Clone, Copy, Debug)]
pub struct SimTuningObjective {
    pub target_laps: u32,
    pub target_speed_ms: f32,
    pub min_avg_speed_ms: f32,
    pub max_ticks: usize,
    pub dt_s: f32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SimEpisodeScore {
    pub cost: f64,
    pub completed_laps: u32,
    pub progress_m: f32,
    pub progress_ratio: f32,
    pub finish_time_s: Option<f32>,
    pub avg_speed_ms: f32,
    pub max_speed_ms: f32,
    pub crashed: bool,
    pub lateral_rms_m: f32,
    pub heading_rms_rad: f32,
    pub steering_rate_rms: f32,
    pub throttle_rate_rms: f32,
    pub unavailable_fraction: f32,
    pub ticks: u64,
}

pub fn evaluate_sim_laps(
    cfg: RuntimeConfig,
    world: World,
    model: Box<dyn VehicleModel>,
    seed: Option<u64>,
    latency: LatencyParams,
    footprint: VehicleFootprintParams,
    objective: &SimTuningObjective,
) -> Result<SimEpisodeScore> {
    let mut progress = TrackProgress::from_waypoints(&world.waypoints)?;
    let track_length_m = progress.length_m();
    let required_progress_m = objective.target_laps.max(1) as f32 * track_length_m;
    let (mut source, mut actuator) = if let Some(seed) = seed {
        SimulatorSource::new_with_seed_latency_and_footprint(
            world.clone(),
            model,
            objective.dt_s,
            seed,
            latency,
            footprint,
        )
    } else {
        SimulatorSource::new_with_latency_and_footprint(
            world.clone(),
            model,
            objective.dt_s,
            latency,
            footprint,
        )
    };
    let mut pipeline = Pipeline::new(cfg, EstimatorMode::DeadReckon);
    pipeline.reset(
        Pose2::new(world.start.x, world.start.y, world.start.yaw_rad),
        0,
    );
    let mut accumulator = SimScoreAccumulator::new(*objective, required_progress_m);

    for _ in 0..objective.max_ticks {
        let Some(snapshot) = source.next_snapshot()? else {
            accumulator.crashed = source.is_exhausted();
            break;
        };
        let step = pipeline.step(snapshot);
        actuator.apply(&step.command)?;
        let state = source.vehicle_state();
        let sample = progress.update(state.x, state.y, state.yaw_rad);
        accumulator.update(&step, state, sample);
        if sample.unwrapped_s_m >= required_progress_m {
            accumulator.finish_time_s = Some(source.timestamp_us() as f32 / 1_000_000.0);
            break;
        }
    }

    actuator.safe_state()?;
    Ok(accumulator.finish())
}

pub fn aggregate_sim_scores(scores: &[SimEpisodeScore], robustness_weight: f64) -> SimEpisodeScore {
    if scores.is_empty() {
        return SimEpisodeScore {
            cost: 1.0e9,
            ..Default::default()
        };
    }

    let mean_cost = scores.iter().map(|s| s.cost).sum::<f64>() / scores.len() as f64;
    let variance = scores
        .iter()
        .map(|s| {
            let d = s.cost - mean_cost;
            d * d
        })
        .sum::<f64>()
        / scores.len() as f64;
    let cost = mean_cost + robustness_weight * variance.sqrt();

    let mut out = scores
        .iter()
        .copied()
        .min_by(|a, b| a.cost.partial_cmp(&b.cost).unwrap())
        .unwrap_or_default();
    out.cost = cost;
    out.completed_laps = scores.iter().map(|s| s.completed_laps).min().unwrap_or(0);
    out.progress_m = scores.iter().map(|s| s.progress_m).sum::<f32>() / scores.len() as f32;
    out.progress_ratio = scores.iter().map(|s| s.progress_ratio).sum::<f32>() / scores.len() as f32;
    out.avg_speed_ms = scores.iter().map(|s| s.avg_speed_ms).sum::<f32>() / scores.len() as f32;
    out.max_speed_ms = scores.iter().map(|s| s.max_speed_ms).fold(0.0, f32::max);
    out.crashed = scores.iter().any(|s| s.crashed);
    out.lateral_rms_m = scores.iter().map(|s| s.lateral_rms_m).sum::<f32>() / scores.len() as f32;
    out.heading_rms_rad =
        scores.iter().map(|s| s.heading_rms_rad).sum::<f32>() / scores.len() as f32;
    out.steering_rate_rms =
        scores.iter().map(|s| s.steering_rate_rms).sum::<f32>() / scores.len() as f32;
    out.throttle_rate_rms =
        scores.iter().map(|s| s.throttle_rate_rms).sum::<f32>() / scores.len() as f32;
    out.unavailable_fraction =
        scores.iter().map(|s| s.unavailable_fraction).sum::<f32>() / scores.len() as f32;
    out.ticks = scores.iter().map(|s| s.ticks).sum::<u64>() / scores.len() as u64;
    out
}

#[derive(Clone, Copy, Debug)]
struct SimScoreAccumulator {
    objective: SimTuningObjective,
    required_progress_m: f32,
    finish_time_s: Option<f32>,
    crashed: bool,
    ticks: u64,
    progress_m: f32,
    completed_laps: u32,
    speed_sum: f64,
    max_speed_ms: f32,
    lateral2: f64,
    heading2: f64,
    steering_rate2: f64,
    throttle_rate2: f64,
    unavailable: u64,
    last_steering_rad: Option<f32>,
    last_throttle: Option<f32>,
}

impl SimScoreAccumulator {
    fn new(objective: SimTuningObjective, required_progress_m: f32) -> Self {
        Self {
            objective,
            required_progress_m,
            finish_time_s: None,
            crashed: false,
            ticks: 0,
            progress_m: 0.0,
            completed_laps: 0,
            speed_sum: 0.0,
            max_speed_ms: 0.0,
            lateral2: 0.0,
            heading2: 0.0,
            steering_rate2: 0.0,
            throttle_rate2: 0.0,
            unavailable: 0,
            last_steering_rad: None,
            last_throttle: None,
        }
    }

    fn update(&mut self, step: &StepOutput, state: VehicleState, sample: nfe_sim::ProgressSample) {
        self.ticks += 1;
        self.progress_m = sample.unwrapped_s_m.max(self.progress_m).max(0.0);
        self.completed_laps = sample.lap;
        self.speed_sum += state.vx.max(0.0) as f64;
        self.max_speed_ms = self.max_speed_ms.max(state.vx.max(0.0));
        self.lateral2 += (sample.lateral_error_m as f64).powi(2);
        self.heading2 += (sample.heading_error_rad as f64).powi(2);

        if let Some(last) = self.last_steering_rad {
            let rate = (step.command.steering_rad - last) / self.objective.dt_s;
            self.steering_rate2 += (rate as f64).powi(2);
        }
        if let Some(last) = self.last_throttle {
            let rate = (step.command.throttle - last) / self.objective.dt_s;
            self.throttle_rate2 += (rate as f64).powi(2);
        }
        self.last_steering_rad = Some(step.command.steering_rad);
        self.last_throttle = Some(step.command.throttle);

        if matches!(step.command.status, ControllerStatus::Unavailable) {
            self.unavailable += 1;
        }
    }

    fn finish(self) -> SimEpisodeScore {
        if self.ticks == 0 {
            return SimEpisodeScore {
                cost: 1.0e9,
                crashed: self.crashed,
                ..Default::default()
            };
        }

        let n = self.ticks as f64;
        let lateral_rms_m = (self.lateral2 / n).sqrt() as f32;
        let heading_rms_rad = (self.heading2 / n).sqrt() as f32;
        let rate_n = (self.ticks.saturating_sub(1)).max(1) as f64;
        let steering_rate_rms = (self.steering_rate2 / rate_n).sqrt() as f32;
        let throttle_rate_rms = (self.throttle_rate2 / rate_n).sqrt() as f32;
        let avg_speed_ms = (self.speed_sum / n) as f32;
        let unavailable_fraction = self.unavailable as f32 / self.ticks as f32;
        let progress_ratio = (self.progress_m / self.required_progress_m).clamp(0.0, 1.0);

        let cost = if let Some(finish_time_s) = self.finish_time_s {
            let reference_time_s =
                self.required_progress_m / self.objective.target_speed_ms.max(0.1);
            finish_time_s as f64 / reference_time_s.max(0.1) as f64
                + 2.0 * (lateral_rms_m as f64).powi(2)
                + 0.5 * (heading_rms_rad as f64).powi(2)
                + 0.01 * (steering_rate_rms as f64).powi(2)
                + 0.005 * (throttle_rate_rms as f64).powi(2)
                + 10.0 * unavailable_fraction as f64
        } else {
            1000.0
                + 500.0 * (1.0 - progress_ratio as f64).powi(2)
                + if self.crashed { 100.0 } else { 0.0 }
                + 50.0
                    * ((self.objective.min_avg_speed_ms - avg_speed_ms).max(0.0)
                        / self.objective.min_avg_speed_ms.max(0.1)) as f64
                + 10.0 * unavailable_fraction as f64
        };

        SimEpisodeScore {
            cost,
            completed_laps: self.completed_laps,
            progress_m: self.progress_m,
            progress_ratio,
            finish_time_s: self.finish_time_s,
            avg_speed_ms,
            max_speed_ms: self.max_speed_ms,
            crashed: self.crashed,
            lateral_rms_m,
            heading_rms_rad,
            steering_rate_rms,
            throttle_rate_rms,
            unavailable_fraction,
            ticks: self.ticks,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn robust_aggregation_penalizes_variance() {
        let a = SimEpisodeScore {
            cost: 1.0,
            ..Default::default()
        };
        let b = SimEpisodeScore {
            cost: 3.0,
            ..Default::default()
        };
        let score = aggregate_sim_scores(&[a, b], 1.0);
        assert!(score.cost > 2.0);
    }

    #[test]
    fn completed_episode_scores_below_failed_episode_band() {
        let objective = SimTuningObjective {
            target_laps: 1,
            target_speed_ms: 2.0,
            min_avg_speed_ms: 0.5,
            max_ticks: 100,
            dt_s: 0.1,
        };
        let mut completed = SimScoreAccumulator::new(objective, 10.0);
        completed.finish_time_s = Some(5.0);
        completed.progress_m = 10.0;
        completed.ticks = 50;
        let completed = completed.finish();

        let mut parked = SimScoreAccumulator::new(objective, 10.0);
        parked.ticks = 50;
        let parked = parked.finish();

        assert!(completed.cost < parked.cost);
        assert!(parked.cost >= 1000.0);
    }
}
