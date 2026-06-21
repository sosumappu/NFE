//! Tuning helpers built on the real pipeline path.
//!
//! Optimizers should use `search_space()` and `config_from_vector()`; episode
//! evaluation uses `Pipeline::step` through `run_sync`, not a copied control
//! loop.

use nfe_algo::config::AlgoConfig;
use nfe_core::io::{ActuatorSink, SensorSource};
use nfe_core::params::{ParamSpec, TunableExt};

use crate::config::RuntimeConfig;
use crate::pipeline::{EstimatorMode, Pipeline};
use crate::run_loop::run_sync;

pub fn search_space() -> Vec<(String, ParamSpec)> {
    AlgoConfig::search_space("algo")
}

pub fn config_from_vector(base: &RuntimeConfig, vector: &[f64]) -> RuntimeConfig {
    let mut cfg = base.clone();
    cfg.algo = AlgoConfig::from_vec("algo", vector);
    cfg
}

#[derive(Clone, Copy, Debug, Default)]
pub struct EpisodeCost {
    pub cost: f32,
    pub steering_rms: f32,
    pub throttle_rms: f32,
    pub n_unavailable: u32,
    pub ticks: u64,
}

pub fn evaluate_episode(
    cfg: RuntimeConfig,
    estimator_mode: EstimatorMode,
    source: &mut dyn SensorSource,
) -> anyhow::Result<EpisodeCost> {
    evaluate_episode_with_limit(cfg, estimator_mode, source, None)
}

pub fn evaluate_episode_with_limit(
    cfg: RuntimeConfig,
    estimator_mode: EstimatorMode,
    source: &mut dyn SensorSource,
    max_ticks: Option<usize>,
) -> anyhow::Result<EpisodeCost> {
    let mut pipeline = Pipeline::new(cfg, estimator_mode);
    let mut actuator = NullActuator;
    let out = run_sync(&mut pipeline, source, &mut actuator, max_ticks)?;
    Ok(summarize(&out))
}

fn summarize(out: &[crate::pipeline::StepOutput]) -> EpisodeCost {
    if out.is_empty() {
        return EpisodeCost::default();
    }
    let mut steer2 = 0.0f64;
    let mut throttle2 = 0.0f64;
    let mut lateral2 = 0.0f64;
    let mut heading2 = 0.0f64;
    let mut n_unavailable = 0u32;
    for s in out {
        steer2 += (s.command.steering_rad as f64).powi(2);
        throttle2 += (s.command.throttle as f64).powi(2);
        lateral2 += (s.corridor.lateral_error_m as f64).powi(2);
        heading2 += (s.corridor.heading_error_rad as f64).powi(2);
        if matches!(
            s.command.status,
            nfe_core::control::ControllerStatus::Unavailable
        ) {
            n_unavailable += 1;
        }
    }
    let n = out.len() as f64;
    let steering_rms = (steer2 / n).sqrt() as f32;
    let throttle_rms = (throttle2 / n).sqrt() as f32;
    let lateral_rms = (lateral2 / n).sqrt() as f32;
    let heading_rms = (heading2 / n).sqrt() as f32;
    EpisodeCost {
        cost: lateral_rms
            + 0.5 * heading_rms
            + 0.2 * steering_rms
            + 0.1 * throttle_rms
            + 5.0 * (n_unavailable as f32 / out.len() as f32),
        steering_rms,
        throttle_rms,
        n_unavailable,
        ticks: out.len() as u64,
    }
}

struct NullActuator;
impl ActuatorSink for NullActuator {
    fn apply(&mut self, _output: &nfe_core::control::ControlOutput) -> anyhow::Result<()> {
        Ok(())
    }
    fn safe_state(&mut self) -> anyhow::Result<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exposes_non_empty_search_space() {
        let ss = search_space();
        assert!(ss.iter().any(|(k, _)| k == "algo.ekf.q_accel"));
        assert!(ss.iter().any(|(k, _)| k == "algo.reactive.lqr.k_lateral"));
    }
}
