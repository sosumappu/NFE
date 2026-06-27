use std::{
    path::PathBuf,
    sync::{Arc, Mutex},
    time::Duration,
};

use anyhow::{Context, Result};
use nfe_core::io::SensorSource as CoreSensorSource;
use nfe_core::params::ParamSpec;
use nfe_runtime::{
    config::RuntimeConfig,
    input_replay::McapSensorReplaySource,
    pipeline::EstimatorMode,
    tuning::{config_from_vector, evaluate_episode_with_limit, search_space},
};
use nfe_sim::{
    DynamicBicycle, IdentifiedModel, KinematicBicycle, SimulatorSource, VehicleModel, World,
};

use nfe_car::{
    config::SimConfig,
    init::{ReadySignal, Sensor},
    replay::live_source::LiveSensorSource,
    sensors::factory::{SensorFactory, SensorReadySignals},
    state::{SensorStateWriter, SharedState},
    tuning::{aggregate_sim_scores, evaluate_sim_laps, SimEpisodeScore, SimTuningObjective},
};

struct Args {
    sim: Option<PathBuf>,
    replay: Option<PathBuf>,
    live: bool,
    out: PathBuf,
    config: Option<PathBuf>,
    max_gen: usize,
    episode_s: f32,
    model: String,
    model_params: Option<PathBuf>,
    sim_seed: Option<u64>,
    sigma: f64,
    parallel: bool,
    progress: bool,
    target_laps: u32,
    target_speed_ms: f32,
    min_avg_speed_ms: f32,
    eval_seeds: usize,
    robustness_weight: f64,
}

impl Args {
    fn parse() -> Self {
        let args: Vec<String> = std::env::args().collect();
        let get = |flag: &str| -> Option<String> {
            args.windows(2).find(|w| w[0] == flag).map(|w| w[1].clone())
        };
        let has = |flag: &str| args.iter().any(|a| a == flag);
        Self {
            sim: get("--sim").or_else(|| get("--world")).map(Into::into),
            replay: get("--replay").map(Into::into),
            live: has("--live"),
            out: get("--out")
                .unwrap_or_else(|| "best_runtime_config.json".into())
                .into(),
            config: get("--config").map(Into::into),
            max_gen: get("--generations")
                .and_then(|v| v.parse().ok())
                .unwrap_or(200),
            episode_s: get("--episode-s")
                .and_then(|v| v.parse().ok())
                .unwrap_or(30.0),
            model: get("--model").unwrap_or_else(|| "kinematic".into()),
            model_params: get("--model-params").map(Into::into),
            sim_seed: get("--sim-seed").and_then(|v| v.parse().ok()),
            sigma: get("--sigma").and_then(|v| v.parse().ok()).unwrap_or(0.3),
            parallel: has("--parallel"),
            progress: has("--progress"),
            target_laps: get("--target-laps")
                .and_then(|v| v.parse().ok())
                .unwrap_or(3),
            target_speed_ms: get("--target-speed")
                .and_then(|v| v.parse().ok())
                .unwrap_or(3.0),
            min_avg_speed_ms: get("--min-avg-speed")
                .and_then(|v| v.parse().ok())
                .unwrap_or(1.0),
            eval_seeds: get("--eval-seeds")
                .and_then(|v| v.parse().ok())
                .unwrap_or(1),
            robustness_weight: get("--robustness-weight")
                .and_then(|v| v.parse().ok())
                .unwrap_or(0.0),
        }
    }
}

#[derive(Clone)]
enum TuneMode {
    Sim {
        world: World,
        model: String,
        model_params: Option<PathBuf>,
        seed: Option<u64>,
        dt_s: f32,
        sim_config: SimConfig,
    },
    Replay {
        path: PathBuf,
    },
    Live {
        state: Arc<SharedState>,
        period: Duration,
        handles: Arc<Vec<std::thread::JoinHandle<()>>>,
    },
}

fn build_vehicle(
    model: &str,
    model_params: Option<&PathBuf>,
    sim_config: &SimConfig,
) -> Result<Box<dyn VehicleModel>> {
    match model {
        "dynamic" => Ok(Box::new(DynamicBicycle::from_params(sim_config.dynamic))),
        "identified" => {
            let path = model_params.context("--model-params required for identified model")?;
            Ok(Box::new(IdentifiedModel::from_json_with_base(
                path,
                sim_config.dynamic,
            )?))
        }
        _ => Ok(Box::new(KinematicBicycle::from_params(
            sim_config.kinematic,
        ))),
    }
}

impl TuneMode {
    fn supports_parallel(&self) -> bool {
        matches!(self, TuneMode::Sim { .. } | TuneMode::Replay { .. })
    }

    fn source(&self) -> Result<Box<dyn CoreSensorSource>> {
        match self {
            TuneMode::Sim {
                world,
                model,
                model_params,
                seed,
                dt_s,
                sim_config,
            } => {
                let vehicle = build_vehicle(model, model_params.as_ref(), sim_config)?;
                let (source, _actuator) = if let Some(seed) = seed {
                    SimulatorSource::new_with_seed_latency_and_footprint(
                        world.clone(),
                        vehicle,
                        *dt_s,
                        *seed,
                        sim_config.latency,
                        sim_config.footprint,
                    )
                } else {
                    SimulatorSource::new_with_latency_and_footprint(
                        world.clone(),
                        vehicle,
                        *dt_s,
                        sim_config.latency,
                        sim_config.footprint,
                    )
                };
                Ok(Box::new(source))
            }
            TuneMode::Replay { path } => Ok(Box::new(McapSensorReplaySource::open(path)?)),
            TuneMode::Live { state, period, .. } => Ok(Box::new(LiveCoreSource {
                source: LiveSensorSource::new(state.clone()),
                period: *period,
            })),
        }
    }
}

impl Drop for TuneMode {
    fn drop(&mut self) {
        if let TuneMode::Live { state, handles, .. } = self {
            state.set_shutdown();
            if let Some(handles) = Arc::get_mut(handles) {
                for handle in handles.drain(..) {
                    let _ = handle.join();
                }
            }
        }
    }
}

struct LiveCoreSource {
    source: LiveSensorSource,
    period: Duration,
}

impl CoreSensorSource for LiveCoreSource {
    fn next_snapshot(&mut self) -> Result<Option<nfe_core::sensors::SensorSnapshot>> {
        std::thread::sleep(self.period);
        Ok(Some(to_core_snapshot(
            nfe_car::hal::SensorSource::next_snapshot(&mut self.source)?,
        )))
    }
}

fn build_mode(args: &Args, runtime: &RuntimeConfig) -> Result<TuneMode> {
    if let Some(path) = &args.replay {
        return Ok(TuneMode::Replay { path: path.clone() });
    }

    if args.live {
        eprintln!(
            "car-tune: WARNING live mode is sensor-only/dry-run for CMA-ES; \
             repeated live candidate evaluation is non-deterministic and should be used only for diagnostics"
        );
        let car_config =
            nfe_car::config::Config::load(args.config.as_ref().and_then(|p| p.to_str()));
        let state = SharedState::new();
        let signals = SensorReadySignals {
            lidar: ReadySignal::dummy(Sensor::Lidar),
            imu: ReadySignal::dummy(Sensor::Imu),
            sonars: vec![
                ReadySignal::dummy(Sensor::Sonar(0)),
                ReadySignal::dummy(Sensor::Sonar(1)),
                ReadySignal::dummy(Sensor::Sonar(2)),
            ],
        };
        let state_writer: Arc<dyn SensorStateWriter> = state.clone();
        let spawned = SensorFactory::spawn_all(&state_writer, car_config.live.lidar_port, signals);
        return Ok(TuneMode::Live {
            state,
            period: Duration::from_secs_f32(runtime.dt_s()),
            handles: Arc::new(spawned.handles),
        });
    }

    let world_path = args
        .sim
        .clone()
        .unwrap_or_else(|| PathBuf::from("track.json"));
    let world = World::load(&world_path)
        .with_context(|| format!("cannot load sim world: {}", world_path.display()))?;
    let sim_config = args
        .config
        .as_ref()
        .and_then(|p| nfe_car::config::Config::from_toml_path(p).ok())
        .map(|cfg| cfg.sim)
        .unwrap_or_default();
    Ok(TuneMode::Sim {
        world,
        model: args.model.clone(),
        model_params: args.model_params.clone(),
        seed: args.sim_seed,
        dt_s: runtime.dt_s(),
        sim_config,
    })
}

fn to_core_snapshot(snapshot: nfe_car::state::SensorSnapshot) -> nfe_core::sensors::SensorSnapshot {
    nfe_core::sensors::SensorSnapshot {
        lidar: nfe_core::sensors::LidarCloud {
            timestamp_us: snapshot.lidar.timestamp_us,
            points: snapshot
                .lidar
                .points
                .iter()
                .map(|p| nfe_core::sensors::LidarPoint {
                    x: p.x,
                    y: p.y,
                    dist_m: p.dist_m,
                    angle_rad: p.angle_rad,
                    timestamp_us: p.timestamp_us,
                })
                .collect(),
        },
        imu: nfe_core::estimation::ImuSample {
            ax: snapshot.imu.ax,
            ay: snapshot.imu.ay,
            az: snapshot.imu.az,
            gx: snapshot.imu.gx,
            gy: snapshot.imu.gy,
            gz: snapshot.imu.gz,
            timestamp_us: snapshot.imu.timestamp_us,
        },
        sonar_m: snapshot.sonar_m,
        sensor_fault: snapshot.sensor_fault,
        start_line_crossed: false,
    }
}

fn load_runtime_config(args: &Args) -> RuntimeConfig {
    let mut cfg = args
        .config
        .as_ref()
        .and_then(|p| RuntimeConfig::from_toml_path(p).ok())
        .unwrap_or_default();
    cfg.mapping.enabled = false;
    cfg
}

fn vector_defaults(space: &[(String, ParamSpec)]) -> Vec<f64> {
    space.iter().map(|(_, spec)| spec.default_value()).collect()
}

fn print_progress<F>(
    cma: &cmaes::CMAES<F>,
    max_gen: usize,
    elapsed: Duration,
    sim_best: Option<SimEpisodeScore>,
) {
    let current = cma
        .current_best_individual()
        .map(|best| format!("{:.6}", best.value))
        .unwrap_or_else(|| "n/a".to_string());
    let overall = cma
        .overall_best_individual()
        .map(|best| format!("{:.6}", best.value))
        .unwrap_or_else(|| "n/a".to_string());
    if let Some(score) = sim_best {
        let finish = score
            .finish_time_s
            .map(|t| format!("{t:.2}s"))
            .unwrap_or_else(|| "n/a".to_string());
        eprintln!(
            "car-tune: gen={}/{} evals={} current={} best={} sigma={:.5} elapsed={:.1}s laps={} progress={:.1}% finish={} avg={:.2}m/s crash={} lat={:.3}m head={:.3}rad",
            cma.generation(),
            max_gen,
            cma.function_evals(),
            current,
            overall,
            cma.sigma(),
            elapsed.as_secs_f32(),
            score.completed_laps,
            100.0 * score.progress_ratio,
            finish,
            score.avg_speed_ms,
            score.crashed,
            score.lateral_rms_m,
            score.heading_rms_rad,
        );
    } else {
        eprintln!(
            "car-tune: gen={}/{} evals={} current={} best={} sigma={:.5} elapsed={:.1}s",
            cma.generation(),
            max_gen,
            cma.function_evals(),
            current,
            overall,
            cma.sigma(),
            elapsed.as_secs_f32()
        );
    }
}

fn run_cma<F>(
    cma: &mut cmaes::CMAES<F>,
    max_gen: usize,
    parallel: bool,
    progress: bool,
    sim_best: Option<Arc<Mutex<Option<SimEpisodeScore>>>>,
) -> cmaes::TerminationData
where
    F: cmaes::ObjectiveFunction + cmaes::ParallelObjectiveFunction,
{
    let started = std::time::Instant::now();
    loop {
        let result = if parallel {
            cma.next_parallel()
        } else {
            cma.next()
        };
        if progress {
            let sim_best = sim_best.as_ref().and_then(|best| *best.lock().unwrap());
            print_progress(cma, max_gen, started.elapsed(), sim_best);
        }
        if let Some(result) = result {
            return result;
        }
    }
}

fn sim_eval_seeds(base_seed: Option<u64>, count: usize) -> Vec<u64> {
    let base = base_seed.unwrap_or(0xC0FF_EE00);
    (0..count.max(1))
        .map(|i| base.wrapping_add(i as u64))
        .collect()
}

fn evaluate_sim_candidate(
    cfg: RuntimeConfig,
    world: &World,
    model: &str,
    model_params: Option<&PathBuf>,
    sim_config: &SimConfig,
    seeds: &[u64],
    objective: &SimTuningObjective,
    robustness_weight: f64,
) -> Result<SimEpisodeScore> {
    let mut scores = Vec::with_capacity(seeds.len());
    for seed in seeds {
        scores.push(evaluate_sim_laps(
            cfg.clone(),
            world.clone(),
            build_vehicle(model, model_params, sim_config)?,
            Some(*seed),
            sim_config.latency,
            sim_config.footprint,
            objective,
        )?);
    }
    Ok(aggregate_sim_scores(&scores, robustness_weight))
}

fn audit_search_space(space: &[(String, ParamSpec)]) {
    let integers = space
        .iter()
        .filter(|(_, spec)| matches!(spec, ParamSpec::Integer { .. }))
        .count();
    let log_scaled = space
        .iter()
        .filter(|(_, spec)| matches!(spec, ParamSpec::Continuous { log: true, .. }))
        .count();
    eprintln!("car-tune: search space from Tunable registry: {} params ({integers} integer, {log_scaled} log-scale)", space.len());
    if integers > 0 {
        eprintln!(
            "car-tune: CMA-ES audit: integer params are currently optimized as continuous values then rounded/clamped by Tunable; covariance/adaptation still treats them as continuous. Mixed discrete+continuous handling needs a separate design."
        );
    }
    if log_scaled > 0 {
        eprintln!(
            "car-tune: CMA-ES audit: log-scale ParamSpec metadata is discovered but not transformed into log-space sampling yet; current objective clamps in linear space."
        );
    }
}

fn main() -> Result<()> {
    let args = Args::parse();
    let base_cfg = load_runtime_config(&args);
    let max_ticks = (args.episode_s / base_cfg.dt_s()).max(1.0) as usize;
    let mode = build_mode(&args, &base_cfg)?;
    if let TuneMode::Sim { world, .. } = &mode {
        if world.waypoints.len() < 2 {
            anyhow::bail!("car-tune sim lap tuning requires at least two world.waypoints entries");
        }
    }
    let space = search_space();
    audit_search_space(&space);
    let x0 = vector_defaults(&space);

    let parallel = if args.parallel && mode.supports_parallel() {
        true
    } else {
        if args.parallel {
            eprintln!("car-tune: --parallel ignored for live mode");
        }
        false
    };

    println!(
        "car-tune: CMA-ES starting sigma={} dim={} max_gen={} ticks={} parallel={} progress={} target_laps={} target_speed={:.2} eval_seeds={}",
        args.sigma,
        x0.len(),
        args.max_gen,
        max_ticks,
        parallel,
        args.progress,
        args.target_laps,
        args.target_speed_ms,
        args.eval_seeds.max(1)
    );

    let objective_mode = mode.clone();
    let base_for_objective = base_cfg.clone();
    let sim_objective = SimTuningObjective {
        target_laps: args.target_laps.max(1),
        target_speed_ms: args.target_speed_ms.max(0.1),
        min_avg_speed_ms: args.min_avg_speed_ms.max(0.1),
        max_ticks,
        dt_s: base_cfg.dt_s(),
    };
    let sim_best: Option<Arc<Mutex<Option<SimEpisodeScore>>>> =
        matches!(mode, TuneMode::Sim { .. }).then(|| Arc::new(Mutex::new(None)));
    let objective_sim_best = sim_best.clone();
    let eval_seeds = args.eval_seeds.max(1);
    let robustness_weight = args.robustness_weight;
    let objective = move |x: &cmaes::DVector<f64>| -> f64 {
        let cfg = config_from_vector(&base_for_objective, x.as_slice());
        match &objective_mode {
            TuneMode::Sim {
                world,
                model,
                model_params,
                seed,
                sim_config,
                ..
            } => {
                let seeds = sim_eval_seeds(*seed, eval_seeds);
                match evaluate_sim_candidate(
                    cfg,
                    world,
                    model,
                    model_params.as_ref(),
                    sim_config,
                    &seeds,
                    &sim_objective,
                    robustness_weight,
                ) {
                    Ok(score) => {
                        if let Some(best) = &objective_sim_best {
                            let mut best = best.lock().unwrap();
                            if best.map(|b| score.cost < b.cost).unwrap_or(true) {
                                *best = Some(score);
                            }
                        }
                        score.cost
                    }
                    Err(e) => 1.0e9 + format!("{e:#}").len() as f64,
                }
            }
            TuneMode::Replay { .. } | TuneMode::Live { .. } => {
                let mut source = match objective_mode.source() {
                    Ok(source) => source,
                    Err(e) => return 1.0e9 + format!("{e:#}").len() as f64,
                };
                match evaluate_episode_with_limit(
                    cfg,
                    EstimatorMode::DeadReckon,
                    source.as_mut(),
                    Some(max_ticks),
                ) {
                    Ok(cost) if cost.ticks > 0 => cost.cost as f64,
                    Ok(_) => 1.0e9,
                    Err(e) => 1.0e9 + format!("{e:#}").len() as f64,
                }
            }
        }
    };

    use cmaes::CMAESOptions;
    let mut cma = CMAESOptions::new(x0, args.sigma)
        .max_generations(args.max_gen)
        .build(objective)
        .map_err(|e| anyhow::anyhow!("CMA-ES build error: {:?}", e))?;
    let result = run_cma(&mut cma, args.max_gen, parallel, args.progress, sim_best);
    let best = result
        .overall_best
        .expect("CMA-ES failed to find any valid solution");
    let tuned = config_from_vector(&base_cfg, best.point.as_slice());
    let json = serde_json::to_string_pretty(&tuned)?;
    std::fs::write(&args.out, json)?;
    println!("car-tune: converged cost={:.4}", best.value);
    println!(
        "car-tune: tuned RuntimeConfig written to {}",
        args.out.display()
    );
    Ok(())
}
