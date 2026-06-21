use std::{path::PathBuf, sync::Arc, time::Duration};

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
    init::{ReadySignal, Sensor},
    replay::live_source::LiveSensorSource,
    sensors::factory::{SensorFactory, SensorReadySignals},
    state::{SensorStateWriter, SharedState},
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

impl TuneMode {
    fn source(&self) -> Result<Box<dyn CoreSensorSource>> {
        match self {
            TuneMode::Sim {
                world,
                model,
                model_params,
                seed,
                dt_s,
            } => {
                let vehicle: Box<dyn VehicleModel> = match model.as_str() {
                    "dynamic" => Box::new(DynamicBicycle::default()),
                    "identified" => {
                        let path = model_params
                            .as_ref()
                            .context("--model-params required for identified model")?;
                        Box::new(IdentifiedModel::from_json(path)?)
                    }
                    _ => Box::new(KinematicBicycle::default()),
                };
                let (source, _actuator) = if let Some(seed) = seed {
                    SimulatorSource::new_with_seed(world.clone(), vehicle, *dt_s, *seed)
                } else {
                    SimulatorSource::new(world.clone(), vehicle, *dt_s)
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
    Ok(TuneMode::Sim {
        world,
        model: args.model.clone(),
        model_params: args.model_params.clone(),
        seed: args.sim_seed,
        dt_s: runtime.dt_s(),
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
    let space = search_space();
    audit_search_space(&space);
    let x0 = vector_defaults(&space);

    println!(
        "car-tune: CMA-ES starting sigma={} dim={} max_gen={} ticks={}",
        args.sigma,
        x0.len(),
        args.max_gen,
        max_ticks
    );

    let objective_mode = mode.clone();
    let base_for_objective = base_cfg.clone();
    let objective = move |x: &cmaes::DVector<f64>| -> f64 {
        let cfg = config_from_vector(&base_for_objective, x.as_slice());
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
    };

    use cmaes::CMAESOptions;
    let mut cma = CMAESOptions::new(x0, args.sigma)
        .max_generations(args.max_gen)
        .build(objective)
        .map_err(|e| anyhow::anyhow!("CMA-ES build error: {:?}", e))?;
    let result = cma.run();
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
