use anyhow::Result;
/// bin/tune.rs — CMA-ES parameter tuner (pure Rust, no Python required)
///
/// Uses the `cmaes` crate (add to Cargo.toml — see comment at bottom).
///
/// Workflow
/// ────────
///   1. Build a track JSON with your python tool
///   2. Run:  cargo run --release --bin car-tune -- --world track.json
///   3. After convergence, best_params.json is written — use with --params
///
/// What gets tuned
/// ───────────────
///   PID:          kp, ki, kd
///   LQR:          k_lat, k_lat_rate, k_heading, k_yaw
///   SpeedPlanner: v_max, k_dist, k_heading_speed
///
/// Total: 10 parameters.  CMA-ES handles them jointly with full covariance.
///
/// Objective
/// ─────────
/// For each candidate θ: construct a `SimulatorSource` (KinematicBicycle),
/// run the control loop synchronously in fast mode, return `RunCost::cost`.
/// One evaluation ≈ 1–5 ms depending on world complexity and episode length.
/// At λ=12 (default population), one generation ≈ 60 ms on a laptop.
use std::path::PathBuf;

// ── Tunable parameter vector ───────────────────────────────────────────────

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct TuningParams {
    // PID (longitudinal)
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    // LQR gains (lateral)
    pub k_lat: f32,
    pub k_lat_rate: f32,
    pub k_heading: f32,
    pub k_yaw: f32,
    // SpeedPlanner
    pub v_max: f32,
    pub k_dist: f32,
    pub k_heading_speed: f32,
}

impl TuningParams {
    /// Reasonable starting point — same as current hardcoded values
    pub fn default_start() -> Self {
        Self {
            kp: 1.5,
            ki: 0.05,
            kd: 0.2,
            k_lat: 0.80,
            k_lat_rate: 0.30,
            k_heading: 1.20,
            k_yaw: 0.40,
            v_max: 1.0,
            k_dist: 1.0,
            k_heading_speed: 1.0,
        }
    }

    pub fn to_vec(&self) -> Vec<f64> {
        vec![
            self.kp as f64,
            self.ki as f64,
            self.kd as f64,
            self.k_lat as f64,
            self.k_lat_rate as f64,
            self.k_heading as f64,
            self.k_yaw as f64,
            self.v_max as f64,
            self.k_dist as f64,
            self.k_heading_speed as f64,
        ]
    }

    pub fn from_slice(v: &[f64]) -> Self {
        Self {
            kp: v[0] as f32,
            ki: v[1] as f32,
            kd: v[2] as f32,
            k_lat: v[3] as f32,
            k_lat_rate: v[4] as f32,
            k_heading: v[5] as f32,
            k_yaw: v[6] as f32,
            v_max: v[7] as f32,
            k_dist: v[8] as f32,
            k_heading_speed: v[9] as f32,
        }
    }

    pub fn clamp(&self) -> Self {
        // Prevent degenerate values — CMA-ES can wander into negatives
        Self {
            kp: self.kp.max(0.0),
            ki: self.ki.max(0.0),
            kd: self.kd.max(0.0),
            k_lat: self.k_lat.max(0.0),
            k_lat_rate: self.k_lat_rate.max(0.0),
            k_heading: self.k_heading.max(0.0),
            k_yaw: self.k_yaw.max(0.0),
            v_max: self.v_max.clamp(0.1, 10.0),
            k_dist: self.k_dist.max(0.0),
            k_heading_speed: self.k_heading_speed.max(0.0),
        }
    }
}

// ── CLI args ───────────────────────────────────────────────────────────────

struct Args {
    world: PathBuf,
    out: PathBuf,
    max_gen: usize,
    episode_s: f32,
    model: String,
    model_params: Option<PathBuf>,
}

impl Args {
    fn parse() -> Self {
        let args: Vec<String> = std::env::args().collect();
        let get = |flag: &str| -> Option<String> {
            args.windows(2).find(|w| w[0] == flag).map(|w| w[1].clone())
        };
        Self {
            world: get("--world").unwrap_or_else(|| "track.json".into()).into(),
            out: get("--out")
                .unwrap_or_else(|| "best_params.json".into())
                .into(),
            max_gen: get("--generations")
                .and_then(|v| v.parse().ok())
                .unwrap_or(200),
            episode_s: get("--episode-s")
                .and_then(|v| v.parse().ok())
                .unwrap_or(30.0),
            model: get("--model").unwrap_or_else(|| "kinematic".into()),
            model_params: get("--model-params").map(Into::into),
        }
    }
}

// ── Episode runner ─────────────────────────────────────────────────────────

/// Run one simulated episode synchronously (no async, no tokio).
/// Returns the scalar cost from `MetricsLog::summarise()`.
fn run_episode(
    world: &car::sim::world::World,
    params: &TuningParams,
    dt: f32,
    max_ticks: usize,
    model_variant: &str,
    model_params_path: Option<&std::path::Path>,
) -> f32 {
    use car::control::{
        lqr::{Lqr, LqrState},
        mpt::Mpt,
        pid::Pid,
        speed::SpeedPlanner,
    };
    use car::fusion::kinematics::Kinematics;
    use car::hal::{ActuatorSink, SensorSource};
    use car::metrics::{MetricsLog, TickMetrics};
    use car::sim::{
        model::{DynamicBicycle, IdentifiedModel, KinematicBicycle},
        source::SimulatorSource,
    };
    use car::types::{ImuBias, LidarPoint};

    let model: Box<dyn car::sim::model::VehicleModel> = match model_variant {
        "dynamic" => Box::new(DynamicBicycle::default()),
        "identified" => {
            let path = model_params_path.expect("--model-params required for identified model");
            Box::new(IdentifiedModel::from_json(path).expect("load model params"))
        }
        _ => Box::new(KinematicBicycle::default()),
    };

    let (mut source, mut actuator) = SimulatorSource::new(world.clone(), model, dt);

    // Build controllers with candidate params
    let mut pid = Pid::new_with_dt(params.kp, params.ki, params.kd, dt);
    let lqr = Lqr::new_with_gains([
        params.k_lat,
        params.k_lat_rate,
        params.k_heading,
        params.k_yaw,
    ]);
    let mpt = Mpt::new();
    let speed = SpeedPlanner::new(params.v_max, params.k_dist, params.k_heading_speed);
    let bias = ImuBias {
        bax: 0.0,
        bay: 0.0,
        baz: 0.0,
        bgx: 0.0,
        bgy: 0.0,
        bgz: 0.0,
    };
    let mut kin = Kinematics::new(500, bias);
    let mut deskew_buf: Vec<LidarPoint> = Vec::with_capacity(256);
    let mut log = MetricsLog::new();

    for tick in 0..max_ticks {
        if source.is_exhausted() {
            break;
        }

        let snap = match source.next_snapshot() {
            Ok(s) => s,
            Err(_) => break,
        };

        let estop = snap.obstacle_closer_than(0.10); // tighter in sim
        if estop {
            actuator.safe_state().ok();
            log.push(TickMetrics {
                tick: tick as u64,
                estop: true,
                ..Default::default()
            });
            // Large penalty — crash / near-crash
            break;
        }

        let handle = kin.update(&snap.imu);
        let cloud = kin.deskew(&snap.lidar, &mut deskew_buf);
        let (err_dist, heading_err) = mpt.compute(&cloud);
        let lateral_error_m = err_dist * heading_err.sin();
        kin.record_lateral_error(handle, lateral_error_m);

        let lqr_state = LqrState {
            lateral_error_m,
            lateral_rate_m_s: kin.lateral_rate(),
            heading_error_rad: heading_err,
            yaw_rate_rad_s: kin.current_yaw_rate(),
        };
        let steering = lqr.compute_lateral(&lqr_state);
        let v_target = speed.compute(&cloud, err_dist, heading_err);
        let v_current = kin.current_speed();
        let throttle = pid.compute_longitudinal(v_target - v_current);

        actuator.set_steering(steering).ok();
        actuator.set_throttle(throttle).ok();

        log.push(TickMetrics {
            tick: tick as u64,
            lateral_error_m,
            heading_error_rad: heading_err,
            steering_rad: steering,
            throttle,
            target_speed_ms: v_target,
            current_speed_ms: v_current,
            gz_rad_s: snap.imu.gz,
            vy_ms: kin.lateral_rate() * dt,
            ..Default::default()
        });
    }

    log.summarise().cost
}

// ── Main ───────────────────────────────────────────────────────────────────

fn main() -> Result<()> {
    let args = Args::parse();

    println!("car-tune: loading world from {}", args.world.display());
    let world = car::sim::world::World::load(&args.world)?;
    println!(
        "car-tune: {} wall segments, {} waypoints",
        world.walls.len(),
        world.waypoints.len()
    );

    let start = TuningParams::default_start();
    let x0 = start.to_vec();
    let sigma0 = 0.3f64;
    let dt = 1.0 / 100.0f32;
    let max_ticks = (args.episode_s / dt) as usize;

    println!(
        "car-tune: CMA-ES starting  sigma0={sigma0}  dim={}  max_gen={}",
        x0.len(),
        args.max_gen
    );

    // ── CMA-ES loop ─────────────────────────────────────────────────────
    // The closure captures world, args by reference.
    let objective = |x: &[f64]| -> f64 {
        let p = TuningParams::from_slice(x).clamp();
        run_episode(
            &world,
            &p,
            dt,
            max_ticks,
            &args.model,
            args.model_params.as_deref(),
        ) as f64
    };

    use cmaes::{CMAESOptions, ObjectiveFunction};
    let mut cma = CMAESOptions::new(x0, sigma0)
        .max_generations(args.max_gen)
        .build(objective)?;
    let result = cma.run();
    let best = TuningParams::from_slice(&result.best_individual.point).clamp();
    println!(
        "car-tune: converged  cost={:.4}",
        result.best_individual.value
    );
    println!("{best:#?}");
    let json = serde_json::to_string_pretty(&best)?;
    std::fs::write(&args.out, &json)?;
    println!("car-tune: best params written to {}", args.out.display());

    // Placeholder: single evaluation to verify the pipeline compiles
    println!("car-tune: running single evaluation to validate pipeline...");
    let cost = objective(&x0);
    println!("car-tune: initial cost = {cost:.4}");
    println!();

    Ok(())
}
