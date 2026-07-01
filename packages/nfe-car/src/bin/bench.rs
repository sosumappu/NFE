use std::io::{BufWriter, Write};
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, Context, Result};
use nfe_car::config::Config;
use nfe_car::control::actuate::ActuatorFactory;
use nfe_car::hal::ActuatorSink;
use nfe_car::init::{ReadySignal, Sensor};
use nfe_car::sensors::factory::{SensorFactory, SensorReadySignals};
use nfe_car::state::{SensorSnapshot, SensorStateWriter, SharedState};
use nfe_car::types::ImuBias;
use nfe_core::control::ControlOutput;
use nfe_sim::{ControlCommand, DynamicBicycle, KinematicBicycle, VehicleModel, VehicleState};

const DEFAULT_HZ: u64 = 100;
const DEFAULT_DURATION_S: f32 = 4.0;
const DEFAULT_MAX_SPEED_MS: f32 = 1.0;
const DEFAULT_MAX_THROTTLE: f32 = 0.12;
const DEFAULT_MAX_STEERING_RAD: f32 = 0.15;
const DEFAULT_MAX_YAW_RATE_RAD_S: f32 = 2.0;
const DEFAULT_MAX_ABS_AX_MS2: f32 = 8.0;
const DEFAULT_OBSTACLE_MIN_M: f32 = 1.0;
const KAPPA_SPEED_FLOOR_MS: f32 = 0.3;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Source {
    Sim,
    Live,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Script {
    ThrottleStep,
    Coastdown,
    SteeringSweep,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Mode {
    source: Source,
    script: Script,
}

#[derive(Clone, Debug)]
struct Args {
    mode: Mode,
    config: Option<String>,
    csv: Option<String>,
    json: Option<String>,
    hz: u64,
    duration_s: f32,
    max_speed_ms: f32,
    max_throttle: f32,
    max_steering_rad: f32,
    max_yaw_rate_rad_s: f32,
    max_abs_ax_ms2: f32,
    obstacle_min_m: f32,
    understand_open_loop: bool,
    model: SimModel,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum SimModel {
    Dynamic,
    Kinematic,
}

#[derive(Clone, Copy, Debug, Default, serde::Serialize)]
struct BenchSummary {
    samples: u64,
    completed: bool,
    aborted: bool,
    max_speed_ms: f32,
    max_abs_yaw_rate_rad_s: f32,
    max_abs_ax_ms2: f32,
    final_speed_ms: f32,
    duration_s: f32,
}

#[derive(Clone, Copy, Debug)]
struct BenchSample {
    t_s: f32,
    speed_ms: f32,
    yaw_rate_rad_s: f32,
    ax_ms2: f32,
    ay_ms2: f32,
    front_obstacle_m: f32,
}

#[derive(Clone, Copy, Debug)]
struct CommandSample {
    phase: &'static str,
    steering_rad: f32,
    throttle: f32,
}

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
        .try_init()
        .ok();

    let args = Args::parse()?;
    validate_args(&args)?;
    let config = Config::load(args.config.as_deref());
    let abort = install_signal_flag();

    let summary = match args.mode.source {
        Source::Sim => run_sim(&args, &config, abort)?,
        Source::Live => run_live(&args, &config, abort)?,
    };

    if let Some(path) = &args.json {
        std::fs::write(path, serde_json::to_string_pretty(&summary)? + "\n")?;
    }
    if summary.aborted {
        bail!("bench run aborted before completion");
    }
    Ok(())
}

fn validate_args(args: &Args) -> Result<()> {
    if args.mode.source == Source::Live && !args.understand_open_loop {
        bail!("live bench modes require --i-understand-open-loop-driving");
    }
    if args.hz == 0 || args.hz > 500 {
        bail!("--hz must be in 1..=500");
    }
    if !args.duration_s.is_finite() || args.duration_s <= 0.0 || args.duration_s > 10.0 {
        bail!("--duration-s must be in (0, 10]");
    }
    if !args.max_speed_ms.is_finite() || args.max_speed_ms <= 0.0 || args.max_speed_ms > 5.0 {
        bail!("--max-speed-ms must be in (0, 5]");
    }
    if !args.max_throttle.is_finite() || args.max_throttle < 0.0 || args.max_throttle > 0.5 {
        bail!("--max-throttle must be in [0, 0.5]");
    }
    if !args.max_steering_rad.is_finite()
        || args.max_steering_rad < 0.0
        || args.max_steering_rad > 0.5
    {
        bail!("--max-steering-rad must be in [0, 0.5]");
    }
    Ok(())
}

fn run_sim(args: &Args, config: &Config, abort: Arc<AtomicBool>) -> Result<BenchSummary> {
    let mut writer = CsvWriter::new(args.csv.as_deref())?;
    let dt = 1.0 / args.hz as f32;
    let mut state = VehicleState::default();
    let mut model: Box<dyn VehicleModel> = match args.model {
        SimModel::Dynamic => Box::new(DynamicBicycle::from_params(config.sim.dynamic)),
        SimModel::Kinematic => Box::new(KinematicBicycle::from_params(config.sim.kinematic)),
    };
    let mut summary = BenchSummary::default();
    let steps = (args.duration_s * args.hz as f32).ceil() as u64;

    for step in 0..=steps {
        let t_s = step as f32 * dt;
        let sample = BenchSample {
            t_s,
            speed_ms: state.vx.max(0.0),
            yaw_rate_rad_s: state.yaw_rate,
            ax_ms2: state.ax,
            ay_ms2: state.ay,
            front_obstacle_m: f32::INFINITY,
        };
        let command = command_at(args.mode.script, args, t_s, sample.speed_ms);
        writer.write(args, &sample, &command, None)?;
        update_summary(&mut summary, &sample);
        if let Some(reason) = abort_reason(args, &sample, abort.load(Ordering::Relaxed)) {
            writer.write(args, &sample, &safe_command("abort"), Some(reason))?;
            summary.aborted = true;
            summary.duration_s = t_s;
            return Ok(summary);
        }
        state = model.step(
            &state,
            &ControlCommand {
                steering_rad: command.steering_rad,
                throttle: command.throttle,
            },
            dt,
        );
    }

    summary.completed = true;
    summary.duration_s = args.duration_s;
    Ok(summary)
}

fn run_live(args: &Args, config: &Config, abort: Arc<AtomicBool>) -> Result<BenchSummary> {
    let state = SharedState::new();
    let state_writer: Arc<dyn SensorStateWriter> = state.clone();
    let signals = SensorReadySignals {
        lidar: ReadySignal::dummy(Sensor::Lidar),
        imu: ReadySignal::dummy(Sensor::Imu),
        sonars: vec![
            ReadySignal::dummy(Sensor::Sonar(0)),
            ReadySignal::dummy(Sensor::Sonar(1)),
            ReadySignal::dummy(Sensor::Sonar(2)),
        ],
    };
    let spawned = SensorFactory::spawn_all(&state_writer, config.live.lidar_port.clone(), signals);
    let actuator = ActuatorFactory::build(10);
    let mut live = LiveBench {
        state,
        _state_writer: state_writer,
        spawned,
        actuator,
    };
    live.actuator.safe_state()?;
    thread::sleep(Duration::from_millis(500));
    let bias = calibrate_imu(&live.state, Duration::from_secs(1))?;
    let mut estimator = LiveEstimator::new(bias);
    let mut writer = CsvWriter::new(args.csv.as_deref())?;
    let mut summary = BenchSummary::default();
    let tick = Duration::from_secs_f32(1.0 / args.hz as f32);
    let started = Instant::now();
    let mut next_tick = started;

    loop {
        let now = Instant::now();
        if now < next_tick {
            thread::sleep(next_tick - now);
        }
        next_tick += tick;
        let t_s = started.elapsed().as_secs_f32();
        let snapshot = live.state.snapshot();
        let sample = estimator.update(snapshot);
        let command = if t_s <= args.duration_s {
            command_at(args.mode.script, args, t_s, sample.speed_ms)
        } else {
            safe_command("done")
        };
        live.actuator.set_steering(command.steering_rad)?;
        live.actuator.set_throttle(command.throttle)?;
        writer.write(args, &sample, &command, None)?;
        update_summary(&mut summary, &sample);

        if t_s > args.duration_s {
            live.actuator.safe_state()?;
            summary.completed = true;
            summary.duration_s = t_s;
            return Ok(summary);
        }
        if let Some(reason) = abort_reason(args, &sample, abort.load(Ordering::Relaxed)) {
            live.actuator.safe_state()?;
            writer.write(args, &sample, &safe_command("abort"), Some(reason))?;
            summary.aborted = true;
            summary.duration_s = t_s;
            return Ok(summary);
        }
    }
}

struct LiveBench {
    state: Arc<SharedState>,
    _state_writer: Arc<dyn SensorStateWriter>,
    spawned: nfe_car::sensors::factory::SpawnedSensors,
    actuator: Box<dyn ActuatorSink>,
}

impl Drop for LiveBench {
    fn drop(&mut self) {
        let _ = self.actuator.safe_state();
        self.state.set_shutdown();
        for handle in self.spawned.handles.drain(..) {
            let _ = handle.join();
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct LiveEstimator {
    bias: ImuBias,
    speed_ms: f32,
    last_timestamp_us: Option<u64>,
}

impl LiveEstimator {
    fn new(bias: ImuBias) -> Self {
        Self {
            bias,
            speed_ms: 0.0,
            last_timestamp_us: None,
        }
    }

    fn update(&mut self, snapshot: SensorSnapshot) -> BenchSample {
        let imu = snapshot.imu - self.bias;
        let dt = self
            .last_timestamp_us
            .map(|prev| snapshot.imu.timestamp_us.saturating_sub(prev) as f32 * 1.0e-6)
            .unwrap_or(0.0)
            .clamp(0.0, 0.05);
        self.last_timestamp_us = Some(snapshot.imu.timestamp_us);
        self.speed_ms = (self.speed_ms + imu.ax * dt).max(0.0);
        let front_obstacle_m = snapshot
            .lidar
            .nearest_in_arc(0.0, 20.0_f32.to_radians())
            .map_or(f32::INFINITY, |p| p.dist_m);
        BenchSample {
            t_s: snapshot.imu.timestamp_us as f32 * 1.0e-6,
            speed_ms: self.speed_ms,
            yaw_rate_rad_s: imu.gz,
            ax_ms2: imu.ax,
            ay_ms2: imu.ay,
            front_obstacle_m,
        }
    }
}

fn calibrate_imu(state: &SharedState, duration: Duration) -> Result<ImuBias> {
    let started = Instant::now();
    let mut samples = Vec::new();
    while started.elapsed() < duration {
        let sample = state.snapshot().imu;
        if sample.timestamp_us != 0 {
            samples.push(sample);
        }
        thread::sleep(Duration::from_millis(5));
    }
    if samples.len() < 20 {
        bail!("IMU did not produce enough samples for bench calibration");
    }
    Ok(ImuBias::estimate(&samples))
}

fn command_at(script: Script, args: &Args, t_s: f32, speed_ms: f32) -> CommandSample {
    match script {
        Script::ThrottleStep => {
            if t_s < 0.5 {
                safe_command("settle")
            } else if t_s < args.duration_s - 0.5 {
                CommandSample {
                    phase: "throttle_step",
                    steering_rad: 0.0,
                    throttle: args.max_throttle,
                }
            } else {
                safe_command("release")
            }
        }
        Script::Coastdown => {
            if t_s < 0.5 {
                safe_command("settle")
            } else if t_s < args.duration_s * 0.5 && speed_ms < args.max_speed_ms * 0.8 {
                CommandSample {
                    phase: "spin_up",
                    steering_rad: 0.0,
                    throttle: args.max_throttle,
                }
            } else {
                safe_command("coast")
            }
        }
        Script::SteeringSweep => {
            if t_s < 0.75 {
                CommandSample {
                    phase: "speed_settle",
                    steering_rad: 0.0,
                    throttle: 0.5 * args.max_throttle,
                }
            } else {
                let period = 1.0;
                let sign = if ((t_s - 0.75) / period).floor() as i32 % 2 == 0 {
                    1.0
                } else {
                    -1.0
                };
                CommandSample {
                    phase: if sign > 0.0 {
                        "steer_left"
                    } else {
                        "steer_right"
                    },
                    steering_rad: sign * args.max_steering_rad,
                    throttle: 0.5 * args.max_throttle,
                }
            }
        }
    }
}

fn safe_command(phase: &'static str) -> CommandSample {
    CommandSample {
        phase,
        steering_rad: 0.0,
        throttle: 0.0,
    }
}

fn abort_reason(args: &Args, sample: &BenchSample, signal_abort: bool) -> Option<&'static str> {
    if signal_abort {
        return Some("signal");
    }
    if sample.speed_ms > args.max_speed_ms {
        return Some("speed_limit");
    }
    if sample.yaw_rate_rad_s.abs() > args.max_yaw_rate_rad_s {
        return Some("yaw_rate_limit");
    }
    if sample.ax_ms2.abs() > args.max_abs_ax_ms2 {
        return Some("ax_limit");
    }
    if args.mode.source == Source::Live && sample.front_obstacle_m < args.obstacle_min_m {
        return Some("front_obstacle");
    }
    None
}

fn update_summary(summary: &mut BenchSummary, sample: &BenchSample) {
    summary.samples = summary.samples.saturating_add(1);
    summary.max_speed_ms = summary.max_speed_ms.max(sample.speed_ms.abs());
    summary.max_abs_yaw_rate_rad_s = summary
        .max_abs_yaw_rate_rad_s
        .max(sample.yaw_rate_rad_s.abs());
    summary.max_abs_ax_ms2 = summary.max_abs_ax_ms2.max(sample.ax_ms2.abs());
    summary.final_speed_ms = sample.speed_ms;
}

struct CsvWriter {
    inner: Box<dyn Write>,
}

impl CsvWriter {
    fn new(path: Option<&str>) -> Result<Self> {
        let inner: Box<dyn Write> = match path {
            Some(path) => Box::new(BufWriter::new(std::fs::File::create(path)?)),
            None => Box::new(BufWriter::new(std::io::stdout())),
        };
        let mut writer = Self { inner };
        writeln!(
            writer.inner,
            "source,script,t_s,phase,steering_rad,throttle,speed_ms,yaw_rate_rad_s,ax_ms2,ay_ms2,kappa_est_m_inv,front_obstacle_m,abort_reason"
        )?;
        Ok(writer)
    }

    fn write(
        &mut self,
        args: &Args,
        sample: &BenchSample,
        command: &CommandSample,
        abort_reason: Option<&str>,
    ) -> Result<()> {
        let source = match args.mode.source {
            Source::Sim => "sim",
            Source::Live => "live",
        };
        let script = match args.mode.script {
            Script::ThrottleStep => "throttle_step",
            Script::Coastdown => "coastdown",
            Script::SteeringSweep => "steering_sweep",
        };
        let kappa_est = sample.yaw_rate_rad_s / sample.speed_ms.max(KAPPA_SPEED_FLOOR_MS);
        writeln!(
            self.inner,
            "{source},{script},{:.6},{},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{}",
            sample.t_s,
            command.phase,
            command.steering_rad,
            command.throttle,
            sample.speed_ms,
            sample.yaw_rate_rad_s,
            sample.ax_ms2,
            sample.ay_ms2,
            kappa_est,
            sample.front_obstacle_m,
            abort_reason.unwrap_or("")
        )?;
        Ok(())
    }
}

fn install_signal_flag() -> Arc<AtomicBool> {
    let abort = Arc::new(AtomicBool::new(false));
    for signal in [2, 15] {
        let flag = abort.clone();
        unsafe {
            let _ = signal_hook_registry::register(signal, move || {
                flag.store(true, Ordering::SeqCst);
            });
        }
    }
    abort
}

impl Args {
    fn parse() -> Result<Self> {
        let mut raw = std::env::args().skip(1);
        let Some(mode_raw) = raw.next() else {
            print_help();
            bail!("missing mode");
        };
        if mode_raw == "--help" || mode_raw == "-h" {
            print_help();
            std::process::exit(0);
        }
        let mode = parse_mode(&mode_raw)?;
        let mut args = Args {
            mode,
            config: None,
            csv: None,
            json: None,
            hz: DEFAULT_HZ,
            duration_s: DEFAULT_DURATION_S,
            max_speed_ms: DEFAULT_MAX_SPEED_MS,
            max_throttle: DEFAULT_MAX_THROTTLE,
            max_steering_rad: DEFAULT_MAX_STEERING_RAD,
            max_yaw_rate_rad_s: DEFAULT_MAX_YAW_RATE_RAD_S,
            max_abs_ax_ms2: DEFAULT_MAX_ABS_AX_MS2,
            obstacle_min_m: DEFAULT_OBSTACLE_MIN_M,
            understand_open_loop: false,
            model: SimModel::Dynamic,
        };

        while let Some(flag) = raw.next() {
            match flag.as_str() {
                "--config" => args.config = Some(next_value(&mut raw, &flag)?),
                "--csv" => args.csv = Some(next_value(&mut raw, &flag)?),
                "--json" => args.json = Some(next_value(&mut raw, &flag)?),
                "--hz" => args.hz = parse_u64(&next_value(&mut raw, &flag)?, &flag)?,
                "--duration-s" => {
                    args.duration_s = parse_f32(&next_value(&mut raw, &flag)?, &flag)?
                }
                "--max-speed-ms" => {
                    args.max_speed_ms = parse_f32(&next_value(&mut raw, &flag)?, &flag)?
                }
                "--max-throttle" => {
                    args.max_throttle = parse_f32(&next_value(&mut raw, &flag)?, &flag)?
                }
                "--max-steering-rad" => {
                    args.max_steering_rad = parse_f32(&next_value(&mut raw, &flag)?, &flag)?
                }
                "--max-yaw-rate-rad-s" => {
                    args.max_yaw_rate_rad_s = parse_f32(&next_value(&mut raw, &flag)?, &flag)?
                }
                "--max-abs-ax-ms2" => {
                    args.max_abs_ax_ms2 = parse_f32(&next_value(&mut raw, &flag)?, &flag)?
                }
                "--obstacle-min-m" => {
                    args.obstacle_min_m = parse_f32(&next_value(&mut raw, &flag)?, &flag)?
                }
                "--model" => {
                    args.model = match next_value(&mut raw, &flag)?.as_str() {
                        "dynamic" => SimModel::Dynamic,
                        "kinematic" => SimModel::Kinematic,
                        other => bail!("unknown --model {other:?}; expected dynamic or kinematic"),
                    }
                }
                "--i-understand-open-loop-driving" => args.understand_open_loop = true,
                "--help" | "-h" => {
                    print_help();
                    std::process::exit(0);
                }
                other => bail!("unknown argument {other:?}"),
            }
        }
        Ok(args)
    }
}

fn parse_mode(raw: &str) -> Result<Mode> {
    let (source, script) = match raw {
        "sim-throttle-step" => (Source::Sim, Script::ThrottleStep),
        "sim-coastdown" => (Source::Sim, Script::Coastdown),
        "sim-steering-sweep" => (Source::Sim, Script::SteeringSweep),
        "live-throttle-step" => (Source::Live, Script::ThrottleStep),
        "live-coastdown" => (Source::Live, Script::Coastdown),
        "live-steering-sweep" => (Source::Live, Script::SteeringSweep),
        other => bail!("unknown bench mode {other:?}"),
    };
    Ok(Mode { source, script })
}

fn next_value(raw: &mut impl Iterator<Item = String>, flag: &str) -> Result<String> {
    raw.next().ok_or_else(|| anyhow!("{flag} requires a value"))
}

fn parse_f32(value: &str, flag: &str) -> Result<f32> {
    value
        .parse()
        .with_context(|| format!("invalid {flag} value {value:?}"))
}

fn parse_u64(value: &str, flag: &str) -> Result<u64> {
    value
        .parse()
        .with_context(|| format!("invalid {flag} value {value:?}"))
}

fn print_help() {
    eprintln!(
        "usage: car-bench <mode> [options]\n\n\
         modes:\n\
           sim-throttle-step | sim-coastdown | sim-steering-sweep\n\
           live-throttle-step | live-coastdown | live-steering-sweep\n\n\
         live modes are open-loop on-ground tests and require:\n\
           --i-understand-open-loop-driving\n\n\
         conservative starting defaults:\n\
           --duration-s {DEFAULT_DURATION_S}\n\
           --max-speed-ms {DEFAULT_MAX_SPEED_MS}\n\
           --max-throttle {DEFAULT_MAX_THROTTLE}\n\
           --max-steering-rad {DEFAULT_MAX_STEERING_RAD}\n\
           --max-yaw-rate-rad-s {DEFAULT_MAX_YAW_RATE_RAD_S}\n\
           --max-abs-ax-ms2 {DEFAULT_MAX_ABS_AX_MS2}\n\n\
         options:\n\
           --config <nfe.toml>\n\
           --csv <samples.csv>\n\
           --json <summary.json>\n\
           --model <dynamic|kinematic>       sim only [default: dynamic]\n\
           --hz <rate>\n\
           --duration-s <seconds>\n\
           --max-speed-ms <m/s>\n\
           --max-throttle <0..0.5>\n\
           --max-steering-rad <rad>\n\
           --max-yaw-rate-rad-s <rad/s>\n\
           --max-abs-ax-ms2 <m/s^2>\n\
           --obstacle-min-m <m>"
    );
}

#[allow(dead_code)]
fn _core_command(command: CommandSample) -> ControlOutput {
    ControlOutput {
        steering_rad: command.steering_rad,
        throttle: command.throttle,
        ..Default::default()
    }
}
