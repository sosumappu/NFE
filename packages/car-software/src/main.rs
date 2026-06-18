/// main.rs — NFE autonomous RC car entry point
///
/// Run modes
/// ─────────
///   cargo run --release                            Live (no recording)
///   cargo run --release -- --record session.mcap   Live + MCAP record
///   cargo run --release -- --replay session.mcap   MCAP replay (realtime)
///   cargo run --release -- --replay s.mcap --fast  MCAP replay (fast)
///   cargo run --release -- --sim track.json        Simulation
///   STREAM=1 cargo run --release                   Live + Foxglove WebSocket
///
/// Recording is always MCAP.  Replay reads MCAP.  The custom UDP streamer
/// and bincode recorder are removed — Foxglove Studio is the single tool
/// for both live inspection and post-run analysis.
mod control;
mod fusion;
mod hal;
mod init;
mod metrics;
mod replay;
mod sensors;
mod sim;
mod state;
mod stream;
mod types;

use std::sync::Arc;

use anyhow::Result;
#[cfg(target_os = "linux")]
use libsystemd::daemon::{self, NotifyState};
use tokio::{
    runtime::Builder,
    signal::unix::{signal, SignalKind},
    time::{interval, MissedTickBehavior},
};
use tracing::{error, info, warn};

use control::{
    actuate::{ActuatorFactory, THROTTLE_MAX},
    lqr::{Lqr, LqrState},
    mpt::Mpt,
    pid::Pid,
    speed::SpeedPlanner,
    watchdog::{Watchdog, WATCHDOG_MAX_MISSED},
};
use fusion::kinematics::Kinematics;
use hal::{ActuatorSink, SensorSource};
use init::{ReadinessBarrier, ReadySignal, Sensor};
use metrics::{MetricsLog, TickMetrics};
use replay::{
    live_source::LiveSensorSource,
    recorder::{ControlFrame, McapMessage, McapRecorder},
    replayer::{McapReplayer, ReplayMode},
};
use sensors::factory::{SensorFactory, SensorReadySignals};
use sim::{
    model::{DynamicBicycle, IdentifiedModel, KinematicBicycle},
    source::SimulatorSource,
    world::World,
};
use state::{SensorStateWriter, SharedState};
use stream::foxglove_bridge::{FoxgloveBridge, DEFAULT_PORT};
use types::{ImuBias, LidarPoint};

// ── Config ─────────────────────────────────────────────────────────────────

const KINEMATICS_HORIZON: usize = 500;
const CONTROL_HZ: u64 = 100;
const CONTROL_DT: f32 = 1.0 / CONTROL_HZ as f32;
const CONTROL_PERIOD: std::time::Duration = std::time::Duration::from_millis(1000 / CONTROL_HZ);
const LIDAR_PORT: &str = "/dev/lidar";
const INIT_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(5);
const ESTOP_DIST_M: f32 = 0.30;

// ── CLI args ───────────────────────────────────────────────────────────────

struct Args {
    replay: Option<String>,
    record: Option<String>,
    sim: Option<String>,
    model: String,
    model_params: Option<String>,
    fast: bool,
    stream: bool,
    stream_port: u16,
    cost_out: Option<String>,
    csv_out: Option<String>,
}

impl Args {
    fn parse() -> Self {
        let args: Vec<String> = std::env::args().collect();
        let get = |flag: &str| -> Option<String> {
            args.windows(2).find(|w| w[0] == flag).map(|w| w[1].clone())
        };
        let has = |flag: &str| args.iter().any(|a| a == flag);
        Self {
            replay: get("--replay"),
            record: get("--record"),
            sim: get("--sim"),
            model: get("--model").unwrap_or_else(|| "kinematic".into()),
            model_params: get("--model-params"),
            fast: has("--fast"),
            stream: has("--stream") || std::env::var("STREAM").is_ok(),
            stream_port: get("--stream-port")
                .and_then(|v| v.parse().ok())
                .unwrap_or(DEFAULT_PORT),
            cost_out: get("--cost-out"),
            csv_out: get("--csv-out"),
        }
    }
}

// ── mlockall ──────────────────────────────────────────────────────────────

fn lock_memory() {
    #[cfg(target_os = "linux")]
    unsafe {
        let flags: libc::c_int = if std::env::var("JOURNAL_STREAM").is_ok() {
            3
        } else {
            1
        };
        extern "C" {
            fn mlockall(flags: libc::c_int) -> libc::c_int;
        }
        if mlockall(flags) != 0 {
            eprintln!("mlockall failed — check LimitMEMLOCK=infinity");
        } else {
            info!("memory: pages locked (flags={})", flags);
        }
    }
}

// ── Entry ──────────────────────────────────────────────────────────────────

fn main() -> Result<()> {
    init_tracing()?;
    let args = Args::parse();
    info!("car: NFE starting");
    lock_memory();

    let rt = Builder::new_current_thread()
        .enable_time()
        .enable_io()
        .build()?;

    rt.block_on(async move {
        match (&args.replay, &args.sim) {
            (Some(path), _) => run_replay(path.clone(), args.fast, &args).await,
            (_, Some(path)) => run_sim(path.clone(), &args).await,
            _ => run_live(args).await,
        }
    })
}

// ══════════════════════════════════════════════════════════════════════════
//  Live mode
// ══════════════════════════════════════════════════════════════════════════

async fn run_live(args: Args) -> Result<()> {
    let state = SharedState::new();
    let (barrier, _signals) = ReadinessBarrier::new();

    let signals = SensorReadySignals {
        lidar: ReadySignal::dummy(Sensor::Lidar),
        imu: ReadySignal::dummy(Sensor::Imu),
        sonars: vec![
            ReadySignal::dummy(Sensor::Sonar(0)),
            ReadySignal::dummy(Sensor::Sonar(1)),
            ReadySignal::dummy(Sensor::Sonar(2)),
        ],
    };

    // ── MCAP recorder (always-on when --record is supplied) ────────────────
    let (recorder, mcap_tx): (Option<McapRecorder>, Option<_>) = if let Some(ref path) = args.record
    {
        match McapRecorder::start(path) {
            Ok(r) => {
                let tx = r.sender();
                info!("live: MCAP recording to {path}");
                (Some(r), Some(tx))
            }
            Err(e) => {
                warn!("live: MCAP recorder failed ({e:#}) — continuing without recording");
                (None, None)
            }
        }
    } else {
        (None, None)
    };

    let state_writer: Arc<dyn SensorStateWriter> = state.clone();
    let spawned = SensorFactory::spawn_all(&state_writer, LIDAR_PORT.to_string(), signals);
    if !spawned.skipped.is_empty() {
        warn!("live: degraded sensors: {:?}", spawned.skipped);
    }

    // ── Foxglove WebSocket bridge (live streaming) ─────────────────────────
    let _bridge = if args.stream {
        match FoxgloveBridge::start(state.clone(), args.stream_port, 50) {
            Ok(b) => {
                info!("live: Foxglove bridge on ws://0.0.0.0:{}", args.stream_port);
                Some(b)
            }
            Err(e) => {
                warn!("live: Foxglove bridge failed ({e:#})");
                None
            }
        }
    } else {
        None
    };

    if let Err(e) = barrier.wait_all_ready(INIT_TIMEOUT).await {
        error!("live: INIT FAILED — {e}");
        #[cfg(target_os = "linux")]
        let _ = daemon::notify(false, &[NotifyState::Other("STATUS=init failed".into())]);
        std::process::exit(1);
    }

    #[cfg(target_os = "linux")]
    let _ = daemon::notify(false, &[NotifyState::Ready]);
    info!("live: all sensors ready — starting control loop");

    let source: Box<dyn SensorSource> = Box::new(LiveSensorSource::new(state.clone()));
    let actuator: Box<dyn ActuatorSink> = ActuatorFactory::build(10);

    let result = control_loop(source, actuator, Some(state.clone()), mcap_tx, &args).await;

    info!("live: shutdown — joining sensor threads");
    drop(state_writer);
    for h in spawned.handles {
        if let Err(e) = h.join() {
            warn!("sensor thread panicked: {:?}", e);
        }
    }
    if let Some(rec) = recorder {
        info!("live: flushing MCAP");
        rec.finish();
    }
    result
}

// ══════════════════════════════════════════════════════════════════════════
//  Replay mode — MCAP only
// ══════════════════════════════════════════════════════════════════════════

async fn run_replay(path: String, fast: bool, args: &Args) -> Result<()> {
    info!("replay: loading {path}");
    let mode = if fast {
        ReplayMode::Fast
    } else {
        ReplayMode::Realtime
    };
    let source: Box<dyn SensorSource> = Box::new(McapReplayer::open(&path, mode)?);
    let actuator: Box<dyn ActuatorSink> = ActuatorFactory::build(1);
    info!("replay: starting control loop ({mode:?})");
    control_loop(source, actuator, None, None, args).await
}

// ══════════════════════════════════════════════════════════════════════════
//  Simulation mode
// ══════════════════════════════════════════════════════════════════════════

async fn run_sim(world_path: String, args: &Args) -> Result<()> {
    info!("sim: loading world from {world_path}");
    let world = World::load(&world_path)?;
    info!(
        "sim: {} walls  {} waypoints  model={}",
        world.walls.len(),
        world.waypoints.len(),
        args.model
    );

    let model: Box<dyn sim::model::VehicleModel> = match args.model.as_str() {
        "dynamic" => Box::new(DynamicBicycle::default()),
        "identified" => {
            let p = args
                .model_params
                .as_deref()
                .ok_or_else(|| anyhow::anyhow!("--model-params required"))?;
            Box::new(IdentifiedModel::from_json(p)?)
        }
        _ => Box::new(KinematicBicycle::default()),
    };

    let (source, actuator) = SimulatorSource::new(world, model, CONTROL_DT);
    control_loop(Box::new(source), Box::new(actuator), None, None, args).await
}

// ══════════════════════════════════════════════════════════════════════════
//  IMU calibration
// ══════════════════════════════════════════════════════════════════════════

async fn calibrate_imu(
    source: &mut dyn SensorSource,
    actuator: &mut dyn ActuatorSink,
    state: Option<&Arc<dyn SensorStateWriter>>,
    ticker: &mut tokio::time::Interval,
) -> Result<ImuBias> {
    info!("car: IMU calibration — keep vehicle stationary");
    actuator.safe_state()?;
    const N: usize = 200;
    let mut samples = Vec::with_capacity(N);
    while samples.len() < N {
        if state.is_some() {
            ticker.tick().await;
        }
        if state.map_or(source.is_exhausted(), |s| s.is_shutdown()) {
            anyhow::bail!("calibration aborted");
        }
        samples.push(source.next_snapshot()?.imu);
        actuator.safe_state()?;
    }
    let bias = ImuBias::estimate(&samples);
    info!("car: calibration complete — {bias:?}");
    Ok(bias)
}

// ══════════════════════════════════════════════════════════════════════════
//  Unified control loop
// ══════════════════════════════════════════════════════════════════════════

async fn control_loop(
    mut source: Box<dyn SensorSource>,
    mut actuator: Box<dyn ActuatorSink>,
    state: Option<Arc<dyn SensorStateWriter>>,
    mcap_tx: Option<std::sync::mpsc::SyncSender<McapMessage>>,
    args: &Args,
) -> Result<()> {
    if let Some(ref s) = state {
        let s2 = s.clone();
        tokio::spawn(async move {
            let mut sigterm = signal(SignalKind::terminate()).unwrap();
            let mut sigint = signal(SignalKind::interrupt()).unwrap();
            tokio::select! {
                _ = sigterm.recv() => {},
                _ = sigint.recv()  => {},
            }
            warn!("car: shutdown signal");
            s2.set_shutdown();
        });
    }

    let mut ticker = interval(CONTROL_PERIOD);
    ticker.set_missed_tick_behavior(MissedTickBehavior::Skip);

    let bias = match calibrate_imu(
        source.as_mut(),
        actuator.as_mut(),
        state.as_ref(),
        &mut ticker,
    )
    .await
    {
        Ok(b) => b,
        Err(_) => return Ok(()),
    };

    let mut kin = Kinematics::new(KINEMATICS_HORIZON, bias);
    let speed = SpeedPlanner::new(THROTTLE_MAX, 1.0, 1.0);
    let lqr = Lqr::new();
    let mpt = Mpt::new();
    let mut pid = Pid::new_with_dt(1.5, 0.05, 0.2, CONTROL_DT);
    let watchdog = Watchdog::new();
    let mut log = MetricsLog::new();
    let mut deskew_buf: Vec<LidarPoint> = Vec::with_capacity(256);
    let mut tick: u64 = 0;
    let mut last_lidar_ts = 0;
    let start = std::time::Instant::now();

    loop {
        if state.is_some() {
            ticker.tick().await;
        }

        let done = state
            .as_ref()
            .map_or(source.is_exhausted(), |s| s.is_shutdown());
        if done {
            actuator.safe_state()?;
            info!(
                "car: shutdown after {tick} ticks ({:.1}s)",
                start.elapsed().as_secs_f32()
            );
            break;
        }

        let t0 = std::time::Instant::now();
        let snap = source.next_snapshot()?;

        // ── Nearest obstacle ──────────────────────────────────────────────
        let nearest_m = {
            let s = snap.sonar_m.iter().cloned().fold(f32::MAX, f32::min);
            let l = snap
                .lidar
                .nearest_in_arc(0.0, 30.0_f32.to_radians())
                .map_or(f32::MAX, |p| p.dist_m);
            s.min(l)
        };

        // ── ESTOP ─────────────────────────────────────────────────────────
        if snap.obstacle_closer_than(ESTOP_DIST_M) {
            actuator.safe_state()?;
            watchdog.kick();
            warn!("car: ESTOP nearest={nearest_m:.2}m");

            let m = TickMetrics {
                tick,
                timestamp_us: snap.lidar.timestamp_us,
                loop_us: t0.elapsed().as_micros() as u32,
                nearest_obstacle_m: nearest_m,
                estop: true,
                ..Default::default()
            };
            log.push(m);
            send_metrics(&mcap_tx, m);
            tick += 1;
            continue;
        }

        // ── Control ───────────────────────────────────────────────────────
        let handle = kin.update(&snap.imu);
        let cloud = kin.deskew(&snap.lidar, &mut deskew_buf);
        let (err_dist, heading_err) = mpt.compute(&cloud);
        let lateral_m = err_dist * heading_err.sin();
        kin.record_lateral_error(handle, lateral_m);

        let lqr_state = LqrState {
            lateral_error_m: lateral_m,
            lateral_rate_m_s: kin.lateral_rate(),
            heading_error_rad: heading_err,
            yaw_rate_rad_s: kin.current_yaw_rate(),
        };
        let steering = lqr.compute_lateral(&lqr_state);
        let v_target = speed.compute(&cloud, err_dist, heading_err);
        let v_current = kin.current_speed();
        let throttle = pid.compute_longitudinal(v_target - v_current);

        actuator.set_steering(steering)?;
        actuator.set_throttle(throttle)?;

        // ── Watchdog ──────────────────────────────────────────────────────
        let loop_us = t0.elapsed().as_micros() as u32;
        let wd_miss = if state.is_some() {
            if loop_us as u128 > CONTROL_PERIOD.as_micros() {
                if watchdog.miss() >= WATCHDOG_MAX_MISSED {
                    error!("car: watchdog — safe state");
                    actuator.safe_state()?;
                }
                true
            } else {
                watchdog.kick();
                false
            }
        } else {
            false
        };

        // ── Record ────────────────────────────────────────────────────────
        let ts = snap.lidar.timestamp_us;
        let m = TickMetrics {
            tick,
            timestamp_us: ts,
            loop_us,
            lateral_error_m: lateral_m,
            heading_error_rad: heading_err,
            steering_rad: steering,
            throttle,
            target_speed_ms: v_target,
            current_speed_ms: v_current,
            nearest_obstacle_m: nearest_m,
            gz_rad_s: snap.imu.gz,
            vy_ms: kin.lateral_rate() * CONTROL_DT,
            estop: false,
            watchdog_miss: wd_miss,
            sensor_fault: snap.sensor_fault,
        };
        log.push(m);

        if let Some(ref tx) = mcap_tx {
            use hal::{SensorFrame, TimestampedFrame};
            // Sensor frames
            let _ = tx.try_send(McapMessage::Sensor(TimestampedFrame {
                ts_us: ts,
                frame: SensorFrame::Imu(snap.imu),
            }));
            if snap.lidar.timestamp_us != last_lidar_ts {
                let _ = tx.try_send(McapMessage::Sensor(TimestampedFrame {
                    ts_us: snap.lidar.timestamp_us,
                    frame: SensorFrame::Lidar((*snap.lidar).clone()),
                }));
                last_lidar_ts = snap.lidar.timestamp_us;
            }
            let _ = tx.try_send(McapMessage::Sensor(TimestampedFrame {
                ts_us: ts,
                frame: SensorFrame::Sonar {
                    front: snap.sonar_m[0],
                    left: snap.sonar_m[1],
                    right: snap.sonar_m[2],
                },
            }));
            // Control + metrics
            let _ = tx.try_send(McapMessage::Control(ControlFrame {
                timestamp_us: ts,
                steering_rad: steering,
                throttle,
                target_speed: v_target,
                current_speed: v_current,
            }));
            send_metrics(&mcap_tx, m);
        }

        tick += 1;
        if tick % 10 == 0 {
            tracing::info!(
                tick,
                loop_us,
                steering = format!("{steering:.3}"),
                throttle = format!("{throttle:.3}"),
                lat_err = format!("{lateral_m:.3}"),
                nearest = format!("{nearest_m:.2}"),
                "tick"
            );
        }
    }

    // ── Post-run ───────────────────────────────────────────────────────────
    let cost = log.summarise();
    info!("car: {cost}");
    if let Some(ref p) = args.cost_out {
        log.cost_to_json(p)?;
    }
    if let Some(ref p) = args.csv_out {
        log.to_csv(p)?;
    }

    Ok(())
}

// ── Helpers ────────────────────────────────────────────────────────────────

#[inline]
fn send_metrics(tx: &Option<std::sync::mpsc::SyncSender<McapMessage>>, m: TickMetrics) {
    if let Some(ref tx) = tx {
        let _ = tx.try_send(McapMessage::Metrics(Box::new(m)));
    }
}

fn init_tracing() -> Result<()> {
    use tracing_subscriber::prelude::*;
    let fmt = tracing_subscriber::fmt::layer().with_filter(
        tracing_subscriber::EnvFilter::from_env("RUST_LOG")
            .add_directive("car=debug".parse().unwrap()),
    );

    #[cfg(target_os = "linux")]
    if std::env::var("JOURNAL_STREAM").is_ok() {
        let jd = tracing_journald::layer()?;
        tracing::subscriber::set_global_default(tracing_subscriber::registry().with(fmt).with(jd))?;
        return Ok(());
    }
    tracing::subscriber::set_global_default(tracing_subscriber::registry().with(fmt))?;
    Ok(())
}
