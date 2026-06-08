/// main.rs — NFE autonomous RC car entry point
///
/// Run modes (selected via CLI args or environment):
///
///   Live (default)
///     cargo run --release
///     Spawns sensor threads, optionally records the session, optionally
///     streams sensor data over UDP.
///
///   Live + record
///     cargo run --release -- --record session.bin
///
///   Live + stream
///     STREAM=1 cargo run --release
///     (or)  cargo run --release -- --stream
///
///   Replay
///     cargo run --release -- --replay session.bin
///     Feeds a recorded session through the control loop. No hardware needed.
///     Use --fast to replay at maximum speed (no sleep between frames).
mod control;
mod fusion;
mod hal;
mod init;
mod replay;
mod sensors;
mod state;
mod stream;
mod types;

use std::sync::Arc;

use anyhow::Result;
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
use replay::{
    live_source::{LiveSensorSource, RecordingSharedState},
    recorder::Recorder,
    replayer::{ReplayMode, ReplaySource},
};
use sensors::factory::{SensorFactory, SensorReadySignals};
use state::{SensorSnapshot, SensorStateWriter, SharedState};
use stream::streamer::Streamer;
use types::{ImuBias, LidarCloudView, LidarPoint};

// ── Config ─────────────────────────────────────────────────────────────────

const KINEMATICS_HORIZON: usize = 500;
const CONTROL_HZ: u64 = 100;
const CONTROL_PERIOD: std::time::Duration = std::time::Duration::from_millis(1000 / CONTROL_HZ);
const LIDAR_PORT: &str = "/dev/lidar";
const INIT_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(5);
const ESTOP_DIST_M: f32 = 0.30;

// ── CLI args ───────────────────────────────────────────────────────────────

struct Args {
    replay: Option<String>,
    record: Option<String>,
    fast: bool,
    stream: bool,
    stream_port: u16,
    full_cloud: bool,
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
            fast: has("--fast"),
            stream: has("--stream") || std::env::var("STREAM").is_ok(),
            stream_port: get("--stream-port")
                .and_then(|v| v.parse().ok())
                .unwrap_or(stream::streamer::DEFAULT_PORT),
            full_cloud: has("--full-cloud"),
        }
    }
}

// ── mlockall ──────────────────────────────────────────────────────────────

fn lock_memory() {
    #[cfg(target_os = "linux")]
    unsafe {
        // MCL_CURRENT=1, MCL_FUTURE=2
        // Omit MCL_FUTURE when not running as the service user (no LimitMEMLOCK=infinity)
        let flags: libc::c_int = if std::env::var("JOURNAL_STREAM").is_ok() {
            3 // service context: MCL_CURRENT | MCL_FUTURE
        } else {
            1 // dev/manual run: MCL_CURRENT only
        };
        extern "C" {
            fn mlockall(flags: libc::c_int) -> libc::c_int;
        }
        if mlockall(flags) != 0 {
            eprintln!("mlockall failed — check LimitMEMLOCK=infinity in the unit file");
        } else {
            info!("memory: pages locked (flags={})", flags);
        }
    }
}

// ── Entry point ────────────────────────────────────────────────────────────

fn main() -> Result<()> {
    init_tracing()?;
    let args = Args::parse();

    info!("car: NFE autonomous RC car starting");
    lock_memory();

    let rt = Builder::new_current_thread()
        .enable_time()
        .enable_io()
        .build()?;

    rt.block_on(async move {
        if let Some(path) = args.replay {
            run_replay(path, args.fast).await
        } else {
            run_live(args).await
        }
    })
}

// ══════════════════════════════════════════════════════════════════════════
//  Live mode
// ══════════════════════════════════════════════════════════════════════════

async fn run_live(args: Args) -> Result<()> {
    let state = SharedState::new();

    // ── Build readiness barrier ────────────────────────────────────────────
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

    // ── Optional recorder ──────────────────────────────────────────────────
    let (recorder, state_writer): (Option<Recorder>, Arc<dyn SensorStateWriter>) =
        if let Some(ref path) = args.record {
            match Recorder::start(path) {
                Ok(r) => {
                    let rs = RecordingSharedState::new(state.clone(), r.sender());
                    info!("live: recording session to {path}");
                    let writer = rs as Arc<dyn SensorStateWriter>;
                    (Some(r), writer)
                }
                Err(e) => {
                    warn!("live: could not open recorder ({e:#}) — continuing without recording");
                    (None, state.clone() as Arc<dyn SensorStateWriter>)
                }
            }
        } else {
            (None, state.clone() as Arc<dyn SensorStateWriter>)
        };

    // ── Spawn sensor threads ───────────────────────────────────────────────
    let spawned = SensorFactory::spawn_all(&state_writer, LIDAR_PORT.to_string(), signals);

    if !spawned.skipped.is_empty() {
        warn!("live: degraded sensor set: {:?}", spawned.skipped);
    }

    // ── Optional streamer ──────────────────────────────────────────────────
    let _streamer = if args.stream {
        match Streamer::start(state.clone(), args.stream_port, args.full_cloud) {
            Ok(s) => {
                info!("live: streaming on port {}", args.stream_port);
                Some(s)
            }
            Err(e) => {
                warn!("live: streamer failed to start ({e:#})");
                None
            }
        }
    } else {
        None
    };

    // ── Wait for sensors ───────────────────────────────────────────────────
    info!("live: waiting for sensors (timeout={INIT_TIMEOUT:?})");
    if let Err(e) = barrier.wait_all_ready(INIT_TIMEOUT).await {
        error!("live: INIT FAILED — {e}");
        let _ = daemon::notify(false, &[NotifyState::Other("STATUS=init failed".into())]);
        std::process::exit(1);
    }

    let _ = daemon::notify(false, &[NotifyState::Ready]);
    info!("live: all sensors ready — starting control loop");

    // ── Build sensor source and actuator ───────────────────────────────────
    let source: Box<dyn SensorSource> = Box::new(LiveSensorSource::new(state.clone()));
    // log every 10th call (100 Hz → 10 Hz logs) — change to 1 for dev verbosity
    let actuator = ActuatorFactory::build(10);

    let result = control_loop(source, actuator, Some(state.clone())).await;

    // ── Cleanup ────────────────────────────────────────────────────────────
    info!("live: shutdown — waiting for sensor threads");

    // drop state_writeer to close tx
    drop(state_writer);

    // Wait on threads to close to release their tx clonees
    for handle in spawned.handles {
        if let Err(e) = handle.join() {
            warn!("live: sensor thread panicked: {:?}", e);
        }
    }

    if let Some(rec) = recorder {
        info!("recorder: flushing session to disk");
        rec.finish();
        info!("recorder: session saved");
    }

    result
}

// ══════════════════════════════════════════════════════════════════════════
//  Replay mode
// ══════════════════════════════════════════════════════════════════════════

async fn run_replay(path: String, fast: bool) -> Result<()> {
    info!("replay: loading session from {path}");

    let mode = if fast {
        ReplayMode::Fast
    } else {
        ReplayMode::Realtime
    };
    let source: Box<dyn SensorSource> = Box::new(ReplaySource::open(&path, mode)?);

    // In replay mode we always use a dry-run actuator with verbose logging
    let actuator = ActuatorFactory::build(1);

    info!("replay: starting control loop (mode={mode:?})");
    control_loop(source, actuator, None).await
}

// ══════════════════════════════════════════════════════════════════════════
//  Imu Calibrating
// ══════════════════════════════════════════════════════════════════════════

async fn calibrate_imu(
    source: &mut dyn SensorSource,
    actuator: &mut dyn ActuatorSink,
    state: Option<&Arc<dyn SensorStateWriter>>,
    ticker: &mut tokio::time::Interval,
) -> Result<ImuBias> {
    info!("car: starting IMU calibration... keeping vehicle stationary.");
    actuator.safe_state()?;

    const CALIBRATION_TICKS: usize = 200;
    let mut samples = Vec::with_capacity(CALIBRATION_TICKS);

    while samples.len() < CALIBRATION_TICKS {
        if state.is_some() {
            ticker.tick().await;
        }

        // Check for early shutdown during calibration
        if state.map_or(source.is_exhausted(), |s| s.is_shutdown()) {
            info!("car: shutdown requested during calibration.");
            anyhow::bail!("Calibration aborted due to shutdown");
        }

        let snap = source.next_snapshot()?;
        samples.push(snap.imu);

        // Ensure actuator stays completely dead during this phase
        actuator.safe_state()?;
    }

    let bias = ImuBias::estimate(&samples);
    info!("car: calibration complete. Bias: {bias:?}");

    Ok(bias)
}

// ══════════════════════════════════════════════════════════════════════════
//  Unified control loop — works with any SensorSource + ActuatorSink
// ══════════════════════════════════════════════════════════════════════════

async fn control_loop(
    mut source: Box<dyn SensorSource>,
    mut actuator: Box<dyn ActuatorSink>,
    state: Option<Arc<dyn SensorStateWriter>>, // None in replay mode
) -> Result<()> {
    // Graceful shutdown on SIGTERM / SIGINT (live mode only)
    if let Some(ref s) = state {
        let s2 = s.clone();
        tokio::spawn(async move {
            let mut sigterm = signal(SignalKind::terminate()).unwrap();
            let mut sigint = signal(SignalKind::interrupt()).unwrap();
            tokio::select! {
                _ = sigterm.recv() => {},
                _ = sigint.recv()  => {},
            }
            warn!("car: shutdown signal received");
            s2.set_shutdown();
        });
    }

    let mut ticker = interval(CONTROL_PERIOD);
    ticker.set_missed_tick_behavior(MissedTickBehavior::Skip);

    //calibrate imu
    let bias = match calibrate_imu(
        source.as_mut(),
        actuator.as_mut(),
        state.as_ref(),
        &mut ticker,
    )
    .await
    {
        Ok(b) => b,
        Err(_) => return Ok(()), // Clean exit if shutdown was requested
    };

    // init control tools
    let mut kinematics = Kinematics::new(KINEMATICS_HORIZON, bias);
    let speed = SpeedPlanner::new(THROTTLE_MAX, 1.0, 1.0);
    let lqr = Lqr::new();
    let mpt = Mpt::new();
    let mut pid = Pid::new(1.5, 0.05, 0.2);
    let watchdog = Watchdog::new();

    let mut deskew_buffer: Vec<LidarPoint> = Vec::with_capacity(256);
    let mut tick: u64 = 0;
    let start = std::time::Instant::now();

    loop {
        // In replay mode we don't pace via the ticker — the source does it
        if state.is_some() {
            ticker.tick().await;
        }

        // Check shutdown
        let should_shutdown = state
            .as_ref()
            .map_or(source.is_exhausted(), |s| s.is_shutdown());

        if should_shutdown {
            actuator.safe_state()?;
            info!(
                "car: clean shutdown after {tick} ticks ({:.1}s)",
                start.elapsed().as_secs_f32()
            );
            break;
        }

        let t0 = std::time::Instant::now();
        let snap = source.next_snapshot()?;

        // ── ESTOP ─────────────────────────────────────────────────────────
        if snap.obstacle_closer_than(ESTOP_DIST_M) {
            actuator.safe_state()?;
            watchdog.kick();
            warn!(
                "car: ESTOP  lidar_front={:.2}m  sonar={:.2}/{:.2}/{:.2}m",
                snap.lidar
                    .nearest_in_arc(0.0, 30.0)
                    .map_or(f32::MAX, |p| p.dist_m),
                snap.sonar_m[0],
                snap.sonar_m[1],
                snap.sonar_m[2],
            );
            continue;
        }

        // ── Control ───────────────────────────────────────────────────────
        // TODO: Calculate the Ricatti equation solution

        let handle = kinematics.update(&snap.imu); // add imut sample to kinematics to deskew with speed
        let cloud: LidarCloudView = kinematics.deskew(&snap.lidar, &mut deskew_buffer); // create a deskewed view of our
                                                                                        // cloud
        let (err_dist, heading_err) = mpt.compute(&cloud);

        let lateral_error_m = err_dist * heading_err.sin();
        kinematics.record_lateral_error(handle, lateral_error_m);

        let lqr_state = LqrState {
            lateral_error_m,
            lateral_rate_m_s: kinematics.lateral_rate(),
            heading_error_rad: heading_err,
            yaw_rate_rad_s: kinematics.current_yaw_rate(),
        };

        let steering_rad = lqr.compute_lateral(&lqr_state);

        let target_speed = speed.compute(&cloud, err_dist, heading_err);
        let current_speed = kinematics.current_speed();

        let throttle = pid.compute_longitudinal(target_speed - current_speed);

        actuator.set_steering(steering_rad)?;
        actuator.set_throttle(throttle)?;

        // ── Watchdog ─────────────────────────────────────────────────────
        let loop_us = t0.elapsed().as_micros();

        if state.is_some() {
            // Disable in replay mode
            if loop_us > CONTROL_PERIOD.as_micros() {
                if watchdog.miss() >= WATCHDOG_MAX_MISSED {
                    error!("car: watchdog — {WATCHDOG_MAX_MISSED} overruns — safe state");
                    actuator.safe_state()?;
                }
            } else {
                watchdog.kick();
            }
        }

        // ── Logging (every 10 ticks) ──────────────────────────────────────
        tick += 1;
        if tick % 10 == 0 {
            tracing::info!(
                tick,
                loop_us,
                uptime_s = start.elapsed().as_secs(),
                steering = format!("{:.3}", steering_rad),
                throttle = format!("{:.3}", throttle),
                lidar_pts = snap.lidar.points.len(),
                lidar_front_m = snap
                    .lidar
                    .nearest_in_arc(0.0, 30.0)
                    .map_or(-1.0, |p| p.dist_m),
                sonar = format!(
                    "{:.2}/{:.2}/{:.2}",
                    snap.sonar_m[0], snap.sonar_m[1], snap.sonar_m[2]
                ),
                gz_rad_s = format!("{:.3}", snap.imu.gz),
                "tick"
            );
        }
    }

    Ok(())
}

// ── Tracing ───────────────────────────────────────────────────────────────

fn init_tracing() -> Result<()> {
    use tracing_subscriber::prelude::*;
    let fmt = tracing_subscriber::fmt::layer().with_filter(
        tracing_subscriber::EnvFilter::from_env("RUST_LOG")
            .add_directive("car=debug".parse().unwrap()),
    );

    if std::env::var("JOURNAL_STREAM").is_ok() {
        let jd = tracing_journald::layer()?;
        tracing::subscriber::set_global_default(tracing_subscriber::registry().with(fmt).with(jd))?;
    } else {
        tracing::subscriber::set_global_default(tracing_subscriber::registry().with(fmt))?;
    }
    Ok(())
}
