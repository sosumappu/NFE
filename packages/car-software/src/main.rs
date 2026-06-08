/// main.rs — NFE autonomous RC car entry point

mod control;
mod init;
mod sensors;
mod state;

use std::sync::Arc;

use libsystemd::daemon::{self, NotifyState};
use rppal::gpio::Gpio;
use tokio::{
    runtime::Builder,
    signal::unix::{signal, SignalKind},
    time::{interval, MissedTickBehavior},
};
use tracing::{error, info, warn};

use control::{Actuate, Lqr, Pid, Watchdog, WATCHDOG_MAX_MISSED};
use init::ReadinessBarrier;
use state::SharedState;

// ── Config ───────────────────────────────────────────────────────────────────

const CONTROL_HZ: u64 = 100;
const CONTROL_PERIOD: std::time::Duration =
    std::time::Duration::from_millis(1000 / CONTROL_HZ);

const LIDAR_PORT: &str = "/dev/lidar";

/// How long to wait for all sensors to produce their first valid reading.
const INIT_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(5);

// ── mlockall ─────────────────────────────────────────────────────────────────

fn lock_memory() {
    unsafe {
        extern "C" { fn mlockall(flags: libc::c_int) -> libc::c_int; }
        if mlockall(3) != 0 {
            eprintln!("mlockall failed — check LimitMEMLOCK=infinity in unit file");
        } else {
            info!("memory: all pages locked (MCL_CURRENT | MCL_FUTURE)");
        }
    }
}

// ── Entry point ───────────────────────────────────────────────────────────────

fn main() -> anyhow::Result<()> {
    init_tracing()?;
    info!("car: NFE autonomous RC car starting");
    lock_memory();

    let state = SharedState::new();
    let gpio  = Gpio::new()?;

    // ── Build readiness barrier ───────────────────────────────────
    // Signals ordered to match REQUIRED in init.rs:
    //   [Lidar, Imu, Sonar(0), Sonar(1), Sonar(2)]
    let (barrier, mut signals) = ReadinessBarrier::new();
    let sig_lidar  = signals.remove(0);
    let sig_imu    = signals.remove(0);
    let sig_sonars = signals; // remaining 3

    // ── Spawn sensor threads ──────────────────────────────────────
    sensors::lidar::spawn(state.clone(), sig_lidar,  LIDAR_PORT.to_string());
    sensors::imu::spawn(state.clone(),   sig_imu);
    sensors::sonar::spawn_all(state.clone(), &gpio,  sig_sonars);

    info!("car: sensor threads spawned — waiting for readiness (timeout={INIT_TIMEOUT:?})");

    // ── Tokio runtime (current_thread — stays on isolated core 3) ─
    let rt = Builder::new_current_thread()
        .enable_time()
        .build()?;

    let actuate = Actuate::new(&gpio)?;

    rt.block_on(async move {
        // ── Init barrier — abort hard if any sensor doesn't respond ─
        if let Err(e) = barrier.wait_all_ready(INIT_TIMEOUT).await {
            error!("car: INIT FAILED — {e}");
            // Tell systemd we are stopping cleanly so it logs the right reason.
            // Restart=always will retry after RestartSec.
            let _ = daemon::notify(false, &[NotifyState::Other("STATUS=init failed".into())]);
            std::process::exit(1);
        }

        // All sensors confirmed ready
        let _ = daemon::notify(false, &[NotifyState::Ready]);
        info!("car: all sensors ready — starting control loop");

        control_loop(state, actuate).await
    })
}

// ── Control loop ──────────────────────────────────────────────────────────────

async fn control_loop(state: Arc<SharedState>, mut actuate: Actuate) -> anyhow::Result<()> {
    let     lqr      = Lqr::new();
    let mut pid      = Pid::new(1.5, 0.05, 0.2);
    let     watchdog = Watchdog::new();

    // Graceful shutdown on SIGTERM / SIGINT
    {
        let state2 = state.clone();
        tokio::spawn(async move {
            let mut sigterm = signal(SignalKind::terminate()).unwrap();
            let mut sigint  = signal(SignalKind::interrupt()).unwrap();
            tokio::select! {
                _ = sigterm.recv() => {},
                _ = sigint.recv()  => {},
            }
            warn!("car: shutdown signal received");
            state2.shutdown.store(true, std::sync::atomic::Ordering::Relaxed);
        });
    }

    let mut ticker = interval(CONTROL_PERIOD);
    ticker.set_missed_tick_behavior(MissedTickBehavior::Skip);

    let mut tick: u64 = 0;
    let start = std::time::Instant::now();

    loop {
        ticker.tick().await;
        let t0 = std::time::Instant::now();

        if state.is_shutdown() {
            actuate.safe_state();
            info!(
                "car: clean shutdown after {tick} ticks ({:.1}s)",
                start.elapsed().as_secs_f32()
            );
            break;
        }

        let snap = state.snapshot(); // snapshot l'état à l'instant t

        // régle d'arret si un point est trop prét
        // WARN: peut ne pas avoir le behaviour attendu
        if snap.obstacle_closer_than(0.30) {
            actuate.safe_state();
            watchdog.kick();
            warn!(
                "car: ESTOP  lidar_front={:.2}m  sonar={:.2}/{:.2}/{:.2}m",
                snap.lidar.nearest_in_arc(0.0, 30.0).map_or(f32::MAX, |p| p.dist_m),
                snap.sonar_m[0], snap.sonar_m[1], snap.sonar_m[2],
            );
            continue;
        }

        // valeur non finale
        // TODO: implémenté l'algo de willen
        let lateral_state = [
            0.0,          // lateral error  (stub — replace with path planner)
            snap.imu.ay,  // lateral accel as proxy for rate
            0.0,          // heading error  (stub)
            snap.imu.gz,  // yaw rate
        ];
        let steering_rad = lqr.compute_lateral(lateral_state);

        let throttle = pid.compute_longitudinal(1.0 - 0.0);

        actuate.set_pwm_servo(steering_rad); // envoyé le pwn au servo
        actuate.set_pwm_esc(throttle); // pwm ESC

        // Watchdog -- paralyse le véhicule si on miss 3 lecture
        let loop_us = t0.elapsed().as_micros();
        if loop_us > CONTROL_PERIOD.as_micros() {
            if watchdog.miss() >= WATCHDOG_MAX_MISSED {
                error!("car: watchdog — {WATCHDOG_MAX_MISSED} overruns — safe state");
                actuate.safe_state();
            } else {
                watchdog.kick();
            }
        }

        // ── Logging (every 10 ticks = 100 ms) ────────────────────
        tick += 1;
        if tick % 10 == 0 {
            tracing::info!(
                tick,
                loop_us,
                uptime_s      = start.elapsed().as_secs(),
                steering      = format!("{:.3}", steering_rad),
                throttle      = format!("{:.3}", throttle),
                lidar_pts     = snap.lidar.points.len(),
                lidar_front_m = snap.lidar.nearest_in_arc(0.0, 30.0)
                                    .map_or(-1.0, |p| p.dist_m),
                sonar         = format!(
                                    "{:.2}/{:.2}/{:.2}",
                                    snap.sonar_m[0], snap.sonar_m[1], snap.sonar_m[2]
                                ),
                gz_rad_s      = format!("{:.3}", snap.imu.gz),
                "tick"
            );
        }
    }

    Ok(())
}

// ── Tracing ───────────────────────────────────────────────────────────────────

fn init_tracing() -> anyhow::Result<()> {
    use tracing_subscriber::prelude::*;

    let fmt = tracing_subscriber::fmt::layer()
        .with_filter(
            tracing_subscriber::EnvFilter::from_env("RUST_LOG")
                .add_directive("car=info".parse().unwrap()),
        );

    if std::env::var("JOURNAL_STREAM").is_ok() {
        // Running under systemd — also write structured fields to journald
        let jd = tracing_journald::layer()?;
        tracing::subscriber::set_global_default(
            tracing_subscriber::registry().with(fmt).with(jd),
        )?;
    } else {
        // Local terminal / SSH session
        tracing::subscriber::set_global_default(
            tracing_subscriber::registry().with(fmt),
        )?;
    }

    Ok(())
}
