/// sensors/sonar.rs — 3× HC-SR04 ultrasonic rangefinders
///
/// Physical wiring (BCM pin numbers):
///   Sensor 0 (front)       TRIG=23  ECHO=24
///   Sensor 1 (front-left)  TRIG=25  ECHO=8
///   Sensor 2 (front-right) TRIG=7   ECHO=1
///
/// Linux / rppal constraint
/// ─────────────────────────
/// We cannot receive hardware IRQs in userspace.  Instead we use rppal's
/// `set_async_interrupt` which backs the ECHO pin with a kernel poll
/// (epoll on /sys/class/gpio) and calls our closure on each edge transition.
/// Latency introduced by this path is typically 20-100 µs on RPi5 — adequate
/// for HC-SR04 whose measurement itself has ±1 cm (~58 µs/cm).
///
/// Measurement sequence per sensor (runs in a dedicated OS thread)
/// ───────────────────────────────────────────────────────────────
///   1. Pull TRIG high for 10 µs  (trigger pulse)
///   2. Wait for ECHO rising edge  → record t_rise
///   3. Wait for ECHO falling edge → record t_fall
///   4. dist_m = (t_fall - t_rise).as_secs_f32() * SOUND_SPEED_M_S / 2.0
///   5. Clamp to [DIST_MIN_M, DIST_MAX_M]; out-of-range = no obstacle detected
///   6. Write result into SharedState
///   7. Sleep MEASURE_INTERVAL before next trigger
///
/// We stagger the trigger timing across the 3 sensors to prevent crosstalk
/// (sensor 0 triggers at t=0, sensor 1 at t+33ms, sensor 2 at t+66ms).
///
/// Safety
/// ──────
/// Each sensor thread is independent; a stuck ECHO (no return) times out
/// after ECHO_TIMEOUT and records f32::MAX (no obstacle) with a warning.

use std::{
    sync::{
        atomic::{AtomicU32, Ordering},
        Arc,
    },
    thread,
    time::{Duration, Instant},
};

use rppal::gpio::{Gpio, InputPin, Level, OutputPin, Trigger};
use tracing::{debug, error, info, warn};

use crate::init::ReadySignal;
use crate::state::SharedState;

// ═══════════════════════════════════════════════════════════════════════════
//  TUNE THESE CONSTANTS FOR YOUR SETUP
// ═══════════════════════════════════════════════════════════════════════════

pub const SOUND_SPEED_M_S: f32 = 343.0;   // m/s at ~20°C; adjust for ambient temp

/// HC-SR04 trigger pulse width (spec: ≥10 µs)
pub const TRIG_PULSE_US: u64 = 15;

/// How long to wait for the ECHO rising edge before declaring a timeout.
pub const ECHO_TIMEOUT: Duration = Duration::from_millis(30);

/// Measurement cycle period per sensor.  At 30 ms you get ~33 Hz per sensor.
pub const MEASURE_INTERVAL: Duration = Duration::from_millis(30);

/// Valid distance range.  HC-SR04 spec: 2 cm – 4 m.
pub const DIST_MIN_M: f32 = 0.02;
pub const DIST_MAX_M: f32 = 4.0;

// ═══════════════════════════════════════════════════════════════════════════

/// Sensor descriptor — pin numbers and which SharedState slot to write.
#[derive(Clone, Copy)]
pub struct SonarConfig {
    pub label:    &'static str,
    pub trig_bcm: u8,
    pub echo_bcm: u8,
    /// Index into SharedState::sonar_slots (0, 1, 2)
    pub slot:     usize,
    /// Stagger delay before first trigger so sensors don't interfere
    pub startup_delay: Duration,
}

/// Default wiring: 23/24, 25/8, 7/1
pub const SONAR_SENSORS: [SonarConfig; 3] = [
    SonarConfig {
        label: "front",       trig_bcm: 23, echo_bcm: 24, slot: 0,
        startup_delay: Duration::from_millis(0),
    },
    SonarConfig {
        label: "front-left",  trig_bcm: 25, echo_bcm: 8,  slot: 1,
        startup_delay: Duration::from_millis(10),
    },
    SonarConfig {
        label: "front-right", trig_bcm: 7,  echo_bcm: 1,  slot: 2,
        startup_delay: Duration::from_millis(20),
    },
];

// ── Thread entry point ──────────────────────────────────────────────────────

/// Spawn one OS thread per sensor.  Returns all three handles.
pub fn spawn_all(
    state: Arc<SharedState>,
    gpio: &Gpio,
    ready_signals: Vec<ReadySignal>,
) -> Vec<thread::JoinHandle<()>> {
    SONAR_SENSORS
        .iter()
        .zip(ready_signals.into_iter())
        .map(|(cfg, ready)| {
            let state  = state.clone();
            let cfg    = *cfg;
            let trig   = gpio.get(cfg.trig_bcm)
                .unwrap_or_else(|e| panic!("sonar {}: TRIG pin {}: {e}", cfg.label, cfg.trig_bcm))
                .into_output();
            let echo   = gpio.get(cfg.echo_bcm)
                .unwrap_or_else(|e| panic!("sonar {}: ECHO pin {}: {e}", cfg.label, cfg.echo_bcm))
                .into_input();

            thread::Builder::new()
                .name(format!("sonar-{}", cfg.label))
                .stack_size(128 * 1024)
                .spawn(move || run(state, cfg, trig, echo, ready))
                .expect("failed to spawn sonar thread")
        })
        .collect()
}

fn run(
    state: Arc<SharedState>,
    cfg: SonarConfig,
    mut trig: OutputPin,
    mut echo: InputPin,
    ready: ReadySignal,
) {
    info!(
        "sonar[{}]: TRIG=BCM{} ECHO=BCM{} slot={}",
        cfg.label, cfg.trig_bcm, cfg.echo_bcm, cfg.slot
    );

    if !cfg.startup_delay.is_zero() {
        thread::sleep(cfg.startup_delay);
    }

    let rise_us: Arc<AtomicU32> = Arc::new(AtomicU32::new(0));
    let fall_us: Arc<AtomicU32> = Arc::new(AtomicU32::new(0));
    let rise_us2 = rise_us.clone();
    let fall_us2 = fall_us.clone();

    let t0   = Instant::now();
    let t0_2 = t0;

    echo.set_async_interrupt(Trigger::Both, move |level| {
        let elapsed_us = t0_2.elapsed().as_micros() as u32;
        match level {
            Level::High => rise_us2.store(elapsed_us, Ordering::Release),
            Level::Low  => fall_us2.store(elapsed_us, Ordering::Release),
        }
    })
    .expect("sonar: failed to set async interrupt");

    let mut ready = Some(ready); // consumed on first valid reading

    loop {
        if state.is_shutdown() { break; }

        let dist_m = measure(&mut trig, &rise_us, &fall_us, &t0, &cfg);
        state.update_sonar(cfg.slot, dist_m);

        // Signal readiness on first ping that returned (valid or out-of-range —
        // both mean the hardware is alive; timeout means it's not)
        if ready.is_some() && dist_m < f32::MAX {
            if let Some(r) = ready.take() { r.signal(); }
        }

        debug!("sonar[{}]: {:.3} m", cfg.label,
               if dist_m >= DIST_MAX_M { f32::INFINITY } else { dist_m });

        thread::sleep(MEASURE_INTERVAL);
    }

    echo.clear_async_interrupt().ok();
    info!("sonar[{}]: thread exiting", cfg.label);
}

/// Fire one trigger pulse and wait for the ECHO edges.
/// Returns distance in metres, or DIST_MAX_M on timeout/invalid.
fn measure(
    trig:    &mut OutputPin,
    rise_us: &Arc<AtomicU32>,
    fall_us: &Arc<AtomicU32>,
    t0:      &Instant,
    cfg:     &SonarConfig,
) -> f32 {
    // Snapshot current edge timestamps so we can detect new edges
    let prev_rise = rise_us.load(Ordering::Relaxed);
    let prev_fall = fall_us.load(Ordering::Relaxed);

    // ── 1. Trigger pulse ─────────────────────────────────────────
    trig.set_high();
    spin_sleep_us(TRIG_PULSE_US);
    trig.set_low();

    let trigger_us = t0.elapsed().as_micros() as u32;

    // ── 2. Wait for ECHO rising edge ─────────────────────────────
    let deadline = Instant::now() + ECHO_TIMEOUT;
    let rise = loop {
        let r = rise_us.load(Ordering::Acquire);
        if r != prev_rise && r >= trigger_us {
            break r;
        }
        if Instant::now() >= deadline {
            warn!("sonar[{}]: ECHO rising edge timeout", cfg.label);
            return DIST_MAX_M;
        }
        thread::yield_now();
    };

    // ── 3. Wait for ECHO falling edge ────────────────────────────
    let deadline = Instant::now() + ECHO_TIMEOUT;
    let fall = loop {
        let f = fall_us.load(Ordering::Acquire);
        if f != prev_fall && f > rise {
            break f;
        }
        if Instant::now() >= deadline {
            warn!("sonar[{}]: ECHO falling edge timeout", cfg.label);
            return DIST_MAX_M;
        }
        thread::yield_now();
    };

    // ── 4. Distance ───────────────────────────────────────────────
    let echo_us = (fall - rise) as f32;
    let dist_m  = echo_us * 1e-6 * SOUND_SPEED_M_S / 2.0;

    if dist_m < DIST_MIN_M || dist_m > DIST_MAX_M {
        return DIST_MAX_M; // out of range — no credible obstacle
    }

    dist_m
}

/// Busy-wait for a precise short delay (< 1 ms).
/// `thread::sleep` has OS scheduler granularity (~1 ms) — too coarse for
/// the 10-15 µs trigger pulse.
#[inline]
fn spin_sleep_us(us: u64) {
    let deadline = Instant::now() + Duration::from_micros(us);
    while Instant::now() < deadline {
        std::hint::spin_loop();
    }
}
