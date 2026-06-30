#![cfg(target_os = "linux")]
/// sensors/sonar.rs — 3× HC-SR04 with per-sensor graceful degradation
///
/// `try_spawn_all` attempts to grab each sensor's GPIO pins independently.
/// If a pin cannot be opened (sensor not connected, pin in use), that slot is
/// skipped and SharedState keeps its initial f32::MAX value ("no obstacle").
/// The other sonars continue to operate normally.
use std::{
    sync::{
        atomic::{AtomicU32, Ordering},
        Arc,
    },
    thread,
    time::{Duration, Instant},
};

use rppal::gpio::{Event, Gpio, InputPin, Level, OutputPin, Trigger};
use tracing::{debug, info, warn};

use crate::init::ReadySignal;
use crate::state::{SensorStateWriter, SharedState};

// ═══════════════════════════════════════════════════════════════════════════
//  Constants
// ═══════════════════════════════════════════════════════════════════════════

pub const SOUND_SPEED_M_S: f32 = 343.0;
pub const TRIG_PULSE_US: u64 = 15;
pub const ECHO_TIMEOUT: Duration = Duration::from_millis(30);
pub const MEASURE_INTERVAL: Duration = Duration::from_millis(30);
pub const DIST_MIN_M: f32 = 0.02;
pub const DIST_MAX_M: f32 = 4.0;

// ═══════════════════════════════════════════════════════════════════════════
//  Sensor configuration
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Clone, Copy)]
pub struct SonarConfig {
    pub label: &'static str,
    pub trig_bcm: u8,
    pub echo_bcm: u8,
    pub slot: usize,
    pub startup_delay: Duration,
}

pub const SONAR_SENSORS: [SonarConfig; 3] = [
    SonarConfig {
        label: "front",
        trig_bcm: 23,
        echo_bcm: 24,
        slot: 0,
        startup_delay: Duration::from_millis(0),
    },
    SonarConfig {
        label: "front-left",
        trig_bcm: 25,
        echo_bcm: 8,
        slot: 1,
        startup_delay: Duration::from_millis(10),
    },
    SonarConfig {
        label: "front-right",
        trig_bcm: 7,
        echo_bcm: 1,
        slot: 2,
        startup_delay: Duration::from_millis(20),
    },
];

// ═══════════════════════════════════════════════════════════════════════════
//  Public spawn interface
// ═══════════════════════════════════════════════════════════════════════════

/// Attempt to spawn a thread for each sensor. Sensors whose GPIO pins cannot
/// be opened are silently skipped — their SharedState slot stays at f32::MAX.
pub fn try_spawn_all(
    state: Arc<dyn SensorStateWriter>,
    gpio: &Gpio,
    ready_signals: Vec<ReadySignal>,
) -> Vec<thread::JoinHandle<()>> {
    SONAR_SENSORS
        .iter()
        .zip(ready_signals.into_iter())
        .filter_map(|(cfg, ready)| {
            let trig = match gpio.get(cfg.trig_bcm) {
                Ok(p) => p.into_output(),
                Err(e) => {
                    warn!(
                        "sonar[{}]: cannot open TRIG pin BCM{} ({e}) — sensor skipped",
                        cfg.label, cfg.trig_bcm
                    );
                    ready.signal();
                    return None;
                }
            };
            let echo = match gpio.get(cfg.echo_bcm) {
                Ok(p) => p.into_input(),
                Err(e) => {
                    warn!(
                        "sonar[{}]: cannot open ECHO pin BCM{} ({e}) — sensor skipped",
                        cfg.label, cfg.echo_bcm
                    );
                    ready.signal();
                    return None;
                }
            };

            let state = state.clone();
            let cfg = *cfg;

            // ← was .expect(...) which panics the whole process
            match thread::Builder::new()
                .name(format!("sonar-{}", cfg.label))
                .stack_size(128 * 1024)
                .spawn(move || run(state, cfg, trig, echo, ready))
            {
                Ok(h) => Some(h),
                Err(e) => {
                    warn!(
                        "sonar[{}]: failed to spawn thread ({e}) — sensor skipped",
                        cfg.label
                    );
                    // ready was moved into the closure; signal already consumed.
                    // The slot stays at f32::MAX which is the safe default.
                    None
                }
            }
        })
        .collect()
}

// ═══════════════════════════════════════════════════════════════════════════
//  Thread body (unchanged from original)
// ═══════════════════════════════════════════════════════════════════════════

fn run(
    state: Arc<dyn SensorStateWriter>,
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
    let t0 = Instant::now();
    let t0_2 = t0;

    // ← was .expect(...) — now we degrade gracefully
    if let Err(e) = echo.set_async_interrupt(Trigger::Both, None, move |event: Event| {
        let us = t0_2.elapsed().as_micros() as u32;
        match event.trigger {
            Trigger::RisingEdge => rise_us2.store(us, Ordering::Release),
            Trigger::FallingEdge => fall_us2.store(us, Ordering::Release),
            _ => {}
        }
    }) {
        warn!(
            "sonar[{}]: set_async_interrupt failed ({e}) — sensor skipped",
            cfg.label
        );
        ready.signal(); // unblock the init barrier
        return; // thread exits cleanly, slot stays at f32::MAX
    }

    let mut ready = Some(ready);

    loop {
        if state.is_shutdown() {
            break;
        }

        let dist_m = measure(&mut trig, &rise_us, &fall_us, &t0, &cfg);
        state.update_sonar(cfg.slot, dist_m);

        if ready.is_some() && dist_m < f32::MAX {
            if let Some(r) = ready.take() {
                r.signal();
            }
        }

        debug!(
            "sonar[{}]: {:.3} m",
            cfg.label,
            if dist_m >= DIST_MAX_M {
                f32::INFINITY
            } else {
                dist_m
            }
        );
        thread::sleep(MEASURE_INTERVAL);
    }

    echo.clear_async_interrupt().ok();
    info!("sonar[{}]: thread exiting", cfg.label);
}

fn measure(
    trig: &mut OutputPin,
    rise_us: &Arc<AtomicU32>,
    fall_us: &Arc<AtomicU32>,
    t0: &Instant,
    cfg: &SonarConfig,
) -> f32 {
    let prev_rise = rise_us.load(Ordering::Relaxed);
    let prev_fall = fall_us.load(Ordering::Relaxed);

    trig.set_high();
    spin_sleep_us(TRIG_PULSE_US);
    trig.set_low();

    let trigger_us = t0.elapsed().as_micros() as u32;

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

    let dist_m = (fall - rise) as f32 * 1e-6 * SOUND_SPEED_M_S / 2.0;
    if dist_m < DIST_MIN_M || dist_m > DIST_MAX_M {
        DIST_MAX_M
    } else {
        dist_m
    }
}

#[inline]
fn spin_sleep_us(us: u64) {
    let deadline = Instant::now() + Duration::from_micros(us);
    while Instant::now() < deadline {
        std::hint::spin_loop();
    }
}
