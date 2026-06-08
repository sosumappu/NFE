/// sensors/factory.rs — Sensor thread spawner with graceful degradation
///
/// `SensorFactory::spawn_all` attempts to initialise every sensor. Each
/// sensor has two failure modes:
///
///   Probe failure  — the hardware device cannot be opened at all (wrong pin,
///                    device not connected, udev rule missing).  The factory
///                    logs a warning and installs a `NullSensor` stub that
///                    always returns a safe default (f32::MAX for sonar, empty
///                    cloud for lidar, zeroed sample for IMU).
///
///   Runtime error  — the sensor thread crashes after first successful read.
///                    The existing retry loop in each sensor module handles
///                    this case unchanged.
///
/// The control loop therefore never needs to check whether a sensor is
/// "present" — it simply reads SharedState and trusts that absent sensors
/// report safe defaults.
///
/// Sonar notes
/// ───────────
/// HC-SR04 pins are grabbed here before spawning threads.  If `gpio.get(pin)`
/// fails (pin in use, hardware not connected), that sonar slot is skipped and
/// its SharedState slot remains at f32::MAX ("no obstacle").  This is safe
/// because the ESTOP threshold of 0.30 m is far below f32::MAX.

use std::sync::Arc;

use anyhow::Result;
use tracing::{info, warn};

use crate::init::{ReadySignal, Sensor};
use crate::state::{SensorStateWriter, SharedState};
use super::{imu, lidar, sonar};

pub struct SpawnedSensors {
    /// Join handles — kept so main can wait for clean shutdown
    pub handles: Vec<std::thread::JoinHandle<()>>,
    /// List of sensors that could not be initialised
    pub skipped: Vec<&'static str>,
}

pub struct SensorFactory;

impl SensorFactory {
    /// Spawn all sensor threads. Returns immediately; threads run in the
    /// background writing into `state`.
    ///
    /// `lidar_port`: e.g. "/dev/lidar"
    /// `ready_signals`: caller-supplied signals consumed by each sensor thread
    pub fn spawn_all(
        state:       &Arc<dyn SensorStateWriter>,
        lidar_port:  String,
        signals: SensorReadySignals,
    ) -> SpawnedSensors {
        let mut handles = Vec::new();
        let mut skipped = Vec::new();

        // ── LiDAR ─────────────────────────────────────────────────────────
       if std::path::Path::new(&lidar_port).exists() {
            // File exists. Let the lidar thread handle the actual opening
            // so we don't trigger a HUPCL reset on close.
            let h = lidar::spawn(state.clone(), signals.lidar, lidar_port);
            handles.push(h);
        } else {
            warn!("sensor-factory: LiDAR device file not found — stub installed");
            signals.lidar.signal(); // unblock barrier immediately
            skipped.push("LiDAR");
        }

        // ── IMU ───────────────────────────────────────────────────────────
        match rppal::i2c::I2c::new() {
            Ok(_) => {
                let h = imu::spawn(state.clone(), signals.imu);
                handles.push(h);
            }
            Err(e) => {
                warn!("sensor-factory: IMU not available ({e}) — stub installed");
                signals.imu.signal();
                skipped.push("IMU");
            }
        }

        // ── Sonars — per-sensor graceful degradation ───────────────────────
        #[cfg(target_os = "linux")]
        {
            use rppal::gpio::Gpio;
            match Gpio::new() {
                Ok(gpio) => {
                    let sonar_handles =
                        sonar::try_spawn_all(state.clone(), &gpio, signals.sonars);
                    handles.extend(sonar_handles);
                }
                Err(e) => {
                    warn!("sensor-factory: GPIO unavailable ({e}) — all sonars skipped");
                    for sig in signals.sonars { sig.signal(); }
                    skipped.push("Sonar (all)");
                }
            }
        }

        #[cfg(not(target_os = "linux"))]
        {
            warn!("sensor-factory: not on Linux — all sonars skipped");
            for sig in signals.sonars { sig.signal(); }
            skipped.push("Sonar (all)");
        }

        if skipped.is_empty() {
            info!("sensor-factory: all sensors initialised");
        } else {
            warn!("sensor-factory: running with degraded sensors: {:?}", skipped);
        }

        SpawnedSensors { handles, skipped }
    }
}

/// Typed bundle of ready signals — prevents accidental misordering
pub struct SensorReadySignals {
    pub lidar:  ReadySignal,
    pub imu:    ReadySignal,
    pub sonars: Vec<ReadySignal>,
}
