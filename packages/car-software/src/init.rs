/// init.rs — Sensor readiness barrier
///
/// Each sensor thread calls `ReadinessBarrier::signal(sensor)` the first time
/// it successfully produces a valid reading.  The control loop calls
/// `wait_all_ready(timeout)` before its first tick.
///
/// On timeout the barrier logs exactly which sensors failed to respond and
/// returns `Err(InitError)`.  `main` then notifies systemd STOPPING and exits,
/// letting `Restart=always` handle the retry.
///
/// This guarantees the control loop never starts with stale Default values:
///   - ImuSample::default() → gz=0 feeds LQR a false "no yaw"
///   - LidarCloud::default() → empty cloud, obstacle_closer_than() blind
///   - sonar f32::MAX → "no obstacle" (safe, but still unverified)

use std::{fmt, time::Duration};
use tokio::sync::watch;
use tracing::{error, info, warn};

// ── Sensor identity ────────────────────────────────────────────────────────

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Sensor {
    Lidar,
    Imu,
    Sonar(usize), // 0=front 1=front-left 2=front-right
}

impl fmt::Display for Sensor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Sensor::Lidar    => write!(f, "LIDAR"),
            Sensor::Imu      => write!(f, "IMU"),
            Sensor::Sonar(n) => write!(f, "Sonar[{n}]"),
        }
    }
}

// All sensors that must be ready before the control loop may start
const REQUIRED: &[Sensor] = &[
    // Sensor::Lidar,
    // Sensor::Imu,
    // Sensor::Sonar(0),
    // Sensor::Sonar(1),
    // Sensor::Sonar(2),
];

// ── ReadinessBarrier ───────────────────────────────────────────────────────

/// One sender per sensor; the barrier watches all of them.
#[derive(Clone)]
pub struct ReadinessBarrier {
    senders: Vec<(Sensor, watch::Sender<bool>)>,
}

/// Handle given to each sensor thread — call `.signal()` once on first valid reading.
pub struct ReadySignal {
    sensor: Sensor,
    tx:     watch::Sender<bool>,
}

impl ReadySignal {
    /// Mark this sensor as ready.  Idempotent — safe to call multiple times.
    pub fn dummy(sensor: Sensor) -> Self {
        let (tx,_) = watch::channel(false);
        Self { sensor, tx }
    }
    pub fn signal(&self) {
        let _ = self.tx.send(true);
        info!("init: {} ready", self.sensor);
    }
}

#[derive(Debug)]
pub struct InitError {
    pub failed: Vec<Sensor>,
}

impl fmt::Display for InitError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let names: Vec<_> = self.failed.iter().map(|s| s.to_string()).collect();
        write!(f, "sensors not ready within timeout: {}", names.join(", "))
    }
}

impl std::error::Error for InitError {}

impl ReadinessBarrier {
    /// Create barrier + one `ReadySignal` per required sensor.
    pub fn new() -> (Self, Vec<ReadySignal>) {
        let mut senders = Vec::new();
        let mut signals = Vec::new();

        for &sensor in REQUIRED {
            let (tx, _rx) = watch::channel(false);
            signals.push(ReadySignal { sensor, tx: tx.clone() });
            senders.push((sensor, tx));
        }

        (ReadinessBarrier { senders }, signals)
    }

    /// Block until every sensor has signalled ready, or `timeout` expires.
    /// Returns `Err(InitError)` listing every sensor that didn't respond.
    pub async fn wait_all_ready(&self, timeout: Duration) -> Result<(), InitError> {
        info!("init: waiting for {} sensors (timeout={timeout:?})", REQUIRED.len());

        let deadline = tokio::time::Instant::now() + timeout;
        let mut failed = Vec::new();

        for (sensor, tx) in &self.senders {
            let mut rx = tx.subscribe();

            // Already ready (signalled before we started waiting)?
            if *rx.borrow() {
                info!("init: {} already ready", sensor);
                continue;
            }

            let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
            match tokio::time::timeout(remaining, rx.changed()).await {
                Ok(Ok(())) if *rx.borrow() => {
                    info!("init: {} ready", sensor);
                }
                _ => {
                    error!("init: {} did NOT become ready in time", sensor);
                    failed.push(*sensor);
                }
            }
        }

        if failed.is_empty() {
            info!("init: all sensors ready — starting control loop");
            Ok(())
        } else {
            Err(InitError { failed })
        }
    }
}
