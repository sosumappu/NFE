/// hal.rs — Hardware Abstraction Layer traits
///
/// Every sensor and actuator is accessed through these traits. The rest of the
/// codebase never imports rppal, serialport, or any hardware crate directly —
/// only the concrete implementations in sensors/ and control/ do.
///
/// This gives us:
///   • Dry-run / simulation with zero code changes in the control loop
///   • Replay: a `ReplaySensorSource` that feeds recorded frames through the
///     same trait, so the control loop is exercised identically offline
///   • Decorator pattern: `LoggingActuator` wraps any `ActuatorSink` and
///     traces every command — useful for both dry-run and live debugging
use crate::state::SensorSnapshot;
use crate::types::{ImuSample, LidarCloud};
use anyhow::Result;

// ── Sensor side ────────────────────────────────────────────────────────────

/// Implemented by `LiveSensorState` (backed by sensor threads + SharedState)
/// and `ReplaySensorSource` (backed by a recorded session file).
pub trait SensorSource: Send {
    /// Blocking: waits until the next control-loop tick's data is ready, then
    /// returns a consistent snapshot of all sensor readings at that instant.
    fn next_snapshot(&mut self) -> Result<SensorSnapshot>;

    /// True when the source has no more data (always false for live sources).
    fn is_exhausted(&self) -> bool {
        false
    }
}

// ── Actuator side ─────────────────────────────────────────────────────────

/// Implemented by `RealActuator` (rppal PWM) and `DryRunActuator`.
/// A `LoggingActuator` decorator wraps either and adds tracing.
pub trait ActuatorSink: Send {
    /// Throttle in [-1.0, +1.0]. Implementations must clamp internally.
    fn set_throttle(&mut self, throttle: f32) -> Result<()>;

    /// Steering angle in radians. Positive = left.
    fn set_steering(&mut self, angle_rad: f32) -> Result<()>;

    /// Immediately centre steering and zero throttle (ESTOP / shutdown).
    fn safe_state(&mut self) -> Result<()>;

    /// Human-readable label for log messages ("real", "dry-run", etc.)
    fn label(&self) -> &'static str;
}

// ── Sensor record / replay ────────────────────────────────────────────────

/// A single timestamped sensor event, written to disk during a live run and
/// read back during replay. `bincode` keeps it compact and fast.
#[derive(serde::Serialize, serde::Deserialize, Debug, Clone)]
pub enum SensorFrame {
    Lidar(LidarCloud),
    Imu(ImuSample),
    Sonar { front: f32, left: f32, right: f32 },
}

#[derive(serde::Serialize, serde::Deserialize, Debug, Clone)]
pub struct TimestampedFrame {
    /// Microseconds since UNIX epoch
    pub ts_us: u64,
    pub frame: SensorFrame,
}
