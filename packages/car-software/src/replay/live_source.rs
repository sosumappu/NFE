use anyhow::Result;
use std::sync::{
    atomic::{AtomicBool, AtomicU32, Ordering},
    mpsc, Arc,
};
/// replay/live_source.rs — Live SensorSource backed by SharedState
///
/// Implements `SensorSource` on top of the existing `SharedState` + thread
/// architecture.  Optionally forwards every sensor update to a `Recorder`
/// so that sessions can be replayed offline without any changes to the
/// sensor threads themselves.
///
/// Recording interception
/// ──────────────────────
/// `RecordingSharedState` wraps `SharedState` and intercepts every `update_*`
/// call.  It serializes the frame and sends it to the Recorder's channel
/// before (transparently) delegating to the inner SharedState.
///
/// This means sensor threads do not need to know about recording at all.
/// The factory in main.rs decides whether to wrap or not.
use std::time::Duration;
use tracing::debug;

use crate::hal::{SensorFrame, SensorSource, TimestampedFrame};
use crate::state::{SensorSnapshot, SensorStateWriter, SharedState};
use crate::types::{ImuSample, LidarCloud};

// ── LiveSensorSource ───────────────────────────────────────────────────────

/// Wraps `SharedState` and implements `SensorSource` for the control loop.
/// Polls at the control-loop tick rate (100 Hz).
pub struct LiveSensorSource {
    state: Arc<SharedState>,
}

impl LiveSensorSource {
    pub fn new(state: Arc<SharedState>) -> Self {
        Self { state }
    }
}

impl SensorSource for LiveSensorSource {
    fn next_snapshot(&mut self) -> Result<SensorSnapshot> {
        Ok(self.state.snapshot())
    }
}

impl SensorStateWriter for RecordingSharedState {
    fn update_lidar(&self, cloud: LidarCloud) {
        self.update_lidar(cloud);
    }
    fn update_imu(&self, sample: ImuSample) {
        self.update_imu(sample);
    }
    fn update_sonar(&self, slot: usize, dist_m: f32) {
        self.update_sonar(slot, dist_m);
    }
    fn is_shutdown(&self) -> bool {
        self.inner.is_shutdown()
    }
    fn sensor_fault(&self) -> &AtomicBool {
        &self.inner.sensor_fault
    }
    fn set_shutdown(&self) {
        self.inner.shutdown.store(true, Ordering::Relaxed);
    }
}

pub struct RecordingSharedState {
    inner: Arc<SharedState>,
    tx: mpsc::SyncSender<TimestampedFrame>,
}

impl RecordingSharedState {
    pub fn new(inner: Arc<SharedState>, tx: mpsc::SyncSender<TimestampedFrame>) -> Arc<Self> {
        Arc::new(Self { inner, tx })
    }

    /// Expose inner SharedState so sensor threads can call update_* normally
    /// via the newtype's delegated methods below.
    pub fn inner(&self) -> &Arc<SharedState> {
        &self.inner
    }

    fn record(&self, frame: SensorFrame) {
        use std::time::{SystemTime, UNIX_EPOCH};
        let ts_us = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_micros() as u64;

        // Non-blocking send: if the recorder channel is full we drop the frame
        // rather than blocking the sensor thread (back-pressure safety).
        if self.tx.try_send(TimestampedFrame { ts_us, frame }).is_err() {
            debug!("recorder: channel full — frame dropped");
        }
    }

    // ── Delegated sensor writer API ────────────────────────────────────────

    pub fn update_lidar(&self, cloud: LidarCloud) {
        self.record(SensorFrame::Lidar(cloud.clone()));
        self.inner.update_lidar(cloud);
    }

    pub fn update_imu(&self, sample: ImuSample) {
        self.record(SensorFrame::Imu(sample));
        self.inner.update_imu(sample);
    }

    pub fn update_sonar(&self, slot: usize, dist_m: f32) {
        // We record sonar as a triplet; read the other two from current state
        // so the frame is always complete.
        let snap = self.inner.snapshot();
        let mut sonar = snap.sonar_m;
        sonar[slot] = dist_m;
        self.record(SensorFrame::Sonar {
            front: sonar[0],
            left: sonar[1],
            right: sonar[2],
        });
        self.inner.update_sonar(slot, dist_m);
    }

    // ── Delegated reader + control API ────────────────────────────────────

    pub fn snapshot(&self) -> SensorSnapshot {
        self.inner.snapshot()
    }
    pub fn is_shutdown(&self) -> bool {
        self.inner.is_shutdown()
    }
}
