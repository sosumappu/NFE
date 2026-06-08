/// state.rs — Shared sensor state between gatherer threads and the control loop.
///
/// Gatherer threads hold an `Arc<SharedState>` and call `update_*`.
/// The control loop calls `snapshot()` once per 10 ms tick to get a
/// consistent, allocation-minimised copy of all sensor data.
///
/// Synchronisation strategy
/// ─────────────────────────
///   LidarCloud   — parking_lot::RwLock  (variable-length Vec of points)
///   ImuSample    — parking_lot::RwLock  (14-byte struct, written at 500 Hz)
///   sonar[3]     — AtomicU32 per slot   (f32 bits; lock-free, written at ~33 Hz each)
///   flags        — AtomicBool           (sensor_fault, shutdown)
///
/// parking_lot::RwLock is chosen over std because:
///   - Does not poison on panic (safe in RT context)
///   - Fair queueing (no writer starvation under high read load)
///   - ~5 ns lock/unlock on aarch64 with no contention
use std::sync::{
    atomic::{AtomicBool, AtomicU32, Ordering},
    Arc,
};

use arc_swap::ArcSwap;
use parking_lot::RwLock;

use crate::types::{ImuSample, LidarCloud};

// in state.rs
pub trait SensorStateWriter: Send + Sync + 'static {
    fn update_lidar(&self, cloud: LidarCloud);
    fn update_imu(&self, sample: ImuSample);
    fn update_sonar(&self, slot: usize, dist_m: f32);
    fn is_shutdown(&self) -> bool;
    fn sensor_fault(&self) -> &AtomicBool;
    // in state.rs, add to the trait:
    fn set_shutdown(&self);
}

// ── Snapshot (what the control loop works with) ────────────────────────────

#[derive(Clone, Debug)]
pub struct SensorSnapshot {
    pub lidar: Arc<LidarCloud>,
    pub imu: ImuSample,
    /// distances in metres for [front, front-left, front-right]
    /// f32::MAX = no obstacle / out of range
    pub sonar_m: [f32; 3],
    pub sensor_fault: bool,
}

impl SensorSnapshot {
    /// Nearest sonar reading across all three sensors.
    pub fn sonar_min(&self) -> f32 {
        self.sonar_m.iter().cloned().fold(f32::MAX, f32::min)
    }

    /// True if any sensor (sonar or LIDAR front arc) detects an obstacle
    /// closer than `threshold_m`.
    pub fn obstacle_closer_than(&self, threshold_m: f32) -> bool {
        if self.sonar_min() < threshold_m {
            return true;
        }
        self.lidar
            .nearest_in_arc(0.0, 30.0) // ±30° front cone
            .map_or(false, |p| p.dist_m < threshold_m)
    }
}

// ── SharedState ────────────────────────────────────────────────────────────

pub struct SharedState {
    lidar: ArcSwap<LidarCloud>,
    imu: RwLock<ImuSample>,
    /// f32::to_bits() stored atomically; index = SonarConfig::slot
    sonar_bits: [AtomicU32; 3],
    pub sensor_fault: AtomicBool,
    pub shutdown: AtomicBool,
}

impl SensorStateWriter for SharedState {
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
        self.is_shutdown()
    }
    fn sensor_fault(&self) -> &AtomicBool {
        &self.sensor_fault
    }
    fn set_shutdown(&self) {
        self.shutdown.store(true, Ordering::Relaxed);
    }
}

impl SharedState {
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            lidar: ArcSwap::from_pointee(LidarCloud::default()),
            imu: RwLock::new(ImuSample::default()),
            sonar_bits: [
                AtomicU32::new(f32::MAX.to_bits()),
                AtomicU32::new(f32::MAX.to_bits()),
                AtomicU32::new(f32::MAX.to_bits()),
            ],
            sensor_fault: AtomicBool::new(false),
            shutdown: AtomicBool::new(false),
        })
    }

    // ── Writer API ──────────────────────────────────────────────

    pub fn update_lidar(&self, cloud: LidarCloud) {
        self.lidar.store(Arc::new(cloud));
    }

    pub fn update_imu(&self, sample: ImuSample) {
        *self.imu.write() = sample;
    }

    /// `slot` must be 0, 1, or 2 — matches `SonarConfig::slot`.
    pub fn update_sonar(&self, slot: usize, dist_m: f32) {
        self.sonar_bits[slot].store(dist_m.to_bits(), Ordering::Relaxed);
    }

    // ── Reader API (control loop) ────────────────────────────────
    pub fn snapshot(&self) -> SensorSnapshot {
        SensorSnapshot {
            lidar: self.lidar.load_full(),
            imu: *self.imu.read(),
            sonar_m: [
                f32::from_bits(self.sonar_bits[0].load(Ordering::Relaxed)),
                f32::from_bits(self.sonar_bits[1].load(Ordering::Relaxed)),
                f32::from_bits(self.sonar_bits[2].load(Ordering::Relaxed)),
            ],
            sensor_fault: self.sensor_fault.load(Ordering::Relaxed),
        }
    }

    pub fn is_shutdown(&self) -> bool {
        self.shutdown.load(Ordering::Relaxed)
    }
}
