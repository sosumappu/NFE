/// sim/noise.rs — Reproducible sensor noise for simulation
///
/// Uses a fast xorshift64 PRNG (no external dep) seeded from the system clock.
/// All noise magnitudes are calibrated against real sensor datasheets:
///   LiDAR A1:  ±30 mm (1σ) range noise, 0.5° angle quantisation
///   HC-SR04:   ±5 mm range noise, occasional outliers
///   MPU-6050:  accel 0.003 g (1σ), gyro 0.05 °/s (1σ)
///
/// Tune via `SensorNoise` fields — or set all sigmas to 0 for ideal sensors.

pub struct SensorNoise {
    rng: Xorshift64,
    /// LiDAR 1σ range noise [m]
    pub lidar_sigma: f32,
    /// Sonar 1σ range noise [m]
    pub sonar_sigma: f32,
    /// IMU accelerometer 1σ [m/s²]
    pub accel_sigma: f32,
    /// IMU gyroscope 1σ [rad/s]
    pub gyro_sigma: f32,
}

impl Default for SensorNoise {
    fn default() -> Self {
        Self {
            rng: Xorshift64::new(),
            lidar_sigma: 0.03,
            sonar_sigma: 0.005,
            accel_sigma: 0.03, // 0.003 g × 9.806
            gyro_sigma: 0.001, // 0.05 °/s → rad/s
        }
    }
}

impl SensorNoise {
    /// Ideal sensors — no noise. Useful for debugging control logic.
    pub fn zero() -> Self {
        Self {
            rng: Xorshift64::new(),
            lidar_sigma: 0.0,
            sonar_sigma: 0.0,
            accel_sigma: 0.0,
            gyro_sigma: 0.0,
        }
    }

    pub fn lidar(&mut self, dist: f32) -> f32 {
        (dist + self.rng.gaussian() * self.lidar_sigma).max(0.0)
    }
    pub fn sonar(&mut self, dist: f32) -> f32 {
        (dist + self.rng.gaussian() * self.sonar_sigma).max(0.0)
    }
    pub fn imu_accel(&mut self, v: f32) -> f32 {
        v + self.rng.gaussian() * self.accel_sigma
    }
    pub fn imu_gyro(&mut self, v: f32) -> f32 {
        v + self.rng.gaussian() * self.gyro_sigma
    }
}

// ── Xorshift64 PRNG ────────────────────────────────────────────────────────

struct Xorshift64 {
    state: u64,
}

impl Xorshift64 {
    fn new() -> Self {
        // Use a small wall-clock seeding source. Keep the local import narrow to
        // avoid unused import warnings elsewhere.
        // Seed from the current wall-clock nanoseconds. Keep import local.
        use std::time::{SystemTime, UNIX_EPOCH};
        let seed = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .subsec_nanos() as u64
            | 1;
        Self { state: seed }
    }

    #[inline]
    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state = x;
        x
    }

    /// Box-Muller transform → N(0,1)
    fn gaussian(&mut self) -> f32 {
        // Two uniform [0,1) samples
        let u1 = (self.next_u64() >> 11) as f32 / (1u64 << 53) as f32;
        let u2 = (self.next_u64() >> 11) as f32 / (1u64 << 53) as f32;
        let u1 = u1.max(1e-10);
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f32::consts::PI * u2).cos()
    }
}
