//! Extended Kalman Filter for planar pose + velocity + IMU bias estimation.
//!
//! State vector (8):
//!   0: x       world position [m]
//!   1: y       world position [m]
//!   2: yaw     heading [rad]
//!   3: vx      body-frame longitudinal velocity [m/s]
//!   4: vy      body-frame lateral velocity [m/s]
//!   5: b_ax    accel bias, body x [m/s^2]
//!   6: b_ay    accel bias, body y [m/s^2]
//!   7: b_gz    gyro bias, z [rad/s]
//!
//! The bias states are what let the filter track slow IMU drift instead of
//! baking a one-shot calibration into pose propagation.
//!
//! `predict` runs at IMU rate from raw (biased) accel/gyro. `correct_pose`
//! folds in a world-frame pose measurement from scan-matching against the map.
//! `correct_zero_vy`
//! is a pseudo-measurement that curbs lateral-velocity drift on the corridor
//! where side-slip is near zero at the speeds we run.
//!
//! Covariance is a dense 8x8. At 8 states the O(n^3) update is a few hundred
//! flops — negligible next to LIDAR processing — so it stays dense and
//! readable rather than exploiting sparsity.

#![allow(clippy::needless_range_loop)]

use nfe_core::estimation::{ImuSample, PoseMeasurement, StateEstimate, StateEstimator};
use nfe_core::params::Tunable;
use nfe_core::{wrap_angle, MotionState, Pose2};

pub const N: usize = 8;
type Vec8 = [f32; N];
type Mat8 = [[f32; N]; N];

// State indices.
const X: usize = 0;
const Y: usize = 1;
const YAW: usize = 2;
const VX: usize = 3;
const VY: usize = 4;
const BAX: usize = 5;
const BAY: usize = 6;
const BGZ: usize = 7;

/// Tunable process/measurement noise. All are std-dev-style densities; squared
/// internally where used as variances.
#[derive(Clone, Debug, serde::Deserialize, serde::Serialize, Tunable)]
#[serde(default)]
pub struct EkfParams {
    /// Accel process noise density [m/s^2 / sqrt(s)] driving velocity.
    #[param(0.05..5.0, default = 0.8, log)]
    pub q_accel: f32,
    /// Gyro process noise density [rad/s / sqrt(s)] driving yaw.
    #[param(0.005..2.0, default = 0.1, log)]
    pub q_gyro: f32,
    /// Accel-bias random-walk density [m/s^2 / sqrt(s)].
    #[param(1e-4..1e-1, default = 0.01, log)]
    pub q_bias_accel: f32,
    /// Gyro-bias random-walk density [rad/s / sqrt(s)].
    #[param(1e-5..1e-2, default = 0.001, log)]
    pub q_bias_gyro: f32,
    /// Pose measurement position std [m].
    #[param(0.01..1.0, default = 0.08, log)]
    pub r_pos: f32,
    /// Pose measurement yaw std [rad].
    #[param(0.005..0.5, default = 0.04, log)]
    pub r_yaw: f32,
    /// Zero-lateral-velocity pseudo-measurement std [m/s]. Larger = weaker.
    #[param(0.01..2.0, default = 0.3, log)]
    pub r_zero_vy: f32,
    /// Innovation gate (normalized innovation squared) above which a pose
    /// update is rejected as an outlier and reported as low confidence.
    #[param(1.0..100.0, default = 16.0)]
    pub pose_gate: f32,
}

impl Default for EkfParams {
    fn default() -> Self {
        Self {
            q_accel: 0.8,
            q_gyro: 0.1,
            q_bias_accel: 0.01,
            q_bias_gyro: 0.001,
            r_pos: 0.08,
            r_yaw: 0.04,
            r_zero_vy: 0.3,
            pose_gate: 16.0,
        }
    }
}

#[derive(Clone)]
pub struct Ekf {
    params: EkfParams,
    x: Vec8,
    p: Mat8,
    initialized: bool,
    /// Bias-corrected gyro from the most recent predict, cached for `motion()`.
    last_yaw_rate: f32,
    /// Normalized innovation squared of the last pose update; supervisor health.
    last_nis: f32,
    /// Confidence in [0,1] derived from the last accepted/rejected update.
    confidence: f32,
    last_timestamp_us: u64,
}

impl Ekf {
    pub fn new(params: EkfParams) -> Self {
        let mut p = [[0.0f32; N]; N];
        // Initial uncertainty: position/yaw fairly certain (we define the
        // origin), velocity unknown, biases moderately unknown.
        let p0 = [0.01, 0.01, 0.01, 0.5, 0.5, 0.1, 0.1, 0.01];
        for i in 0..N {
            p[i][i] = p0[i];
        }
        Self {
            params,
            x: [0.0; N],
            p,
            initialized: false,
            last_yaw_rate: 0.0,
            last_nis: 0.0,
            confidence: 0.0,
            last_timestamp_us: 0,
        }
    }

    /// Seed the filter at a known pose (e.g. world origin at the start line).
    pub fn initialize(&mut self, pose: Pose2) {
        self.x[X] = pose.x;
        self.x[Y] = pose.y;
        self.x[YAW] = pose.yaw;
        self.x[VX] = 0.0;
        self.x[VY] = 0.0;
        self.initialized = true;
        self.confidence = 1.0;
    }

    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    pub fn pose(&self) -> Pose2 {
        Pose2::new(self.x[X], self.x[Y], self.x[YAW])
    }

    pub fn motion(&self) -> MotionState {
        MotionState {
            speed_ms: self.x[VX].hypot(self.x[VY]),
            yaw_rate_rad_s: self.last_yaw_rate,
        }
    }

    pub fn confidence(&self) -> f32 {
        self.confidence
    }

    pub fn last_nis(&self) -> f32 {
        self.last_nis
    }

    pub fn accel_bias(&self) -> (f32, f32) {
        (self.x[BAX], self.x[BAY])
    }

    pub fn gyro_bias(&self) -> f32 {
        self.x[BGZ]
    }

    /// IMU prediction step. `ax,ay` are raw body-frame accelerations [m/s^2],
    /// `gz` raw yaw rate [rad/s], `dt` seconds.
    ///
    /// Motion model (body velocity, world position):
    ///   yaw'  = yaw + (gz - b_gz) * dt
    ///   vx'   = vx + (ax - b_ax) * dt
    ///   vy'   = vy + (ay - b_ay) * dt
    ///   x'    = x + (vx*cos(yaw) - vy*sin(yaw)) * dt
    ///   y'    = y + (vx*sin(yaw) + vy*cos(yaw)) * dt
    ///   biases: random walk (identity propagation)
    pub fn predict(&mut self, ax: f32, ay: f32, gz: f32, dt: f32) {
        if dt <= 0.0 {
            return;
        }
        let yaw = self.x[YAW];
        let (s, c) = yaw.sin_cos();
        let vx = self.x[VX];
        let vy = self.x[VY];
        let corr_gz = gz - self.x[BGZ];
        self.last_yaw_rate = corr_gz;

        // ── State propagation ────────────────────────────────────────────
        let mut xn = self.x;
        xn[X] = self.x[X] + (vx * c - vy * s) * dt;
        xn[Y] = self.x[Y] + (vx * s + vy * c) * dt;
        xn[YAW] = wrap_angle(yaw + corr_gz * dt);
        xn[VX] = vx + (ax - self.x[BAX]) * dt;
        xn[VY] = vy + (ay - self.x[BAY]) * dt;
        // biases unchanged in the mean (random walk)
        self.x = xn;

        // ── Jacobian F = d f / d x ───────────────────────────────────────
        let mut f = identity();
        // d x / d yaw, vx, vy
        f[X][YAW] = (-vx * s - vy * c) * dt;
        f[X][VX] = c * dt;
        f[X][VY] = -s * dt;
        // d y / d yaw, vx, vy
        f[Y][YAW] = (vx * c - vy * s) * dt;
        f[Y][VX] = s * dt;
        f[Y][VY] = c * dt;
        // d yaw / d b_gz
        f[YAW][BGZ] = -dt;
        // d vx / d b_ax
        f[VX][BAX] = -dt;
        // d vy / d b_ay
        f[VY][BAY] = -dt;

        // ── Process noise Q (diagonal, integrated over dt) ───────────────
        let mut q = [[0.0f32; N]; N];
        let qa = (self.params.q_accel * self.params.q_accel) * dt;
        let qg = (self.params.q_gyro * self.params.q_gyro) * dt;
        let qba = (self.params.q_bias_accel * self.params.q_bias_accel) * dt;
        let qbg = (self.params.q_bias_gyro * self.params.q_bias_gyro) * dt;
        // Position noise enters through velocity; add a small direct term too.
        q[X][X] = qa * dt * dt;
        q[Y][Y] = qa * dt * dt;
        q[YAW][YAW] = qg;
        q[VX][VX] = qa;
        q[VY][VY] = qa;
        q[BAX][BAX] = qba;
        q[BAY][BAY] = qba;
        q[BGZ][BGZ] = qbg;

        // P = F P F^T + Q
        let fp = matmul(&f, &self.p);
        let fpft = matmul_t(&fp, &f);
        self.p = add(&fpft, &q);
        symmetrize(&mut self.p);
    }

    /// Fold in a world-frame pose measurement (x, y, yaw). Returns true if the
    /// update was accepted (passed the innovation gate), false if rejected as
    /// an outlier. Either way `confidence`/`last_nis` are updated.
    pub fn correct_pose(&mut self, m: &PoseMeasurement) -> bool {
        // Measurement model H maps state -> (x, y, yaw); rows for X, Y, YAW.
        // Innovation y = z - h(x).
        let inn = [
            m.pose.x - self.x[X],
            m.pose.y - self.x[Y],
            wrap_angle(m.pose.yaw - self.x[YAW]),
        ];

        // Measurement noise scaled by (1/quality): a poor match inflates R.
        let q = m.quality.clamp(0.05, 1.0);
        let rp = (self.params.r_pos * self.params.r_pos) / q;
        let ry = (self.params.r_yaw * self.params.r_yaw) / q;
        let r = [rp, rp, ry];
        let rows = [X, Y, YAW];

        // S = H P H^T + R  (3x3). H selects rows/cols X,Y,YAW.
        let mut s = [[0.0f32; 3]; 3];
        for (a, &ra) in rows.iter().enumerate() {
            for (b, &rb) in rows.iter().enumerate() {
                s[a][b] = self.p[ra][rb];
            }
            s[a][a] += r[a];
        }
        let s_inv = match inv3(&s) {
            Some(v) => v,
            None => {
                self.confidence = 0.0;
                return false;
            }
        };

        // Normalized innovation squared: y^T S^-1 y. Gate outliers.
        let nis = quad3(&inn, &s_inv);
        self.last_nis = nis;
        if nis > self.params.pose_gate || !nis.is_finite() {
            // Reject: keep state, report low confidence proportional to overshoot.
            self.confidence = (self.params.pose_gate / nis).clamp(0.0, 1.0) * 0.5;
            return false;
        }

        // Kalman gain K = P H^T S^-1  (Nx3). P H^T selects cols X,Y,YAW of P.
        let mut pht = [[0.0f32; 3]; N];
        for i in 0..N {
            for (b, &rb) in rows.iter().enumerate() {
                pht[i][b] = self.p[i][rb];
            }
        }
        let mut k = [[0.0f32; 3]; N];
        for i in 0..N {
            for col in 0..3 {
                let mut acc = 0.0;
                for j in 0..3 {
                    acc += pht[i][j] * s_inv[j][col];
                }
                k[i][col] = acc;
            }
        }

        // State update x += K y
        for i in 0..N {
            let dx = k[i][0] * inn[0] + k[i][1] * inn[1] + k[i][2] * inn[2];
            self.x[i] += dx;
        }
        self.x[YAW] = wrap_angle(self.x[YAW]);

        // Joseph-form-lite covariance update: P = (I - K H) P.
        // (I - K H) has columns X,Y,YAW reduced; build it explicitly.
        let mut kh = [[0.0f32; N]; N];
        for i in 0..N {
            for (b, &rb) in rows.iter().enumerate() {
                // (K H)[i][rb] = K[i][b]; other columns zero.
                kh[i][rb] = k[i][b];
            }
        }
        let mut imkh = identity();
        for i in 0..N {
            for j in 0..N {
                imkh[i][j] -= kh[i][j];
            }
        }
        self.p = matmul(&imkh, &self.p);
        symmetrize(&mut self.p);

        // Confidence high when innovation is well inside the gate.
        self.confidence = (1.0 - nis / self.params.pose_gate).clamp(0.0, 1.0);
        true
    }

    /// Pseudo-measurement vy ≈ 0 to bound lateral-velocity drift. Cheap 1x1.
    pub fn correct_zero_vy(&mut self) {
        let r = self.params.r_zero_vy * self.params.r_zero_vy;
        let s = self.p[VY][VY] + r;
        if s <= 0.0 {
            return;
        }
        let inn = -self.x[VY];
        // K = P[:,VY] / s
        let mut k = [0.0f32; N];
        for i in 0..N {
            k[i] = self.p[i][VY] / s;
        }
        for i in 0..N {
            self.x[i] += k[i] * inn;
        }
        // P = (I - K H) P, H selects VY.
        let mut imkh = identity();
        for i in 0..N {
            for j in 0..N {
                // (K H)[i][j] nonzero only for j == VY.
                if j == VY {
                    imkh[i][j] -= k[i];
                }
            }
        }
        self.p = matmul(&imkh, &self.p);
        symmetrize(&mut self.p);
    }
}

impl StateEstimator for Ekf {
    fn reset(&mut self, pose: Pose2, timestamp_us: u64) {
        *self = Ekf::new(self.params.clone());
        self.initialize(pose);
        self.last_timestamp_us = timestamp_us;
    }

    fn predict_imu(&mut self, sample: ImuSample) {
        if !self.initialized {
            self.initialize(Pose2::default());
            self.last_timestamp_us = sample.timestamp_us;
            return;
        }
        let dt = if self.last_timestamp_us == 0 {
            0.0
        } else {
            sample.timestamp_us.saturating_sub(self.last_timestamp_us) as f32 * 1e-6
        };
        self.last_timestamp_us = sample.timestamp_us;
        // Ignore pathological timestamp jumps rather than injecting a huge
        // one-step covariance/state change.
        if (0.0..=0.25).contains(&dt) {
            self.predict(sample.ax, sample.ay, sample.gz, dt);
        }
    }

    fn correct_pose(&mut self, measurement: PoseMeasurement) -> bool {
        Ekf::correct_pose(self, &measurement)
    }

    fn estimate(&self) -> StateEstimate {
        let diverged = !self.x.iter().all(|v| v.is_finite())
            || !self.p.iter().flatten().all(|v| v.is_finite())
            || self.p[X][X] > 25.0
            || self.p[Y][Y] > 25.0
            || self.p[YAW][YAW] > 4.0;
        StateEstimate {
            pose: self.pose(),
            motion: self.motion(),
            confidence: self.confidence(),
            consistency: self.last_nis(),
            diverged,
            timestamp_us: self.last_timestamp_us,
        }
    }
}

// ── Small dense linear algebra (fixed N) ────────────────────────────────────

fn identity() -> Mat8 {
    let mut m = [[0.0f32; N]; N];
    for i in 0..N {
        m[i][i] = 1.0;
    }
    m
}

fn matmul(a: &Mat8, b: &Mat8) -> Mat8 {
    let mut o = [[0.0f32; N]; N];
    for i in 0..N {
        for k in 0..N {
            let aik = a[i][k];
            if aik == 0.0 {
                continue;
            }
            for j in 0..N {
                o[i][j] += aik * b[k][j];
            }
        }
    }
    o
}

/// a * b^T
fn matmul_t(a: &Mat8, b: &Mat8) -> Mat8 {
    let mut o = [[0.0f32; N]; N];
    for i in 0..N {
        for j in 0..N {
            let mut acc = 0.0;
            for k in 0..N {
                acc += a[i][k] * b[j][k];
            }
            o[i][j] = acc;
        }
    }
    o
}

fn add(a: &Mat8, b: &Mat8) -> Mat8 {
    let mut o = [[0.0f32; N]; N];
    for i in 0..N {
        for j in 0..N {
            o[i][j] = a[i][j] + b[i][j];
        }
    }
    o
}

/// Force symmetry to suppress round-off asymmetry that can break PSD-ness.
fn symmetrize(m: &mut Mat8) {
    for i in 0..N {
        for j in (i + 1)..N {
            let avg = 0.5 * (m[i][j] + m[j][i]);
            m[i][j] = avg;
            m[j][i] = avg;
        }
    }
}

fn inv3(m: &[[f32; 3]; 3]) -> Option<[[f32; 3]; 3]> {
    let a = m[0][0];
    let b = m[0][1];
    let c = m[0][2];
    let d = m[1][0];
    let e = m[1][1];
    let f = m[1][2];
    let g = m[2][0];
    let h = m[2][1];
    let i = m[2][2];

    let det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    if det.abs() < 1e-12 {
        return None;
    }
    let inv_det = 1.0 / det;
    Some([
        [
            (e * i - f * h) * inv_det,
            (c * h - b * i) * inv_det,
            (b * f - c * e) * inv_det,
        ],
        [
            (f * g - d * i) * inv_det,
            (a * i - c * g) * inv_det,
            (c * d - a * f) * inv_det,
        ],
        [
            (d * h - e * g) * inv_det,
            (b * g - a * h) * inv_det,
            (a * e - b * d) * inv_det,
        ],
    ])
}

/// y^T M y for 3-vectors.
fn quad3(y: &[f32; 3], m: &[[f32; 3]; 3]) -> f32 {
    let mut my = [0.0f32; 3];
    for i in 0..3 {
        my[i] = m[i][0] * y[0] + m[i][1] * y[1] + m[i][2] * y[2];
    }
    y[0] * my[0] + y[1] * my[1] + y[2] * my[2]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_close(actual: f32, expected: f32, tolerance: f32) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "actual={actual} expected={expected} tolerance={tolerance}"
        );
    }

    #[test]
    fn straight_drive_predicts_forward() {
        let mut ekf = Ekf::new(EkfParams::default());
        ekf.initialize(Pose2::new(0.0, 0.0, 0.0));
        // Accelerate forward at 1 m/s^2 for 1 s, then coast 1 s.
        for _ in 0..100 {
            ekf.predict(1.0, 0.0, 0.0, 0.01);
        }
        let m = ekf.motion();
        assert!((m.speed_ms - 1.0).abs() < 0.05, "speed={}", m.speed_ms);
        let p = ekf.pose();
        // x ≈ 0.5 a t^2 = 0.5
        assert!((p.x - 0.5).abs() < 0.05, "x={}", p.x);
        assert!(p.y.abs() < 1e-3, "y drift={}", p.y);
    }

    #[test]
    fn imu_prediction_matches_dead_reckon_phase3_baseline() {
        let baseline: serde_json::Value = serde_json::from_str(include_str!(
            "../../tests/fixtures/phase3_dead_reckon_baseline.json"
        ))
        .unwrap();
        let expected = &baseline["estimate"];
        let mut ekf = Ekf::new(EkfParams::default());
        ekf.reset(Pose2::new(1.0, -2.0, 0.1), 1_000);

        for sample in [
            ImuSample {
                ax: 0.5,
                ay: 0.0,
                gz: 0.1,
                timestamp_us: 101_000,
                ..Default::default()
            },
            ImuSample {
                ax: 0.5,
                ay: 0.0,
                gz: 0.1,
                timestamp_us: 201_000,
                ..Default::default()
            },
            ImuSample {
                ax: 0.0,
                ay: 0.0,
                gz: 0.1,
                timestamp_us: 301_000,
                ..Default::default()
            },
        ] {
            ekf.predict_imu(sample);
        }
        let out = ekf.estimate();

        assert_close(
            out.pose.x,
            expected["pose"]["x"].as_f64().unwrap() as f32,
            1e-6,
        );
        assert_close(
            out.pose.y,
            expected["pose"]["y"].as_f64().unwrap() as f32,
            1e-6,
        );
        assert_close(
            out.pose.yaw,
            expected["pose"]["yaw"].as_f64().unwrap() as f32,
            1e-6,
        );
        assert_close(
            out.motion.speed_ms,
            expected["speed_ms"].as_f64().unwrap() as f32,
            1e-6,
        );
        assert_close(
            out.motion.yaw_rate_rad_s,
            expected["yaw_rate_rad_s"].as_f64().unwrap() as f32,
            1e-6,
        );
        assert_eq!(out.timestamp_us, expected["timestamp_us"].as_u64().unwrap());
    }

    #[test]
    fn gyro_bias_is_observable_from_pose_yaw() {
        let mut ekf = Ekf::new(EkfParams::default());
        ekf.initialize(Pose2::new(0.0, 0.0, 0.0));
        // True gz = 0, but IMU reports a constant +0.1 rad/s bias.
        // Repeatedly predict then correct against the true (unchanging) yaw.
        for _ in 0..400 {
            ekf.predict(0.0, 0.0, 0.1, 0.01);
            ekf.correct_pose(&PoseMeasurement {
                pose: Pose2::new(ekf.pose().x, ekf.pose().y, 0.0),
                quality: 1.0,
            });
        }
        // Filter should attribute the drift to gyro bias, converging toward 0.1.
        assert!(
            (ekf.gyro_bias() - 0.1).abs() < 0.05,
            "gyro bias not recovered: {}",
            ekf.gyro_bias()
        );
    }

    #[test]
    fn outlier_pose_is_gated_out() {
        let mut ekf = Ekf::new(EkfParams::default());
        ekf.initialize(Pose2::new(0.0, 0.0, 0.0));
        ekf.predict(0.0, 0.0, 0.0, 0.01);
        // A wild 10 m jump must be rejected.
        let accepted = ekf.correct_pose(&PoseMeasurement {
            pose: Pose2::new(10.0, 10.0, 3.0),
            quality: 1.0,
        });
        assert!(!accepted, "huge jump should be gated");
        assert!(ekf.pose().x.abs() < 0.5, "state moved despite gate");
        assert!(ekf.confidence() < 0.6);
    }

    #[test]
    fn covariance_stays_finite_and_symmetric() {
        let mut ekf = Ekf::new(EkfParams::default());
        ekf.initialize(Pose2::new(0.0, 0.0, 0.0));
        for k in 0..500 {
            ekf.predict(0.3, 0.05, 0.02, 0.01);
            if k % 5 == 0 {
                let p = ekf.pose();
                ekf.correct_pose(&PoseMeasurement {
                    pose: Pose2::new(p.x, p.y, p.yaw),
                    quality: 0.8,
                });
            }
            ekf.correct_zero_vy();
        }
        for i in 0..N {
            for j in 0..N {
                assert!(ekf.p[i][j].is_finite(), "P has non-finite entry");
                assert!((ekf.p[i][j] - ekf.p[j][i]).abs() < 1e-3, "P asymmetric");
            }
        }
    }

    #[test]
    fn zero_vy_pseudo_measurement_reduces_lateral_velocity() {
        let mut ekf = Ekf::new(EkfParams::default());
        ekf.initialize(Pose2::new(0.0, 0.0, 0.0));
        // Inject lateral accel to build up vy.
        for _ in 0..50 {
            ekf.predict(0.0, 0.5, 0.0, 0.01);
        }
        let vy_before = ekf.x[VY].abs();
        for _ in 0..50 {
            ekf.correct_zero_vy();
        }
        let vy_after = ekf.x[VY].abs();
        assert!(
            vy_after < vy_before,
            "zero-vy should shrink vy: {vy_before}->{vy_after}"
        );
    }
}
