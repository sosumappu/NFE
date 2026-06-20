/// types.rs
///
/// Contains all necessary types
/// LidarPoint, LidarCloud are structtured to store the LiDAR data
/// ImuSample stores the imu readings
/// ImuBias, Attitude, and KinematicState are used to properly integrate the imu readings to get the
/// speed and pose
///
/// One point in the car's local frame.
///   +x = forward, +y = left
///   angle_deg: car-frame angle, -180..+180 (negative = right)
///
use std::f32::consts::{PI, TAU};
use std::ops::Sub;

const GRAVITY_MS2: f32 = 9.80665; // depending on orientation can be negative

#[derive(serde::Serialize, serde::Deserialize, Clone, Copy, Debug)]
pub struct LidarPoint {
    pub x: f32,         // metres
    pub y: f32,         // metres
    pub dist_m: f32,    // range (redundant but convenient for filtering)
    pub angle_rad: f32, // car-frame degrees
    pub timestamp_us: u64,
}

impl LidarPoint {
    pub fn angle_diff(&self, rhs: &Self) -> f32 {
        (self.angle_rad - rhs.angle_rad + PI).rem_euclid(TAU) - PI
    }
    pub fn derivative(&self, rhs: &Self) -> f32 {
        let d_theta = self.angle_diff(rhs);
        if d_theta.abs() < f32::EPSILON {
            0.0
        } else {
            (self.dist_m - rhs.dist_m) / d_theta
        }
    }

    pub fn opposite(&self) -> f32 {
        (self.angle_rad + PI).rem_euclid(TAU) - PI
    }
    pub fn hermit_interpolation(&self, rhs: &Self, prev: &Self, next: &Self, theta: f32) -> f32 {
        let dot_rho_a = self.derivative(prev);
        let dot_rho_b = next.derivative(rhs);

        let d_theta = rhs.angle_diff(self); // B - A

        let target_diff = (theta - self.angle_rad + PI).rem_euclid(TAU) - PI;
        let t = target_diff / d_theta;

        // Calculate power for performance
        let t2 = t * t;
        let t3 = t2 * t;

        let h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
        let h10 = -2.0 * t3 + 3.0 * t2;
        let h01 = (t3 - 2.0 * t2 + t) * d_theta;
        let h11 = (t3 - t2) * d_theta;

        h00 * self.dist_m + h10 * rhs.dist_m + h01 * dot_rho_a + h11 * dot_rho_b
    }
}

/// éviter les allocations
#[derive(serde::Serialize, serde::Deserialize, Clone, Debug, Default)]
pub struct LidarCloud {
    pub points: Vec<LidarPoint>,
    pub timestamp_us: u64,
}

pub struct LidarCloudView<'a> {
    pub points: &'a [LidarPoint],
    pub timestamp_us: u64,
}

impl LidarCloud {
    /// Nearest point within |angle_deg| <= half_arc_deg.
    /// Returns None if the arc has no valid returns.
    pub fn nearest_in_arc(&self, centre_rad: f32, half_arc_rad: f32) -> Option<&LidarPoint> {
        self.points
            .iter()
            .filter(|p| (p.angle_rad - centre_rad).abs() <= half_arc_rad)
            .min_by(|a, b| a.dist_m.partial_cmp(&b.dist_m).unwrap())
    }

    /// Nearest obstacle anywhere in the cloud.
    pub fn nearest(&self) -> Option<&LidarPoint> {
        self.points
            .iter()
            .min_by(|a, b| a.dist_m.partial_cmp(&b.dist_m).unwrap())
    }
}

impl<'a> LidarCloudView<'a> {
    pub fn median_filtered(
        &self,
        buf: &'a mut Vec<LidarPoint>,
        half_width: usize,
    ) -> LidarCloudView<'a> {
        let n = self.points.len();
        buf.clear();
        buf.reserve(n);

        // Scratch space — reused each iteration, no per-point allocation
        let window_size = 2 * half_width + 1;
        let mut scratch = Vec::with_capacity(window_size);

        for i in 0..n {
            scratch.clear();

            // Collect distances from the window, wrapping circularly
            for j in 0..window_size {
                let idx = (i + n + j - half_width) % n;
                scratch.push(self.points[idx].dist_m);
            }

            // Partial sort — only need the median, not full sort
            let mid = scratch.len() / 2;
            let median_dist = *scratch
                .select_nth_unstable_by(mid, |a, b| {
                    a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal)
                })
                .1;

            // Preserve original point geometry, replace distance
            let p = &self.points[i];
            let angle_rad = p.angle_rad;
            buf.push(LidarPoint {
                dist_m: median_dist,
                x: median_dist * angle_rad.cos(),
                y: -median_dist * angle_rad.sin(),
                ..*p
            });
        }

        LidarCloudView {
            points: buf,
            timestamp_us: self.timestamp_us,
        }
    }
    pub fn find_breakpoint(&self) -> Option<&LidarPoint> {
        if self.points.len() < 2 {
            return None;
        }

        let mut max = 0.0f32;
        let mut best: Option<&LidarPoint> = None;

        for w in self.points.windows(2) {
            //TODO: Monitor how .abs
            let deriv = (w[1].derivative(&w[0]));

            if deriv.abs() > max {
                max = deriv.abs();
                best = Some(if w[0].dist_m < w[1].dist_m {
                    &w[0]
                } else {
                    &w[1]
                });
            }
        }

        best
    }
}

// ── IMU sample ─────────────────────────────────────────────────────────────

#[derive(serde::Serialize, serde::Deserialize, Clone, Copy, Debug, Default)]
pub struct ImuSample {
    pub ax: f32,
    pub ay: f32,
    pub az: f32, // m/s²
    pub gx: f32,
    pub gy: f32,
    pub gz: f32, // rad/s
    pub timestamp_us: u64,
}

// ── Kinematics ─────────────────────────────────────────────────────────────
#[derive(Debug, Clone, Copy)]
pub struct ImuBias {
    pub bax: f32,
    pub bay: f32,
    pub baz: f32,
    pub bgx: f32,
    pub bgy: f32,
    pub bgz: f32,
}

impl ImuSample {
    /// Rotates the sample's linear acceleration components by a given yaw angle (rad).
    /// Assumes the sample has already been bias-corrected.
    pub fn rotate_accel_2d(&self, yaw_rad: f32) -> (f32, f32) {
        let (sin_y, cos_y) = yaw_rad.sin_cos();
        let ax_w = cos_y * self.ax - sin_y * self.ay;
        let ay_w = sin_y * self.ax + cos_y * self.ay;
        (ax_w, ay_w)
    }
}

impl Sub<ImuBias> for ImuSample {
    type Output = ImuSample;

    fn sub(self, rhs: ImuBias) -> ImuSample {
        ImuSample {
            ax: self.ax - rhs.bax,
            ay: self.ay - rhs.bay,
            az: self.az - rhs.baz,
            gx: self.gx - rhs.bgx,
            gy: self.gy - rhs.bgy,
            gz: self.gz - rhs.bgz,
            ..self
        }
    }
}

impl ImuBias {
    pub fn estimate(samples: &[ImuSample]) -> Self {
        let n = samples.len() as f32;

        // Safety fallback if an empty slice is passed
        if n <= 0.0 {
            return Self {
                bax: 0.0,
                bay: 0.0,
                baz: 0.0,
                bgx: 0.0,
                bgy: 0.0,
                bgz: 0.0,
            };
        }

        let mut sum_ax = 0.0;
        let mut sum_ay = 0.0;
        let mut sum_az = 0.0;
        let mut sum_gx = 0.0;
        let mut sum_gy = 0.0;
        let mut sum_gz = 0.0;

        for sample in samples {
            sum_ax += sample.ax;
            sum_ay += sample.ay;
            sum_az += sample.az;
            sum_gx += sample.gx;
            sum_gy += sample.gy;
            sum_gz += sample.gz;
        }

        Self {
            bax: sum_ax / n,
            bay: sum_ay / n,
            // WARN:needs to match the way we install the MPU on the car
            baz: sum_az / n - GRAVITY_MS2,
            bgx: sum_gx / n,
            bgy: sum_gy / n,
            bgz: sum_gz / n,
        }
    }
}
pub struct KinematicState {
    pub vx: f32,
    pub vy: f32,
    pub yaw_rad: f32,
    pub gz: f32,
    pub lateral_error_m: f32,
    pub timestamp_us: u64,
}

impl KinematicState {
    // Linearly interpolate volocities between 'lo' and 'hi' at target time t
    pub fn linear_interpolation_veloc(&self, other: &Self, t: u64) -> (f32, f32) {
        if self.timestamp_us >= other.timestamp_us {
            return (self.vx, self.vy);
        }
        let alpha =
            (t - self.timestamp_us) as f32 / (other.timestamp_us - self.timestamp_us) as f32;
        (
            self.vx + alpha * (other.vx - self.vx),
            self.vy + alpha * (other.vy - self.vy),
        )
    }
}
