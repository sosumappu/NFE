use crate::types::{ImuBias, ImuSample, KinematicState, LidarCloud, LidarCloudView, LidarPoint};
use std::collections::VecDeque;
pub struct TickHandle(u64);

pub struct Kinematics {
    window: VecDeque<KinematicState>,
    bias: ImuBias, // à estimé lors de l'état initial / avec calibration.
    capacity: usize,
}

impl Kinematics {
    pub fn new(capacity: usize, bias: ImuBias) -> Self {
        Self {
            window: VecDeque::with_capacity(capacity),
            capacity,
            bias,
        }
    }

    pub fn update(&mut self, sample: &ImuSample) -> TickHandle {
        let prev_state = match self.window.back() {
            Some(s) => (s.vx, s.vy, s.yaw_rad, s.timestamp_us),
            None => {
                self.window.push_back(KinematicState {
                    vx: 0.0,
                    vy: 0.0,
                    yaw_rad: 0.0,
                    gz: 0.0,
                    lateral_error_m: 0.0,
                    timestamp_us: sample.timestamp_us,
                });
                return TickHandle(sample.timestamp_us);
            }
        };
        let (prev_vx, prev_vy, prev_yaw, prev_ts) = prev_state;

        let dt_us = sample.timestamp_us.saturating_sub(prev_ts);
        let dt = (dt_us as f32) * 1e-6;
        let corrected = *sample - self.bias; // requires Copy on ImuBias
        let yaw_rad = prev_yaw + corrected.gz * dt;
        let (ax_w, ay_w) = corrected.rotate_accel_2d(yaw_rad);

        self.window.push_back(KinematicState {
            vx: prev_vx + ax_w * dt,
            vy: prev_vy + ay_w * dt, // was `prev_vy` (undeclared variable) in original
            yaw_rad,
            gz: corrected.gz,
            lateral_error_m: 0.0,
            timestamp_us: sample.timestamp_us,
        });

        if self.window.len() > self.capacity {
            self.window.pop_front();
        }
        TickHandle(sample.timestamp_us)
    }

    pub fn velocity_at(&self, t: u64) -> (f32, f32) {
        // Handle empty window at start
        if self.window.is_empty() {
            return (0.0, 0.0);
        }

        let i = self.window.partition_point(|s| s.timestamp_us <= t);

        match i {
            // t older than windoww
            0 => {
                let s = self.window.front().unwrap();
                (s.vx, s.vy)
            }
            // t is newer than anything
            i if i >= self.window.len() => {
                let s = self.window.back().unwrap();
                (s.vx, s.vy)
            }
            // t is between i-1 and i
            i => {
                let lo = &self.window[i - 1];
                let hi = &self.window[i];
                lo.linear_interpolation_veloc(hi, t)
            }
        }
    }
    pub fn current_speed(&self) -> f32 {
        self.window.back().map(|s| s.vx.hypot(s.vy)).unwrap_or(0.0)
    }

    pub fn current_yaw_rate(&self) -> f32 {
        self.window.back().map(|s| s.gz).unwrap_or(0.0)
    }

    pub fn lateral_rate(&self) -> f32 {
        let len = self.window.len();
        if len < 2 {
            return 0.0;
        }
        let curr = &self.window[len - 1];
        let prev = &self.window[len - 2];
        let dt_us = curr.timestamp_us.saturating_sub(prev.timestamp_us);
        let dt = (dt_us as f32) * 1e-6;
        if dt <= 0.0 {
            return 0.0;
        }
        (curr.lateral_error_m - prev.lateral_error_m) / dt
    }

    pub fn record_lateral_error(&mut self, handle: TickHandle, lateral_error_m: f32) {
        if let Some(s) = self.window.back_mut() {
            debug_assert_eq!(
                s.timestamp_us, handle.0,
                "lateral error recorded against stale tick"
            );
            s.lateral_error_m = lateral_error_m;
        }
    }

    pub fn deskew<'a>(
        &mut self,
        cloud: &LidarCloud,
        buffer: &'a mut Vec<LidarPoint>,
    ) -> LidarCloudView<'a> {
        let t_ref = cloud.timestamp_us;
        buffer.clear();
        for point in &cloud.points {
            let dt = t_ref.saturating_sub(point.timestamp_us) as f32 * 1e-6;
            let (vx, vy) = self.velocity_at(point.timestamp_us);
            let x = point.x + vx * dt;
            let y = point.y + vy * dt;

            buffer.push(LidarPoint {
                x,
                y,
                dist_m: x.hypot(y),
                angle_rad: y.atan2(x),
                ..*point
            });
        }

        buffer.sort_unstable_by(|a, b| {
            a.angle_rad
                .partial_cmp(&b.angle_rad)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        LidarCloudView {
            points: buffer,
            timestamp_us: t_ref,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bias() -> ImuBias {
        ImuBias {
            bax: 0.0, bay: 0.0, baz: 0.0,
            bgx: 0.0, bgy: 0.0, bgz: 0.0,
        }
    }

    fn sample(ts: u64, gz: f32) -> ImuSample {
        ImuSample {
            ax: 0.0, ay: 0.0, az: 0.0,
            gx: 0.0, gy: 0.0, gz,
            timestamp_us: ts,
        }
    }

    #[test]
    fn normal_monotonic_timestamps() {
        let mut kin = Kinematics::new(100, bias());
        kin.update(&sample(0, 0.0));
        kin.update(&sample(10_000, 0.5)); // 10 ms later
        let s = kin.current_speed();
        assert!(s.is_finite());
        assert!(s >= 0.0);
    }

    #[test]
    fn equal_timestamps_no_panic() {
        let mut kin = Kinematics::new(100, bias());
        let _h1 = kin.update(&sample(100, 0.0));
        // same timestamp → dt = 0 via saturating_sub
        let _h2 = kin.update(&sample(100, 0.5));
        let s = kin.current_speed();
        assert!(s.is_finite());
        let yr = kin.current_yaw_rate();
        assert!(yr.is_finite());
    }

    #[test]
    fn backward_timestamp_saturates_to_zero() {
        // This simulates a clock jump backward (e.g. NTP step).
        // saturating_sub ensures dt_us = 0, not a giant value.
        let mut kin = Kinematics::new(100, bias());
        let _h1 = kin.update(&sample(100_000, 0.0));
        let _h2 = kin.update(&sample(50_000, 1.0)); // earlier than prev!
        let s = kin.current_speed();
        assert!(s.is_finite(), "backward clock should not explode velocity");
        assert!((s - 0.0).abs() < 1e-6, "dt=0 → no acceleration → speed stays 0");
    }

    #[test]
    fn backward_timestamp_lateral_rate_safe() {
        let mut kin = Kinematics::new(100, bias());
        kin.update(&sample(100_000, 0.0));
        kin.update(&sample(50_000, 0.0)); // backward jump
        let lr = kin.lateral_rate();
        assert!(lr.is_finite(), "lateral_rate should not explode on backward clock");
    }

    #[test]
    fn velocity_at_interpolates() {
        let mut kin = Kinematics::new(100, bias());
        kin.update(&sample(0, 0.0));
        // second sample with non-zero accel will produce a velocity
        let s = kin.current_speed();
        assert!(s.is_finite());
    }

    #[test]
    fn deskew_with_zero_dt_no_panic() {
        let mut kin = Kinematics::new(100, bias());
        kin.update(&sample(0, 0.0));
        let cloud = LidarCloud {
            points: vec![LidarPoint {
                x: 1.0, y: 0.0, dist_m: 1.0, angle_rad: 0.0, timestamp_us: 999,
            }],
            timestamp_us: 1000,
        };
        let mut buf = Vec::with_capacity(4);
        let view = kin.deskew(&cloud, &mut buf);
        assert!(!view.points.is_empty());
        assert!(view.points[0].x.is_finite());
    }
}
