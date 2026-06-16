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

        let dt = (sample.timestamp_us - prev_ts) as f32 * 1e-6;
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
        let dt = (curr.timestamp_us - prev.timestamp_us) as f32 * 1e-6;
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
