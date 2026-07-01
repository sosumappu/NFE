//! Sensor-frame types used by pure runtime pipeline tests.

use crate::Point2;

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct LidarPoint {
    pub x: f32,
    pub y: f32,
    pub dist_m: f32,
    pub angle_rad: f32,
    pub timestamp_us: u64,
}

impl LidarPoint {
    pub fn from_polar(dist_m: f32, angle_rad: f32, timestamp_us: u64) -> Self {
        Self {
            x: dist_m * angle_rad.cos(),
            y: dist_m * angle_rad.sin(),
            dist_m,
            angle_rad,
            timestamp_us,
        }
    }

    pub fn point2(&self) -> Point2 {
        Point2::new(self.x, self.y)
    }
}

#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct LidarCloud {
    pub points: Vec<LidarPoint>,
    pub timestamp_us: u64,
}

#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct SensorSnapshot {
    pub lidar: LidarCloud,
    pub imu: crate::estimation::ImuSample,
    pub sensor_fault: bool,
    /// Distances from [front, left, right] sonars; may be f32::MAX if unused.
    pub sonar_m: [f32; 3],
    /// Optional physical start-line crossing edge from timing gates.
    pub start_line_crossed: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lidar_frame_convention_positive_angles_point_left() {
        let left = LidarPoint::from_polar(1.5, std::f32::consts::FRAC_PI_2, 10);
        assert!(left.x.abs() < 1.0e-6, "left x={}", left.x);
        assert!((left.y - 1.5).abs() < 1.0e-6, "left y={}", left.y);

        let right = LidarPoint::from_polar(1.5, -std::f32::consts::FRAC_PI_2, 20);
        assert!(right.x.abs() < 1.0e-6, "right x={}", right.x);
        assert!((right.y + 1.5).abs() < 1.0e-6, "right y={}", right.y);
    }
}
