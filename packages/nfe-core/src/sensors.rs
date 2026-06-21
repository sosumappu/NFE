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
    pub fn point2(&self) -> Point2 {
        Point2::new(self.x, self.y)
    }
}

#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct LidarCloud {
    pub points: Vec<LidarPoint>,
    pub timestamp_us: u64,
}

impl LidarCloud {
    pub fn as_points2(&self) -> Vec<Point2> {
        self.points.iter().map(LidarPoint::point2).collect()
    }
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
