//! Localization trait boundary.

use crate::estimation::PoseMeasurement;
use crate::mapping::TrackMap;
use crate::sensors::LidarCloud;
use crate::Pose2;

#[derive(Clone, Copy, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct LocalizationResult {
    pub measurement: Option<PoseMeasurement>,
    pub confidence: f32,
    pub residual_m: f32,
    pub used_fallback: bool,
}

pub trait Localizer {
    fn localize(&mut self, cloud: &LidarCloud, prior: Pose2, map: &TrackMap) -> LocalizationResult;
}
