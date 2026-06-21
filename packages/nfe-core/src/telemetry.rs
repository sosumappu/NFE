//! Telemetry schema shared by runtime, recorders, replay, tests, and tools.
//!
//! This module intentionally contains only pure data types. It does not know
//! about mpsc, MCAP, protobuf, Foxglove, threads, files, or sockets.

use crate::control::{ControlOutput, CorridorEstimate};
use crate::estimation::StateEstimate;
use crate::localization::LocalizationResult;
use crate::mapping::{LoopClosureReport, MapStatus, TrackMap};
use crate::raceline::{RaceLine, RaceReference};
use crate::sensors::SensorSnapshot;
use crate::{MotionState, WallLine};

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub enum TelemetryEvent {
    Sensor(SensorTelemetry),
    Control(ControlTelemetry),
    Metrics(MetricsTelemetry),
    Perception(PerceptionTelemetry),
    Estimation(EstimationTelemetry),
    Mapping(MappingTelemetry),
    Localization(LocalizationTelemetry),
    Planning(PlanningTelemetry),
    Supervisor(SupervisorTelemetry),
    Race(RaceTelemetry),
    World(WorldTelemetry),
    GroundTruth(GroundTruthTelemetry),
    StartGate(StartGateTelemetry),
}

impl TelemetryEvent {
    pub fn timestamp_us(&self) -> u64 {
        match self {
            TelemetryEvent::Sensor(e) => e.timestamp_us(),
            TelemetryEvent::Control(e) => e.timestamp_us,
            TelemetryEvent::Metrics(e) => e.timestamp_us,
            TelemetryEvent::Perception(e) => e.timestamp_us,
            TelemetryEvent::Estimation(e) => e.timestamp_us,
            TelemetryEvent::Mapping(e) => e.timestamp_us,
            TelemetryEvent::Localization(e) => e.timestamp_us,
            TelemetryEvent::Planning(e) => e.timestamp_us,
            TelemetryEvent::Supervisor(e) => e.timestamp_us,
            TelemetryEvent::Race(e) => e.timestamp_us,
            TelemetryEvent::World(e) => e.timestamp_us(),
            TelemetryEvent::GroundTruth(e) => e.timestamp_us(),
            TelemetryEvent::StartGate(e) => e.timestamp_us,
        }
    }
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub enum SensorTelemetry {
    Snapshot(SensorSnapshot),
}

impl SensorTelemetry {
    pub fn timestamp_us(&self) -> u64 {
        match self {
            SensorTelemetry::Snapshot(s) => s.lidar.timestamp_us.max(s.imu.timestamp_us),
        }
    }
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ControlTelemetry {
    pub timestamp_us: u64,
    pub output: ControlOutput,
    pub motion: MotionState,
}

#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct MetricsTelemetry {
    pub tick: u64,
    pub timestamp_us: u64,
    pub loop_us: u32,
    pub lateral_error_m: f32,
    pub heading_error_rad: f32,
    pub steering_rad: f32,
    pub throttle: f32,
    pub target_speed_ms: f32,
    pub current_speed_ms: f32,
    pub nearest_obstacle_m: f32,
    pub estop: bool,
    pub watchdog_miss: bool,
    pub sensor_fault: bool,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub enum PerceptionTelemetryKind {
    ReactiveCorridor(CorridorEstimate),
    ReactiveRansacWalls(RansacWallFitFrame),
    MappingRansacWalls(RansacWallFitFrame),
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct PerceptionTelemetry {
    pub timestamp_us: u64,
    pub kind: PerceptionTelemetryKind,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct RansacWallFitFrame {
    pub frame_id: String,
    pub source: WallFitSource,
    pub walls: Vec<WallLine>,
    pub points_total: u32,
    pub inliers_total: u32,
    pub confidence: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum WallFitSource {
    Reactive,
    Mapping,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub enum EstimationTelemetryKind {
    EkfState(EkfStateTelemetry),
    EkfCovariance(EkfCovarianceTelemetry),
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct EstimationTelemetry {
    pub timestamp_us: u64,
    pub kind: EstimationTelemetryKind,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct EkfStateTelemetry {
    pub estimate: StateEstimate,
    pub accel_bias: [f32; 2],
    pub gyro_bias_z: f32,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct EkfCovarianceTelemetry {
    /// Row-major covariance. Empty means unavailable/not published.
    pub covariance: Vec<f32>,
    pub dimension: u32,
    pub confidence: f32,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub enum MappingTelemetryKind {
    Status(MapStatus),
    GlobalMapDelta {
        revision: u64,
        walls_added: Vec<WallLine>,
    },
    GlobalMapSnapshot(TrackMap),
    LoopClosure(LoopClosureReport),
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct MappingTelemetry {
    pub timestamp_us: u64,
    pub kind: MappingTelemetryKind,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub enum LocalizationTelemetryKind {
    ScanMatch(LocalizerScoreTelemetry),
    ParticleFilter(LocalizerScoreTelemetry),
    Result(LocalizationResult),
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct LocalizationTelemetry {
    pub timestamp_us: u64,
    pub kind: LocalizationTelemetryKind,
}

#[derive(Clone, Copy, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct LocalizerScoreTelemetry {
    pub confidence: f32,
    pub residual_m: f32,
    pub match_count: u32,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub enum PlanningTelemetryKind {
    RaceLine(RaceLine),
    RaceReference(RaceReference),
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct PlanningTelemetry {
    pub timestamp_us: u64,
    pub kind: PlanningTelemetryKind,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub enum SupervisorTelemetryKind {
    State(SupervisorStateTelemetry),
    Transition(SupervisorTransitionTelemetry),
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct SupervisorTelemetry {
    pub timestamp_us: u64,
    pub kind: SupervisorTelemetryKind,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct SupervisorStateTelemetry {
    pub drive_mode: String,
    pub localization_confidence: f32,
    pub estimator_diverged: bool,
    pub loop_closure_detected: bool,
    pub loop_closure_residual_m: f32,
    pub loop_closure_overlap: f32,
    pub mapping_enabled: bool,
    pub raceline_ready: bool,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct SupervisorTransitionTelemetry {
    pub from_mode: String,
    pub to_mode: String,
    pub reason: String,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub enum RaceTelemetryKind {
    StartLineGate { crossed: bool },
    LapComplete { lap: u32 },
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct RaceTelemetry {
    pub timestamp_us: u64,
    pub kind: RaceTelemetryKind,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub enum WorldTelemetry {
    Snapshot(WorldSnapshotTelemetry),
}

impl WorldTelemetry {
    pub fn timestamp_us(&self) -> u64 {
        match self {
            WorldTelemetry::Snapshot(v) => v.timestamp_us,
        }
    }
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct WorldSnapshotTelemetry {
    pub timestamp_us: u64,
    pub frame_id: String,
    pub revision: u64,
    pub walls: Vec<WallSegmentTelemetry>,
    pub start_pose: crate::Pose2,
    pub waypoints: Vec<crate::Point2>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum WallKind {
    Inner,
    Outer,
    Unknown,
}

#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct WallSegmentTelemetry {
    pub id: u64,
    pub kind: WallKind,
    pub p0: crate::Point2,
    pub p1: crate::Point2,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub enum GroundTruthTelemetry {
    State(GroundTruthStateTelemetry),
}

impl GroundTruthTelemetry {
    pub fn timestamp_us(&self) -> u64 {
        match self {
            GroundTruthTelemetry::State(v) => v.timestamp_us,
        }
    }
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct GroundTruthStateTelemetry {
    pub timestamp_us: u64,
    pub frame_id: String,
    pub pose: crate::Pose2,
    pub vx_ms: f32,
    pub vy_ms: f32,
    pub yaw_rate_rad_s: f32,
    pub steering_rad: f32,
    pub throttle: f32,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct StartGateTelemetry {
    pub timestamp_us: u64,
    pub state: String,
    pub allow_actuation: bool,
    pub reason: String,
    pub mode: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum TelemetryTopic {
    SensorImu,
    SensorLidar,
    SensorSonar,
    ControlCommand,
    ControlMetrics,
    ControlSafety,
    ControlStartGate,
    PerceptionReactiveCorridor,
    PerceptionReactiveRansacWalls,
    MappingRansacWalls,
    EstimationEkfState,
    EstimationEkfBias,
    EstimationEkfCovariance,
    MappingGlobalMapDelta,
    MappingGlobalMapSnapshot,
    MappingStatus,
    MappingLoopClosure,
    RaceStartLine,
    RaceLap,
    PlanningRaceLine,
    PlanningRaceReference,
    SupervisorState,
    SupervisorTransition,
    LocalizationScanMatch,
    LocalizationParticleFilter,
    LocalizationResult,
    WorldSnapshot,
    WorldWalls,
    SimGroundTruthState,
    SimGroundTruthPose,
    EstimationEkfPose,
}

impl TelemetryTopic {
    pub fn as_str(self) -> &'static str {
        match self {
            TelemetryTopic::SensorImu => "/sensor/imu",
            TelemetryTopic::SensorLidar => "/sensor/lidar",
            TelemetryTopic::SensorSonar => "/sensor/sonar",
            TelemetryTopic::ControlCommand => "/control/command",
            TelemetryTopic::ControlMetrics => "/control/metrics",
            TelemetryTopic::ControlSafety => "/control/safety",
            TelemetryTopic::ControlStartGate => "/control/start_gate",
            TelemetryTopic::PerceptionReactiveCorridor => "/perception/reactive/corridor",
            TelemetryTopic::PerceptionReactiveRansacWalls => "/perception/reactive/ransac_walls",
            TelemetryTopic::MappingRansacWalls => "/mapping/ransac_walls",
            TelemetryTopic::EstimationEkfState => "/estimation/ekf/state",
            TelemetryTopic::EstimationEkfBias => "/estimation/ekf/bias",
            TelemetryTopic::EstimationEkfCovariance => "/estimation/ekf/covariance",
            TelemetryTopic::MappingGlobalMapDelta => "/mapping/global_map_delta",
            TelemetryTopic::MappingGlobalMapSnapshot => "/mapping/global_map_snapshot",
            TelemetryTopic::MappingStatus => "/mapping/status",
            TelemetryTopic::MappingLoopClosure => "/mapping/loop_closure",
            TelemetryTopic::RaceStartLine => "/race/start_line",
            TelemetryTopic::RaceLap => "/race/lap",
            TelemetryTopic::PlanningRaceLine => "/planning/raceline",
            TelemetryTopic::PlanningRaceReference => "/planning/raceline_reference",
            TelemetryTopic::SupervisorState => "/supervisor/state",
            TelemetryTopic::SupervisorTransition => "/supervisor/transition",
            TelemetryTopic::LocalizationScanMatch => "/localization/scan_match",
            TelemetryTopic::LocalizationParticleFilter => "/localization/particle_filter",
            TelemetryTopic::LocalizationResult => "/localization/result",
            TelemetryTopic::WorldSnapshot => "/world/snapshot",
            TelemetryTopic::WorldWalls => "/world/walls",
            TelemetryTopic::SimGroundTruthState => "/sim/ground_truth/state",
            TelemetryTopic::SimGroundTruthPose => "/sim/ground_truth/pose",
            TelemetryTopic::EstimationEkfPose => "/estimation/ekf/pose",
        }
    }
}
