use nfe_algo::control::reactive_stanley::ReactiveStanleyController;
use nfe_algo::estimation::dead_reckon::DeadReckonEstimator;
use nfe_algo::estimation::ekf::Ekf;
use nfe_algo::localization::particle::ParticleLocalizer;
use nfe_algo::localization::scan_match::ScanMatchLocalizer;
use nfe_algo::perception::apex::{ApexCorridorPerception, ApexPerception};
use nfe_algo::perception::corridor::{CorridorPerception, RansacCorridorPerception};
use nfe_algo::perception::{ApexObservation, PerceptionObserver, RansacWallsObservation};
use nfe_algo::raceline::controller::RaceLineController;
use nfe_algo::raceline::solver::solve_min_curvature;
use nfe_algo::supervisor::{HealthReport, LoopClosureStatus, RaceSupervisor};
use nfe_core::control::{ControlInput, ControlOutput, Controller, CorridorEstimate};
use nfe_core::estimation::{StateEstimate, StateEstimator};
use nfe_core::localization::{LocalizationResult, Localizer};
use nfe_core::mapping::{MapStatus, MapperClient, MappingInput, TrackMap};
use nfe_core::raceline::{RaceLine, RaceLinePoint, RaceReference};
use nfe_core::sensors::{LidarPoint, SensorSnapshot};
use nfe_core::telemetry::{
    ApexFrame, ControlTelemetry, EkfStateTelemetry, EstimationTelemetry, EstimationTelemetryKind,
    LocalizationTelemetry, LocalizationTelemetryKind, MappingTelemetry, MappingTelemetryKind,
    MetricsTelemetry, PerceptionTelemetry, PerceptionTelemetryKind, PlanningTelemetry,
    PlanningTelemetryKind, RaceTelemetry, RaceTelemetryKind, RansacWallFitFrame,
    SupervisorStateTelemetry, SupervisorTelemetry, SupervisorTelemetryKind,
    SupervisorTransitionTelemetry, TelemetryEvent, WallFitSource,
};
use nfe_core::{wrap_angle, Point2, Pose2, WallLine};

use crate::config::RuntimeConfig;
use crate::mapping_worker::MappingWorker;
use crate::telemetry_bus::TelemetryBus;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EstimatorMode {
    DeadReckon,
    Ekf,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Deserialize, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum PerceptionMode {
    #[default]
    Corridor,
    Apex,
}

#[derive(Clone, Debug)]
pub struct StepOutput {
    pub estimate: StateEstimate,
    pub corridor: CorridorEstimate,
    pub command: ControlOutput,
    pub drive_mode: nfe_algo::supervisor::DriveMode,
    pub localization: LocalizationResult,
    pub map_revision: Option<u64>,
    pub raceline_revision: Option<u64>,
}

#[derive(Clone, Debug)]
struct ObservedRansacWalls {
    timestamp_us: u64,
    walls: Vec<WallLine>,
    points_total: usize,
    confidence: f32,
}

#[derive(Clone, Debug)]
struct ObservedApex {
    timestamp_us: u64,
    apex: LidarPoint,
    opposite: LidarPoint,
    target: LidarPoint,
    cartesian_midpoint: LidarPoint,
    range_jump_m: f32,
    derivative_score: f32,
    points_total: u32,
    confidence: f32,
}

#[derive(Default)]
struct RuntimePerceptionObserver {
    enabled: bool,
    ransac_walls: Option<ObservedRansacWalls>,
    apex: Option<ObservedApex>,
}

impl RuntimePerceptionObserver {
    fn begin_step(&mut self, enabled: bool) {
        self.enabled = enabled;
        self.ransac_walls = None;
        self.apex = None;
    }
}

impl PerceptionObserver for RuntimePerceptionObserver {
    fn wants_ransac_walls(&self) -> bool {
        self.enabled
    }

    fn ransac_walls(&mut self, event: RansacWallsObservation<'_>) {
        self.ransac_walls = Some(ObservedRansacWalls {
            timestamp_us: event.timestamp_us,
            walls: event.walls.to_vec(),
            points_total: event.points.len(),
            confidence: event.confidence,
        });
    }

    fn wants_apex(&self) -> bool {
        self.enabled
    }

    fn apex(&mut self, event: ApexObservation<'_>) {
        self.apex = Some(ObservedApex {
            timestamp_us: event.timestamp_us,
            apex: *event.apex,
            opposite: *event.opposite,
            target: *event.target,
            cartesian_midpoint: *event.cartesian_midpoint,
            range_jump_m: event.range_jump_m,
            derivative_score: event.derivative_score,
            points_total: event.filtered_points.len() as u32,
            confidence: event.confidence,
        });
    }
}

pub struct Pipeline {
    cfg: RuntimeConfig,
    telemetry: Option<TelemetryBus>,
    perception_observer: RuntimePerceptionObserver,
    tick: u64,
    last_mode: Option<nfe_algo::supervisor::DriveMode>,
    last_map_revision_published: Option<u64>,
    last_raceline_revision_published: Option<u64>,
    estimator_mode: EstimatorMode,
    dead_reckon: DeadReckonEstimator,
    ekf: Ekf,
    corridor_perception: RansacCorridorPerception,
    apex_perception: ApexCorridorPerception,
    mapper: MappingWorker,
    scan_match: ScanMatchLocalizer,
    particle: ParticleLocalizer,
    supervisor: RaceSupervisor,
    reactive: ReactiveStanleyController,
    raceline_controller: RaceLineController,
    raceline: Option<RaceLine>,
}

impl Pipeline {
    pub fn new(cfg: RuntimeConfig, estimator_mode: EstimatorMode) -> Self {
        let mapper = if cfg.mapping.enabled {
            MappingWorker::start(cfg.algo.mapper.clone(), cfg.mapping.queue_capacity, 0xA11CE)
        } else {
            MappingWorker::disabled()
        };
        Self {
            dead_reckon: DeadReckonEstimator::new(),
            ekf: Ekf::new(cfg.algo.ekf.clone()),
            corridor_perception: RansacCorridorPerception::new(
                cfg.algo.perception.clone(),
                0xC0FFEE,
            ),
            apex_perception: ApexCorridorPerception::new(cfg.algo.apex.clone()),
            mapper,
            scan_match: ScanMatchLocalizer::new(cfg.algo.scan_match.clone(), 0x0515_CA11),
            particle: ParticleLocalizer::new(cfg.algo.particle.clone(), 0x9EED),
            supervisor: RaceSupervisor::new(cfg.algo.supervisor.clone()),
            reactive: ReactiveStanleyController::new(cfg.algo.reactive.clone()),
            raceline_controller: RaceLineController::new(cfg.algo.raceline_controller.clone()),
            raceline: None,
            cfg,
            telemetry: None,
            perception_observer: RuntimePerceptionObserver::default(),
            tick: 0,
            last_mode: None,
            last_map_revision_published: None,
            last_raceline_revision_published: None,
            estimator_mode,
        }
    }

    pub fn with_telemetry(mut self, telemetry: TelemetryBus) -> Self {
        self.telemetry = Some(telemetry);
        self
    }

    pub fn set_telemetry(&mut self, telemetry: Option<TelemetryBus>) {
        self.telemetry = telemetry;
    }

    pub fn reset(&mut self, pose: Pose2, timestamp_us: u64) {
        self.dead_reckon.reset(pose, timestamp_us);
        self.ekf.reset(pose, timestamp_us);
        self.supervisor = RaceSupervisor::new(self.cfg.algo.supervisor.clone());
        self.apex_perception.reset();
        self.reactive.reset();
        self.raceline_controller.reset();
        self.raceline = None;
        self.tick = 0;
        self.last_mode = None;
        self.last_map_revision_published = None;
        self.last_raceline_revision_published = None;
        self.perception_observer.begin_step(false);
    }

    pub fn step(&mut self, snapshot: SensorSnapshot) -> StepOutput {
        self.predict(snapshot.imu);
        let mut estimate = self.estimate();
        let observe_perception = self.telemetry.as_ref().is_some_and(|bus| !bus.is_empty());
        self.perception_observer.begin_step(observe_perception);

        let corridor = match self.cfg.perception_mode {
            PerceptionMode::Corridor => {
                let points = snapshot.lidar.as_points2();
                self.corridor_perception.estimate_observed(
                    &points,
                    snapshot.lidar.timestamp_us,
                    &mut self.perception_observer,
                )
            }
            PerceptionMode::Apex => self.apex_perception.estimate_observed(
                &snapshot.lidar,
                snapshot.lidar.timestamp_us,
                &mut self.perception_observer,
            ),
        };

        if self.cfg.mapping.enabled {
            let _ = self.mapper.submit(MappingInput {
                cloud: snapshot.lidar.clone(),
                pose: estimate.pose,
                timestamp_us: snapshot.lidar.timestamp_us,
            });
        }
        if snapshot.start_line_crossed {
            self.mapper.mark_lap_complete();
        }

        let latest_map = self.mapper.latest_map();
        let mut localization = LocalizationResult::default();
        if let Some(map) = latest_map.as_ref().filter(|m| m.complete) {
            localization = self
                .scan_match
                .localize(&snapshot.lidar, estimate.pose, map);
            if localization.confidence < self.cfg.algo.supervisor.min_localization_confidence {
                localization = self.particle.localize(&snapshot.lidar, estimate.pose, map);
            }
            if let Some(meas) = localization.measurement {
                self.correct_pose(meas);
                estimate = self.estimate();
            }
            self.ensure_raceline(map);
        }

        let status = self.mapper.latest_status();
        let health = HealthReport {
            localization_confidence: estimate.confidence.max(localization.confidence),
            loop_closure: LoopClosureStatus {
                detected: status.loop_closure.detected,
                residual_m: status.loop_closure.residual_m,
                overlap: status.loop_closure.overlap,
            },
            estimator_diverged: estimate.diverged || snapshot.sensor_fault,
            raceline_ready: self.raceline.is_some(),
            mapping_enabled: self.cfg.mapping.enabled,
            start_line_crossed: snapshot.start_line_crossed,
        };
        let mode = self.supervisor.step(&health);

        let reference = self
            .raceline
            .as_ref()
            .and_then(|r| reference_for_pose(r, estimate.pose));
        let input = ControlInput {
            dt_s: self.cfg.dt_s(),
            pose: estimate.pose,
            motion: estimate.motion,
            estimate: &estimate,
            corridor: Some(&corridor),
            race_reference: reference.as_ref(),
        };
        let command = match mode {
            nfe_algo::supervisor::DriveMode::Reactive => self.reactive.compute(&input),
            nfe_algo::supervisor::DriveMode::RaceLine => self.raceline_controller.compute(&input),
        };

        let map_revision = latest_map.as_ref().map(|m| m.revision);
        let raceline_revision = self.raceline.as_ref().map(|r| r.revision);
        self.publish_step_telemetry(
            &snapshot,
            &corridor,
            estimate,
            &status,
            &health,
            mode,
            reference.as_ref(),
            &localization,
            latest_map.as_ref(),
            command,
        );
        self.tick = self.tick.saturating_add(1);

        StepOutput {
            estimate,
            corridor,
            command,
            drive_mode: mode,
            localization,
            map_revision,
            raceline_revision,
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn publish_step_telemetry(
        &mut self,
        snapshot: &SensorSnapshot,
        corridor: &CorridorEstimate,
        estimate: StateEstimate,
        map_status: &MapStatus,
        health: &HealthReport,
        mode: nfe_algo::supervisor::DriveMode,
        reference: Option<&RaceReference>,
        localization: &LocalizationResult,
        latest_map: Option<&TrackMap>,
        command: ControlOutput,
    ) {
        let Some(bus) = &self.telemetry else {
            return;
        };
        let ts = snapshot.lidar.timestamp_us.max(snapshot.imu.timestamp_us);

        bus.publish(TelemetryEvent::Sensor(
            nfe_core::telemetry::SensorTelemetry::Snapshot(snapshot.clone()),
        ));
        bus.publish(TelemetryEvent::Perception(PerceptionTelemetry {
            timestamp_us: ts,
            kind: PerceptionTelemetryKind::ReactiveCorridor(corridor.clone()),
        }));
        match self.cfg.perception_mode {
            PerceptionMode::Corridor => {
                if let Some(frame) = &self.perception_observer.ransac_walls {
                    bus.publish(TelemetryEvent::Perception(PerceptionTelemetry {
                        timestamp_us: frame.timestamp_us,
                        kind: PerceptionTelemetryKind::ReactiveRansacWalls(ransac_frame(frame)),
                    }));
                }
            }
            PerceptionMode::Apex => {
                if let Some(frame) = &self.perception_observer.apex {
                    bus.publish(TelemetryEvent::Perception(PerceptionTelemetry {
                        timestamp_us: frame.timestamp_us,
                        kind: PerceptionTelemetryKind::ReactiveApex(apex_frame(frame)),
                    }));
                }
            }
        }
        bus.publish(TelemetryEvent::Estimation(EstimationTelemetry {
            timestamp_us: ts,
            kind: EstimationTelemetryKind::EkfState(EkfStateTelemetry {
                estimate,
                accel_bias: [0.0, 0.0],
                gyro_bias_z: 0.0,
            }),
        }));
        bus.publish(TelemetryEvent::Mapping(MappingTelemetry {
            timestamp_us: ts,
            kind: MappingTelemetryKind::Status(*map_status),
        }));
        if map_status.loop_closure.detected {
            bus.publish(TelemetryEvent::Mapping(MappingTelemetry {
                timestamp_us: ts,
                kind: MappingTelemetryKind::LoopClosure(map_status.loop_closure),
            }));
        }
        if let Some(map) = latest_map {
            if self.last_map_revision_published != Some(map.revision) {
                self.last_map_revision_published = Some(map.revision);
                bus.publish(TelemetryEvent::Mapping(MappingTelemetry {
                    timestamp_us: ts,
                    kind: MappingTelemetryKind::GlobalMapSnapshot(map.clone()),
                }));
            }
        }
        bus.publish(TelemetryEvent::Localization(LocalizationTelemetry {
            timestamp_us: ts,
            kind: LocalizationTelemetryKind::Result(*localization),
        }));
        if let Some(reference) = reference {
            bus.publish(TelemetryEvent::Planning(PlanningTelemetry {
                timestamp_us: ts,
                kind: PlanningTelemetryKind::RaceReference(*reference),
            }));
        }
        if let Some(raceline) = &self.raceline {
            if self.last_raceline_revision_published != Some(raceline.revision) {
                self.last_raceline_revision_published = Some(raceline.revision);
                bus.publish(TelemetryEvent::Planning(PlanningTelemetry {
                    timestamp_us: ts,
                    kind: PlanningTelemetryKind::RaceLine(raceline.clone()),
                }));
            }
        }
        bus.publish(TelemetryEvent::Supervisor(SupervisorTelemetry {
            timestamp_us: ts,
            kind: SupervisorTelemetryKind::State(SupervisorStateTelemetry {
                drive_mode: format!("{mode:?}"),
                localization_confidence: health.localization_confidence,
                estimator_diverged: health.estimator_diverged,
                loop_closure_detected: health.loop_closure.detected,
                loop_closure_residual_m: health.loop_closure.residual_m,
                loop_closure_overlap: health.loop_closure.overlap,
                mapping_enabled: health.mapping_enabled,
                raceline_ready: health.raceline_ready,
            }),
        }));
        if self.last_mode != Some(mode) {
            let from = self
                .last_mode
                .map(|m| format!("{m:?}"))
                .unwrap_or_else(|| "None".to_string());
            let reason = if health.estimator_diverged {
                "EstimatorDiverged"
            } else if !health.mapping_enabled {
                "MappingDisabled"
            } else if health.raceline_ready {
                "RacelineReady"
            } else {
                "InitialOrReactive"
            };
            bus.publish(TelemetryEvent::Supervisor(SupervisorTelemetry {
                timestamp_us: ts,
                kind: SupervisorTelemetryKind::Transition(SupervisorTransitionTelemetry {
                    from_mode: from,
                    to_mode: format!("{mode:?}"),
                    reason: reason.to_string(),
                }),
            }));
            self.last_mode = Some(mode);
        }
        if snapshot.start_line_crossed {
            bus.publish(TelemetryEvent::Race(RaceTelemetry {
                timestamp_us: ts,
                kind: RaceTelemetryKind::StartLineGate { crossed: true },
            }));
        }
        bus.publish(TelemetryEvent::Control(ControlTelemetry {
            timestamp_us: ts,
            output: command,
            motion: estimate.motion,
        }));
    }

    pub fn publish_run_metrics(&self, metrics: MetricsTelemetry) {
        self.publish_event(TelemetryEvent::Metrics(metrics));
    }

    pub fn publish_event(&self, event: TelemetryEvent) {
        if let Some(bus) = &self.telemetry {
            bus.publish(event);
        }
    }

    pub fn publish_events(&self, events: impl IntoIterator<Item = TelemetryEvent>) {
        for event in events {
            self.publish_event(event);
        }
    }

    fn predict(&mut self, imu: nfe_core::estimation::ImuSample) {
        match self.estimator_mode {
            EstimatorMode::DeadReckon => self.dead_reckon.predict_imu(imu),
            EstimatorMode::Ekf => self.ekf.predict_imu(imu),
        }
    }

    fn correct_pose(&mut self, meas: nfe_core::estimation::PoseMeasurement) {
        match self.estimator_mode {
            EstimatorMode::DeadReckon => {
                let _ = self.dead_reckon.correct_pose(meas);
            }
            EstimatorMode::Ekf => {
                let _ = StateEstimator::correct_pose(&mut self.ekf, meas);
            }
        }
    }

    fn estimate(&self) -> StateEstimate {
        match self.estimator_mode {
            EstimatorMode::DeadReckon => self.dead_reckon.estimate(),
            EstimatorMode::Ekf => self.ekf.estimate(),
        }
    }

    fn ensure_raceline(&mut self, map: &TrackMap) {
        let needs_update = self
            .raceline
            .as_ref()
            .is_none_or(|r| r.revision != map.revision);
        if needs_update {
            if let Ok(line) = solve_min_curvature(map, &self.cfg.algo.raceline_solver) {
                self.raceline = Some(line);
            }
        }
    }
}

fn apex_frame(frame: &ObservedApex) -> ApexFrame {
    ApexFrame {
        frame_id: "base_link".to_string(),
        apex: Point2::new(frame.apex.x, frame.apex.y),
        opposite: Point2::new(frame.opposite.x, frame.opposite.y),
        target: Point2::new(frame.target.x, frame.target.y),
        cartesian_midpoint: Point2::new(frame.cartesian_midpoint.x, frame.cartesian_midpoint.y),
        apex_range_m: frame.apex.dist_m,
        opposite_range_m: frame.opposite.dist_m,
        target_range_m: frame.target.dist_m,
        apex_angle_rad: frame.apex.angle_rad,
        opposite_angle_rad: frame.opposite.angle_rad,
        target_angle_rad: frame.target.angle_rad,
        range_jump_m: frame.range_jump_m,
        derivative_score: frame.derivative_score,
        points_total: frame.points_total,
        confidence: frame.confidence,
    }
}

fn ransac_frame(frame: &ObservedRansacWalls) -> RansacWallFitFrame {
    let points_total = frame.points_total as u32;
    let inliers_total = frame
        .walls
        .iter()
        .map(|w| (w.support.clamp(0.0, 1.0) * points_total as f32).round() as u32)
        .sum();
    RansacWallFitFrame {
        frame_id: "base_link".to_string(),
        source: WallFitSource::Reactive,
        walls: frame.walls.clone(),
        points_total,
        inliers_total,
        confidence: frame.confidence,
    }
}

fn reference_for_pose(line: &RaceLine, pose: Pose2) -> Option<RaceReference> {
    let target = nearest_point(line, Point2::new(pose.x, pose.y))?;
    let dx = pose.x - target.p.x;
    let dy = pose.y - target.p.y;
    let lateral_error_m = -target.yaw.sin() * dx + target.yaw.cos() * dy;
    let heading_error_rad = wrap_angle(target.yaw - pose.yaw);
    Some(RaceReference {
        target,
        lateral_error_m,
        heading_error_rad,
        lookahead_m: 0.0,
        confidence: 1.0,
    })
}

fn nearest_point(line: &RaceLine, p: Point2) -> Option<RaceLinePoint> {
    line.points.iter().copied().min_by(|a, b| {
        a.p.dist(&p)
            .partial_cmp(&b.p.dist(&p))
            .unwrap_or(std::cmp::Ordering::Equal)
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use nfe_core::estimation::ImuSample;
    use nfe_core::sensors::{LidarCloud, LidarPoint, SensorSnapshot};

    fn corridor_snapshot(ts: u64) -> SensorSnapshot {
        let mut cloud = LidarCloud {
            points: Vec::new(),
            timestamp_us: ts,
        };
        for i in 0..40 {
            let x = i as f32 * 0.05;
            cloud.points.push(LidarPoint {
                x,
                y: 0.5,
                dist_m: x.hypot(0.5),
                angle_rad: 0.0,
                timestamp_us: ts,
            });
            cloud.points.push(LidarPoint {
                x,
                y: -0.5,
                dist_m: x.hypot(0.5),
                angle_rad: 0.0,
                timestamp_us: ts,
            });
        }
        SensorSnapshot {
            lidar: cloud,
            imu: ImuSample {
                timestamp_us: ts,
                ..Default::default()
            },
            sensor_fault: false,
            sonar_m: [f32::MAX; 3],
            start_line_crossed: false,
        }
    }

    fn apex_snapshot(ts: u64) -> SensorSnapshot {
        let mut cloud = LidarCloud {
            points: Vec::new(),
            timestamp_us: ts,
        };
        for p in [
            Point2::new(0.5, -0.8),
            Point2::new(1.0, -0.8),
            Point2::new(1.5, -0.8),
            Point2::new(1.0, 0.8),
            Point2::new(1.0, 2.0),
            Point2::new(1.5, 2.1),
        ] {
            cloud.points.push(LidarPoint {
                x: p.x,
                y: p.y,
                dist_m: p.x.hypot(p.y),
                angle_rad: p.y.atan2(p.x),
                timestamp_us: ts,
            });
        }
        SensorSnapshot {
            lidar: cloud,
            imu: ImuSample {
                timestamp_us: ts,
                ..Default::default()
            },
            sensor_fault: false,
            sonar_m: [f32::MAX; 3],
            start_line_crossed: false,
        }
    }

    #[test]
    fn pipeline_reactive_step_produces_finite_command() {
        let mut cfg = RuntimeConfig::default();
        cfg.mapping.enabled = false;
        let mut p = Pipeline::new(cfg, EstimatorMode::DeadReckon);
        p.reset(Pose2::default(), 0);
        let out = p.step(corridor_snapshot(10_000));
        assert!(out.command.steering_rad.is_finite());
        assert!(out.command.throttle.is_finite());
        assert_eq!(out.drive_mode, nfe_algo::supervisor::DriveMode::Reactive);
    }

    #[test]
    fn pipeline_apex_perception_step_produces_finite_command() {
        let mut cfg = RuntimeConfig::default();
        cfg.mapping.enabled = false;
        cfg.perception_mode = PerceptionMode::Apex;
        cfg.algo.apex.median_window = 1;
        cfg.algo.apex.min_points = 4;
        cfg.algo.apex.min_range_jump_m = 0.2;
        let mut p = Pipeline::new(cfg, EstimatorMode::DeadReckon);
        p.reset(Pose2::default(), 0);
        let out = p.step(apex_snapshot(10_000));
        assert!(out.command.steering_rad.is_finite());
        assert!(out.command.throttle.is_finite());
        assert!(out.corridor.confidence > 0.0, "corridor={:?}", out.corridor);
    }
}
