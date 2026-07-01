use nfe_algo::control::reactive_stanley::ReactiveStanleyController;
use nfe_algo::estimation::ekf::Ekf;
use nfe_algo::localization::correlative::CorrelativeLocalizer;
use nfe_algo::localization::particle::ParticleLocalizer;
use nfe_algo::perception::apex::{ApexCorridorPerception, ApexPerception};
use nfe_algo::perception::corridor::{CorridorPerception, RansacCorridorPerception};
use nfe_algo::perception::{ApexObservation, PerceptionObserver, RansacWallsObservation};
use nfe_algo::raceline::controller::RaceLineController;
use nfe_algo::raceline::solver::RaceLineError;
use nfe_algo::supervisor::{HealthReport, LoopClosureStatus, RaceSupervisor};
use nfe_core::control::{
    ControlInput, ControlOutput, Controller, ControllerStatus, CorridorEstimate,
};
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
use crate::raceline_worker::{RaceLinePlannerEvent, RaceLinePlannerSubmit, RaceLinePlannerWorker};
use crate::telemetry_bus::TelemetryBus;

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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct PlannerPolicy {
    max_consecutive_failures: u32,
    max_outdated_ticks: u32,
    max_revision_lag: u64,
}

impl PlannerPolicy {
    fn for_hz(hz: u64) -> Self {
        Self {
            max_consecutive_failures: 3,
            max_outdated_ticks: hz.clamp(1, u32::MAX as u64) as u32,
            max_revision_lag: 10,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
enum PlannerAvailability {
    NoLine,
    Fresh,
    UsableStale {
        revision_lag: u64,
        outdated_ticks: u32,
        consecutive_failures: u32,
    },
    UnusableStale {
        reason: StaleReason,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum StaleReason {
    SolverFailedWithoutPrevious,
    TooManyFailures,
    TooOldInTicks,
    MapRevisionLagTooLarge,
    /// Detects backward/reset revisions. Equal revision numbers with different
    /// map contents require a future map-session identifier to distinguish.
    MapRevisionInconsistent,
}

#[derive(Clone, Debug)]
struct PlannerState {
    line: Option<RaceLine>,
    source_map_revision: Option<u64>,
    availability: PlannerAvailability,
    last_attempted_map_revision: Option<u64>,
    last_failed_map_revision: Option<u64>,
    last_error: Option<RaceLineError>,
    consecutive_failures: u32,
    outdated_ticks: u32,
    policy: PlannerPolicy,
}

impl PlannerState {
    fn new(policy: PlannerPolicy) -> Self {
        Self {
            line: None,
            source_map_revision: None,
            availability: PlannerAvailability::NoLine,
            last_attempted_map_revision: None,
            last_failed_map_revision: None,
            last_error: None,
            consecutive_failures: 0,
            outdated_ticks: 0,
            policy,
        }
    }

    fn usable_line(&self) -> Option<&RaceLine> {
        if !matches!(
            self.availability,
            PlannerAvailability::Fresh | PlannerAvailability::UsableStale { .. }
        ) {
            return None;
        }
        self.line.as_ref().filter(|line| !line.points.is_empty())
    }

    fn should_attempt(&self, map_revision: u64) -> bool {
        self.source_map_revision != Some(map_revision)
            && self.last_attempted_map_revision != Some(map_revision)
    }

    fn record_attempt_started(&mut self, map_revision: u64) {
        self.last_attempted_map_revision = Some(
            self.last_attempted_map_revision
                .map_or(map_revision, |last| last.max(map_revision)),
        );
    }

    fn advance_for_map_revision(&mut self, map_revision: u64) {
        let Some(source_revision) = self.source_map_revision else {
            return;
        };
        if map_revision < source_revision {
            self.line = None;
            self.source_map_revision = None;
            self.outdated_ticks = 0;
            self.availability = PlannerAvailability::UnusableStale {
                reason: StaleReason::MapRevisionInconsistent,
            };
            return;
        }
        if map_revision == source_revision {
            self.outdated_ticks = 0;
            if self
                .line
                .as_ref()
                .is_some_and(|line| !line.points.is_empty())
            {
                self.availability = PlannerAvailability::Fresh;
            }
            return;
        }

        self.outdated_ticks = self.outdated_ticks.saturating_add(1);
        self.evaluate_stale_policy(map_revision);
    }

    fn record_success(&mut self, line: RaceLine, map_revision: u64) {
        self.line = Some(line);
        self.source_map_revision = Some(map_revision);
        self.availability = PlannerAvailability::Fresh;
        self.record_attempt_started(map_revision);
        self.last_failed_map_revision = None;
        self.last_error = None;
        self.consecutive_failures = 0;
        self.outdated_ticks = 0;
    }

    fn record_failure(&mut self, map_revision: u64, error: RaceLineError) {
        self.record_attempt_started(map_revision);
        self.last_failed_map_revision = Some(map_revision);
        self.last_error = Some(error);
        self.consecutive_failures = self.consecutive_failures.saturating_add(1);

        if self.line.is_none() || self.source_map_revision.is_none() {
            self.availability = PlannerAvailability::UnusableStale {
                reason: StaleReason::SolverFailedWithoutPrevious,
            };
            return;
        }

        self.evaluate_stale_policy(map_revision);
    }

    fn evaluate_stale_policy(&mut self, map_revision: u64) {
        let Some(source_revision) = self.source_map_revision else {
            self.availability = PlannerAvailability::NoLine;
            return;
        };
        if map_revision < source_revision {
            self.line = None;
            self.source_map_revision = None;
            self.outdated_ticks = 0;
            self.availability = PlannerAvailability::UnusableStale {
                reason: StaleReason::MapRevisionInconsistent,
            };
            return;
        }
        let revision_lag = map_revision - source_revision;
        if revision_lag == 0 {
            self.availability = PlannerAvailability::Fresh;
            return;
        }
        if self.consecutive_failures > self.policy.max_consecutive_failures {
            self.availability = PlannerAvailability::UnusableStale {
                reason: StaleReason::TooManyFailures,
            };
            return;
        }
        if self.outdated_ticks > self.policy.max_outdated_ticks {
            self.availability = PlannerAvailability::UnusableStale {
                reason: StaleReason::TooOldInTicks,
            };
            return;
        }
        if revision_lag > self.policy.max_revision_lag {
            self.availability = PlannerAvailability::UnusableStale {
                reason: StaleReason::MapRevisionLagTooLarge,
            };
            return;
        }
        self.availability = PlannerAvailability::UsableStale {
            revision_lag,
            outdated_ticks: self.outdated_ticks,
            consecutive_failures: self.consecutive_failures,
        };
    }
}

pub struct Pipeline {
    cfg: RuntimeConfig,
    telemetry: Option<TelemetryBus>,
    perception_observer: RuntimePerceptionObserver,
    tick: u64,
    last_mode: Option<nfe_algo::supervisor::DriveMode>,
    last_slam_lidar_timestamp_us: Option<u64>,
    last_map_revision_published: Option<u64>,
    last_raceline_revision_published: Option<u64>,
    last_localization_confidence: Option<f32>,
    ekf: Ekf,
    corridor_perception: RansacCorridorPerception,
    apex_perception: ApexCorridorPerception,
    mapper: MappingWorker,
    correlative: CorrelativeLocalizer,
    particle: ParticleLocalizer,
    supervisor: RaceSupervisor,
    reactive: ReactiveStanleyController,
    raceline_controller: RaceLineController,
    raceline_worker: RaceLinePlannerWorker,
    planner: PlannerState,
}

impl Pipeline {
    pub fn new(cfg: RuntimeConfig) -> Self {
        let mapper = if cfg.mapping.enabled {
            MappingWorker::start(cfg.algo.mapper.clone(), cfg.mapping.queue_capacity, 0xA11CE)
        } else {
            MappingWorker::disabled()
        };
        let planner = PlannerState::new(PlannerPolicy::for_hz(cfg.hz));
        let raceline_worker = RaceLinePlannerWorker::start(cfg.algo.raceline_solver.clone());
        Self {
            ekf: Ekf::new(cfg.algo.ekf.clone()),
            corridor_perception: RansacCorridorPerception::new(
                cfg.algo.perception.clone(),
                0xC0FFEE,
            ),
            apex_perception: ApexCorridorPerception::new(cfg.algo.apex.clone()),
            mapper,
            correlative: CorrelativeLocalizer::new(cfg.algo.correlative.clone()),
            particle: ParticleLocalizer::new(cfg.algo.particle.clone(), 0x9EED),
            supervisor: RaceSupervisor::new(cfg.algo.supervisor.clone()),
            reactive: ReactiveStanleyController::new(cfg.algo.reactive.clone()),
            raceline_controller: RaceLineController::new(cfg.algo.raceline_controller.clone()),
            raceline_worker,
            planner,
            cfg,
            telemetry: None,
            perception_observer: RuntimePerceptionObserver::default(),
            tick: 0,
            last_mode: None,
            last_slam_lidar_timestamp_us: None,
            last_map_revision_published: None,
            last_raceline_revision_published: None,
            last_localization_confidence: None,
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
        self.ekf.reset(pose, timestamp_us);
        self.supervisor = RaceSupervisor::new(self.cfg.algo.supervisor.clone());
        self.apex_perception.reset();
        self.reactive.reset();
        self.raceline_controller.reset();
        self.raceline_worker.shutdown();
        self.raceline_worker = RaceLinePlannerWorker::start(self.cfg.algo.raceline_solver.clone());
        self.planner = PlannerState::new(PlannerPolicy::for_hz(self.cfg.hz));
        self.tick = 0;
        self.last_mode = None;
        self.last_slam_lidar_timestamp_us = None;
        self.last_map_revision_published = None;
        self.last_raceline_revision_published = None;
        self.last_localization_confidence = None;
        self.perception_observer.begin_step(false);
    }

    pub fn step(&mut self, snapshot: SensorSnapshot) -> StepOutput {
        self.predict(snapshot.imu);
        let mut estimate = self.estimate();
        let observe_perception = self.telemetry.as_ref().is_some_and(|bus| !bus.is_empty());
        self.perception_observer.begin_step(observe_perception);

        let corridor = match self.cfg.perception_mode {
            PerceptionMode::Corridor => {
                let points: Vec<_> = snapshot.lidar.points.iter().map(|p| p.point2()).collect();
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

        let new_lidar_scan = self.last_slam_lidar_timestamp_us != Some(snapshot.lidar.timestamp_us);
        if new_lidar_scan {
            self.last_slam_lidar_timestamp_us = Some(snapshot.lidar.timestamp_us);
            if self.cfg.mapping.enabled {
                let _ = self.mapper.submit(MappingInput {
                    cloud: snapshot.lidar.clone(),
                    pose: estimate.pose,
                    timestamp_us: snapshot.lidar.timestamp_us,
                });
            }
        }
        if snapshot.start_line_crossed {
            self.mapper.mark_lap_complete();
        }

        let latest_map = self.mapper.latest_map();
        let mut localization = LocalizationResult::default();
        if new_lidar_scan {
            if let Some(map) = latest_map.as_ref().filter(|m| m.complete) {
                localization = self
                    .correlative
                    .localize(&snapshot.lidar, estimate.pose, map);
                if localization.confidence < self.cfg.algo.supervisor.min_localization_confidence {
                    localization = self.particle.localize(&snapshot.lidar, estimate.pose, map);
                }
                self.last_localization_confidence = Some(localization.confidence);
                if let Some(meas) = localization.measurement {
                    self.correct_pose(meas);
                    estimate = self.estimate();
                }
            }
        }
        self.drain_raceline_worker_events();
        if let Some(map) = latest_map.as_ref().filter(|m| m.complete) {
            self.ensure_raceline(map);
        }

        let status = self.mapper.latest_status();
        let localization_health_confidence = latest_map
            .as_ref()
            .filter(|m| m.complete)
            .and(self.last_localization_confidence)
            .map_or(estimate.confidence, |loc| loc.min(estimate.confidence));
        let health = HealthReport {
            localization_confidence: localization_health_confidence,
            loop_closure: LoopClosureStatus {
                detected: status.loop_closure.detected,
                residual_m: status.loop_closure.residual_m,
                overlap: status.loop_closure.overlap,
            },
            estimator_diverged: estimate.diverged || snapshot.sensor_fault,
            raceline_ready: self.planner.usable_line().is_some(),
            mapping_enabled: self.cfg.mapping.enabled,
            start_line_crossed: snapshot.start_line_crossed,
        };
        let requested_mode = self.supervisor.step(&health);

        let reference = self
            .planner
            .usable_line()
            .and_then(|r| reference_for_pose(r, estimate.pose));
        let input = ControlInput {
            dt_s: self.cfg.dt_s(),
            pose: estimate.pose,
            motion: estimate.motion,
            estimate: &estimate,
            corridor: Some(&corridor),
            race_reference: reference.as_ref(),
        };
        let (mode, command) = match requested_mode {
            nfe_algo::supervisor::DriveMode::Reactive => {
                (requested_mode, self.reactive.compute(&input))
            }
            nfe_algo::supervisor::DriveMode::RaceLine => {
                let command = self.raceline_controller.compute(&input);
                if command.status == ControllerStatus::Unavailable {
                    (
                        nfe_algo::supervisor::DriveMode::Reactive,
                        self.reactive.compute(&input),
                    )
                } else {
                    (requested_mode, command)
                }
            }
        };

        let map_revision = latest_map.as_ref().map(|m| m.revision);
        let raceline_revision = self.planner.usable_line().map(|r| r.revision);
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
        if let Some(raceline) = self.planner.usable_line() {
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
        self.ekf.predict_imu(imu);
    }

    fn correct_pose(&mut self, meas: nfe_core::estimation::PoseMeasurement) {
        let _ = StateEstimator::correct_pose(&mut self.ekf, meas);
    }

    fn estimate(&self) -> StateEstimate {
        self.ekf.estimate()
    }

    fn drain_raceline_worker_events(&mut self) {
        while let Some(event) = self.raceline_worker.poll_event() {
            match event {
                RaceLinePlannerEvent::Started { .. } => {}
                RaceLinePlannerEvent::Completed { revision, line, .. } => {
                    self.planner.record_success(line, revision)
                }
                RaceLinePlannerEvent::Failed {
                    revision, error, ..
                } => self.planner.record_failure(revision, error),
            }
        }
    }

    fn ensure_raceline(&mut self, map: &TrackMap) {
        self.planner.advance_for_map_revision(map.revision);
        if !self.planner.should_attempt(map.revision) {
            return;
        }
        match self.raceline_worker.submit_latest(map.clone()) {
            RaceLinePlannerSubmit::Accepted
            | RaceLinePlannerSubmit::Duplicate
            | RaceLinePlannerSubmit::ReplacedPending => {
                self.planner.record_attempt_started(map.revision)
            }
            RaceLinePlannerSubmit::BusyCurrentKept | RaceLinePlannerSubmit::Disabled => {}
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

    fn l_shape_world_points() -> Vec<Point2> {
        let mut points = Vec::new();
        for i in 0..=30 {
            let t = -1.5 + i as f32 * 0.10;
            points.push(Point2::new(1.8, t));
        }
        for i in 0..=25 {
            let t = -0.7 + i as f32 * 0.10;
            points.push(Point2::new(t, 1.2));
        }
        points
    }

    fn scan_from_pose(pose: Pose2, ts: u64) -> LidarCloud {
        let (s, c) = pose.yaw.sin_cos();
        let points = l_shape_world_points()
            .into_iter()
            .map(|world| {
                let dx = world.x - pose.x;
                let dy = world.y - pose.y;
                let x = c * dx + s * dy;
                let y = -s * dx + c * dy;
                LidarPoint {
                    x,
                    y,
                    dist_m: x.hypot(y),
                    angle_rad: y.atan2(x),
                    timestamp_us: ts,
                }
            })
            .collect();
        LidarCloud {
            points,
            timestamp_us: ts,
        }
    }

    fn l_shape_snapshot(scan_pose: Pose2, ts: u64, ax: f32) -> SensorSnapshot {
        SensorSnapshot {
            lidar: scan_from_pose(scan_pose, ts),
            imu: ImuSample {
                ax,
                timestamp_us: ts,
                ..Default::default()
            },
            sensor_fault: false,
            sonar_m: [f32::MAX; 3],
            start_line_crossed: false,
        }
    }

    fn simple_raceline(revision: u64) -> RaceLine {
        RaceLine {
            points: vec![
                RaceLinePoint {
                    p: Point2::new(0.0, 0.0),
                    yaw: 0.0,
                    curvature: 0.0,
                    speed_ms: 1.0,
                    accel_x_ms2: 0.0,
                    s_m: 0.0,
                },
                RaceLinePoint {
                    p: Point2::new(1.0, 0.0),
                    yaw: 0.0,
                    curvature: 0.0,
                    speed_ms: 1.0,
                    accel_x_ms2: 0.0,
                    s_m: 1.0,
                },
                RaceLinePoint {
                    p: Point2::new(2.0, 0.0),
                    yaw: 0.0,
                    curvature: 0.0,
                    speed_ms: 1.0,
                    accel_x_ms2: 0.0,
                    s_m: 2.0,
                },
            ],
            closed: true,
            revision,
        }
    }

    fn planner_policy() -> PlannerPolicy {
        PlannerPolicy {
            max_consecutive_failures: 1,
            max_outdated_ticks: 1,
            max_revision_lag: 1,
        }
    }

    #[test]
    fn planner_success_marks_line_fresh() {
        let mut planner = PlannerState::new(planner_policy());

        planner.record_success(simple_raceline(7), 7);

        assert!(matches!(planner.availability, PlannerAvailability::Fresh));
        assert_eq!(planner.source_map_revision, Some(7));
        assert_eq!(planner.usable_line().map(|line| line.revision), Some(7));
        assert_eq!(planner.consecutive_failures, 0);
    }

    #[test]
    fn planner_success_resets_failure_and_stale_counters() {
        let mut planner = PlannerState::new(PlannerPolicy {
            max_consecutive_failures: 5,
            max_outdated_ticks: 5,
            max_revision_lag: 5,
        });
        planner.record_success(simple_raceline(1), 1);
        planner.advance_for_map_revision(2);
        planner.record_failure(2, RaceLineError::EmptyMap);

        assert_eq!(planner.consecutive_failures, 1);
        assert_eq!(planner.outdated_ticks, 1);

        planner.record_success(simple_raceline(2), 2);

        assert_eq!(planner.consecutive_failures, 0);
        assert_eq!(planner.outdated_ticks, 0);
        assert!(matches!(planner.availability, PlannerAvailability::Fresh));
        assert_eq!(planner.usable_line().map(|line| line.revision), Some(2));
    }

    #[test]
    fn planner_failure_without_previous_line_is_unusable() {
        let mut planner = PlannerState::new(planner_policy());

        planner.record_failure(1, RaceLineError::EmptyMap);

        assert_eq!(planner.usable_line().map(|line| line.revision), None);
        assert!(matches!(
            planner.availability,
            PlannerAvailability::UnusableStale {
                reason: StaleReason::SolverFailedWithoutPrevious
            }
        ));
        assert_eq!(planner.last_failed_map_revision, Some(1));
        assert_eq!(planner.consecutive_failures, 1);
    }

    #[test]
    fn planner_failure_with_previous_line_stays_usable_within_policy() {
        let mut planner = PlannerState::new(PlannerPolicy {
            max_consecutive_failures: 2,
            max_outdated_ticks: 5,
            max_revision_lag: 5,
        });
        planner.record_success(simple_raceline(1), 1);
        planner.advance_for_map_revision(2);

        planner.record_failure(2, RaceLineError::EmptyMap);

        assert_eq!(planner.usable_line().map(|line| line.revision), Some(1));
        assert!(matches!(
            planner.availability,
            PlannerAvailability::UsableStale {
                revision_lag: 1,
                outdated_ticks: 1,
                consecutive_failures: 1,
            }
        ));
    }

    #[test]
    fn planner_failure_with_previous_line_exceeding_policy_is_unusable() {
        let mut planner = PlannerState::new(planner_policy());
        planner.record_success(simple_raceline(1), 1);
        planner.advance_for_map_revision(2);
        planner.record_failure(2, RaceLineError::EmptyMap);

        planner.advance_for_map_revision(3);
        planner.record_failure(3, RaceLineError::InsufficientBoundaries);

        assert_eq!(planner.usable_line().map(|line| line.revision), None);
        assert!(matches!(
            planner.availability,
            PlannerAvailability::UnusableStale {
                reason: StaleReason::TooManyFailures
            }
        ));
    }

    #[test]
    fn planner_line_becomes_unusable_when_stale_too_long() {
        let mut planner = PlannerState::new(planner_policy());
        planner.record_success(simple_raceline(1), 1);

        planner.advance_for_map_revision(2);
        assert_eq!(planner.usable_line().map(|line| line.revision), Some(1));
        planner.advance_for_map_revision(2);

        assert_eq!(planner.usable_line().map(|line| line.revision), None);
        assert!(matches!(
            planner.availability,
            PlannerAvailability::UnusableStale {
                reason: StaleReason::TooOldInTicks
            }
        ));
    }

    #[test]
    fn planner_line_becomes_unusable_when_revision_lag_is_too_large() {
        let mut planner = PlannerState::new(planner_policy());
        planner.record_success(simple_raceline(1), 1);

        planner.advance_for_map_revision(3);

        assert_eq!(planner.usable_line().map(|line| line.revision), None);
        assert!(matches!(
            planner.availability,
            PlannerAvailability::UnusableStale {
                reason: StaleReason::MapRevisionLagTooLarge
            }
        ));
    }

    #[test]
    fn planner_revision_reset_invalidates_previous_line() {
        let mut planner = PlannerState::new(planner_policy());
        planner.record_success(simple_raceline(10), 10);

        planner.advance_for_map_revision(1);

        assert_eq!(planner.usable_line().map(|line| line.revision), None);
        assert_eq!(planner.source_map_revision, None);
        assert!(matches!(
            planner.availability,
            PlannerAvailability::UnusableStale {
                reason: StaleReason::MapRevisionInconsistent
            }
        ));
    }

    fn slam_test_config() -> RuntimeConfig {
        let mut cfg = RuntimeConfig::default();
        cfg.mapping.enabled = true;
        cfg.mapping.queue_capacity = 8;
        cfg.perception_mode = PerceptionMode::Corridor;
        cfg.algo.mapper.resolution_m = 0.05;
        cfg.algo.mapper.width_m = 8.0;
        cfg.algo.mapper.height_m = 8.0;
        cfg.algo.mapper.origin_x_m = -4.0;
        cfg.algo.mapper.origin_y_m = -4.0;
        cfg.algo.correlative.search_window_xy_m = 0.6;
        cfg.algo.correlative.search_window_yaw_rad = 0.3;
        cfg.algo.correlative.coarse_xy_step_m = 0.10;
        cfg.algo.correlative.coarse_yaw_step_rad = 0.10;
        cfg.algo.correlative.fine_xy_step_m = 0.02;
        cfg.algo.correlative.fine_yaw_step_rad = 0.02;
        cfg.algo.correlative.min_points = 8;
        cfg.algo.correlative.min_confidence = 0.10;
        cfg.algo.ekf.r_pos = 0.01;
        cfg.algo.ekf.r_yaw = 0.01;
        cfg.algo.ekf.pose_gate = 100.0;
        cfg
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
        let mut p = Pipeline::new(cfg);
        p.reset(Pose2::default(), 0);
        let out = p.step(corridor_snapshot(10_000));
        assert!(out.command.steering_rad.is_finite());
        assert!(out.command.throttle.is_finite());
        assert_eq!(out.drive_mode, nfe_algo::supervisor::DriveMode::Reactive);
    }

    fn wait_for_complete_map(p: &Pipeline) -> Option<TrackMap> {
        for _ in 0..100 {
            let map = p.mapper.latest_map();
            if map.as_ref().is_some_and(|m| m.complete) {
                return map;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        p.mapper.latest_map()
    }

    #[test]
    fn pipeline_apex_perception_step_produces_finite_command() {
        let mut cfg = RuntimeConfig::default();
        cfg.mapping.enabled = false;
        cfg.perception_mode = PerceptionMode::Apex;
        cfg.algo.apex.median_window = 1;
        cfg.algo.apex.min_points = 4;
        cfg.algo.apex.min_range_jump_m = 0.2;
        cfg.algo.apex.min_lookahead_m = 8.0;
        cfg.algo.apex.max_lookahead_m = 8.0;
        cfg.algo.apex.lookahead_sensitivity = 0.0;
        let mut p = Pipeline::new(cfg);
        p.reset(Pose2::default(), 0);
        let out = p.step(apex_snapshot(10_000));
        assert!(out.command.steering_rad.is_finite());
        assert!(out.command.throttle.is_finite());
        assert!(out.corridor.confidence > 0.0, "corridor={:?}", out.corridor);
    }

    #[test]
    fn slam_work_is_deduped_by_lidar_timestamp() {
        let mut cfg = RuntimeConfig::default();
        cfg.mapping.enabled = true;
        cfg.mapping.queue_capacity = 8;
        cfg.perception_mode = PerceptionMode::Corridor;
        let mut p = Pipeline::new(cfg);
        p.reset(Pose2::default(), 0);

        let _ = p.step(corridor_snapshot(10_000));
        let _ = p.step(corridor_snapshot(10_000));
        let _ = p.step(corridor_snapshot(20_000));
        p.mapper.shutdown();

        let status = p.mapper.latest_status();
        assert_eq!(status.submitted_scans, 2, "status={status:?}");
        assert_eq!(status.processed_scans, 2, "status={status:?}");
    }

    #[test]
    fn start_line_edge_marks_mapping_lap_complete() {
        let mut cfg = RuntimeConfig::default();
        cfg.mapping.enabled = true;
        cfg.mapping.queue_capacity = 8;
        cfg.perception_mode = PerceptionMode::Corridor;
        let mut p = Pipeline::new(cfg);
        p.reset(Pose2::default(), 0);

        let _ = p.step(corridor_snapshot(10_000));
        let mut lap_edge = corridor_snapshot(20_000);
        lap_edge.start_line_crossed = true;
        let _ = p.step(lap_edge);
        p.mapper.shutdown();

        let map = p.mapper.latest_map().expect("map should be published");
        assert!(map.complete);
    }

    #[test]
    fn ekf_is_corrected_by_correlative_localization() {
        let cfg = slam_test_config();
        let mut p = Pipeline::new(cfg);
        p.reset(Pose2::default(), 1);

        let mut map_seed = l_shape_snapshot(Pose2::default(), 10_000, 0.0);
        map_seed.start_line_crossed = true;
        let _ = p.step(map_seed);
        let map = wait_for_complete_map(&p).expect("complete map");
        assert!(map.distance_field.is_some());

        let _ = p.step(l_shape_snapshot(Pose2::default(), 1_010_000, 0.4));
        let corrected = p.step(l_shape_snapshot(Pose2::default(), 2_010_000, 0.0));

        assert!(
            corrected.estimate.pose.x.abs() < 0.08,
            "estimate={:?} localization={:?}",
            corrected.estimate,
            corrected.localization
        );
        assert!(corrected.localization.confidence > 0.5);
    }

    #[test]
    fn mcl_stays_dormant_when_primary_localizer_is_confident() {
        let mut cfg = slam_test_config();
        cfg.algo.supervisor.min_localization_confidence = 0.5;
        let mut p = Pipeline::new(cfg);
        p.reset(Pose2::default(), 1);

        let mut map_seed = l_shape_snapshot(Pose2::default(), 10_000, 0.0);
        map_seed.start_line_crossed = true;
        let _ = p.step(map_seed);
        let _ = wait_for_complete_map(&p).expect("complete map");
        let before = p.particle.update_count();

        let out = p.step(l_shape_snapshot(Pose2::default(), 20_000, 0.0));

        assert!(
            out.localization.confidence > 0.5,
            "localization={:?}",
            out.localization
        );
        assert_eq!(p.particle.update_count(), before);
    }

    fn prime_supervisor_for_raceline(p: &mut Pipeline) {
        let mode = p.supervisor.step(&HealthReport {
            localization_confidence: 1.0,
            loop_closure: LoopClosureStatus {
                detected: true,
                residual_m: 0.0,
                overlap: 1.0,
            },
            estimator_diverged: false,
            raceline_ready: true,
            mapping_enabled: true,
            start_line_crossed: true,
        });
        assert_eq!(mode, nfe_algo::supervisor::DriveMode::RaceLine);
    }

    #[test]
    fn degraded_localization_confidence_triggers_reactive_fallback() {
        let mut cfg = slam_test_config();
        cfg.algo.supervisor.engage_dwell_ticks = 1;
        cfg.algo.supervisor.fallback_dwell_ticks = 1;
        cfg.algo.supervisor.min_lap_for_raceline = 1;
        cfg.algo.supervisor.min_localization_confidence = 0.5;
        let mut p = Pipeline::new(cfg);
        p.reset(Pose2::default(), 1);

        let mut map_seed = l_shape_snapshot(Pose2::default(), 10_000, 0.0);
        map_seed.start_line_crossed = true;
        let _ = p.step(map_seed);
        let map = wait_for_complete_map(&p).expect("complete map");
        p.planner
            .record_success(simple_raceline(map.revision), map.revision);
        prime_supervisor_for_raceline(&mut p);

        let degraded = SensorSnapshot {
            lidar: LidarCloud {
                points: Vec::new(),
                timestamp_us: 30_000,
            },
            imu: ImuSample {
                timestamp_us: 30_000,
                ..Default::default()
            },
            sensor_fault: false,
            sonar_m: [f32::MAX; 3],
            start_line_crossed: false,
        };
        let fallback = p.step(degraded);

        assert_eq!(
            fallback.drive_mode,
            nfe_algo::supervisor::DriveMode::Reactive
        );
        assert_eq!(fallback.localization.confidence, 0.0);
    }

    #[test]
    fn unavailable_raceline_controller_forces_reactive_command() {
        let mut cfg = slam_test_config();
        cfg.algo.supervisor.engage_dwell_ticks = 1;
        cfg.algo.supervisor.fallback_dwell_ticks = 5;
        cfg.algo.supervisor.min_lap_for_raceline = 1;
        cfg.algo.supervisor.min_localization_confidence = 0.5;
        let mut p = Pipeline::new(cfg);
        p.reset(Pose2::default(), 1);

        let mut map_seed = l_shape_snapshot(Pose2::default(), 10_000, 0.0);
        map_seed.start_line_crossed = true;
        let _ = p.step(map_seed);
        let map = wait_for_complete_map(&p).expect("complete map");
        p.planner
            .record_success(simple_raceline(map.revision), map.revision);
        prime_supervisor_for_raceline(&mut p);

        p.planner.line = None;
        p.planner.source_map_revision = None;
        p.planner.availability = PlannerAvailability::NoLine;
        let fallback = p.step(l_shape_snapshot(Pose2::default(), 30_000, 0.0));

        assert_eq!(
            p.supervisor.mode(),
            nfe_algo::supervisor::DriveMode::RaceLine
        );
        assert_eq!(
            fallback.drive_mode,
            nfe_algo::supervisor::DriveMode::Reactive
        );
        assert_ne!(fallback.command.status, ControllerStatus::Unavailable);
    }
}
