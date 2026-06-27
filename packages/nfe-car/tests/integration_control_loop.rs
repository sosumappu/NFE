use std::{
    fs,
    path::PathBuf,
    sync::{Arc, Mutex},
};

use anyhow::Result;
use nfe_car::config::Config;
use nfe_car::control_loop::{self, ControlLoopOptions};
use nfe_car::hal::{ActuatorSink, SensorSource};
use nfe_car::state::SensorSnapshot;
use nfe_car::types::{ImuSample, LidarCloud};
use nfe_runtime::{
    sinks::mcap::McapSink,
    telemetry_bus::{TelemetryBus, TelemetrySink},
};
use serde_json::json;

// ── Mock SensorSource ───────────────────────────────────────────

struct LimitedSource {
    inner: Box<dyn SensorSource>,
    max: usize,
    count: usize,
}

impl SensorSource for LimitedSource {
    fn next_snapshot(&mut self) -> Result<SensorSnapshot> {
        self.count += 1;
        self.inner.next_snapshot()
    }

    fn is_exhausted(&self) -> bool {
        self.count >= self.max || self.inner.is_exhausted()
    }

    fn telemetry_events(&mut self) -> Vec<nfe_core::telemetry::TelemetryEvent> {
        self.inner.telemetry_events()
    }
}

struct CoreSimSourceAdapter {
    inner: nfe_sim::SimulatorSource,
    exhausted: bool,
    pending_events: Vec<nfe_core::telemetry::TelemetryEvent>,
}

impl CoreSimSourceAdapter {
    fn new(inner: nfe_sim::SimulatorSource) -> Self {
        Self {
            inner,
            exhausted: false,
            pending_events: Vec::new(),
        }
    }
}

impl SensorSource for CoreSimSourceAdapter {
    fn next_snapshot(&mut self) -> Result<SensorSnapshot> {
        let Some(snapshot) = nfe_core::io::SensorSource::next_snapshot(&mut self.inner)? else {
            self.exhausted = true;
            anyhow::bail!("sim exhausted");
        };
        self.pending_events
            .push(nfe_sim::telemetry::ground_truth_event_with_footprint(
                self.inner.vehicle_state(),
                self.inner.command(),
                self.inner.timestamp_us(),
                self.inner.footprint(),
            ));
        Ok(core_to_car_snapshot(snapshot))
    }

    fn is_exhausted(&self) -> bool {
        self.exhausted || self.inner.is_exhausted()
    }

    fn telemetry_events(&mut self) -> Vec<nfe_core::telemetry::TelemetryEvent> {
        std::mem::take(&mut self.pending_events)
    }
}

struct CoreSimActuatorAdapter {
    inner: nfe_sim::SimActuator,
    command: nfe_core::control::ControlOutput,
}

impl ActuatorSink for CoreSimActuatorAdapter {
    fn set_throttle(&mut self, throttle: f32) -> Result<()> {
        self.command.throttle = throttle;
        nfe_core::io::ActuatorSink::apply(&mut self.inner, &self.command)
    }

    fn set_steering(&mut self, angle_rad: f32) -> Result<()> {
        self.command.steering_rad = angle_rad;
        nfe_core::io::ActuatorSink::apply(&mut self.inner, &self.command)
    }

    fn safe_state(&mut self) -> Result<()> {
        self.command = nfe_core::control::ControlOutput::default();
        nfe_core::io::ActuatorSink::safe_state(&mut self.inner)
    }

    fn label(&self) -> &'static str {
        "sim"
    }
}

struct ScriptedSource {
    snapshots: Vec<SensorSnapshot>,
    idx: usize,
}

impl ScriptedSource {
    fn with_calibration(calib_count: usize, scenario: Vec<SensorSnapshot>) -> Self {
        let neutral = SensorSnapshot {
            lidar: Arc::new(LidarCloud::default()),
            imu: ImuSample::default(),
            sonar_m: [f32::MAX; 3],
            sensor_fault: false,
        };

        let mut snapshots = vec![neutral; calib_count];
        snapshots.extend(scenario);
        Self { snapshots, idx: 0 }
    }
}

impl SensorSource for ScriptedSource {
    fn next_snapshot(&mut self) -> Result<SensorSnapshot> {
        let i = self.idx.min(self.snapshots.len() - 1);
        self.idx += 1;
        Ok(self.snapshots[i].clone())
    }

    fn is_exhausted(&self) -> bool {
        self.idx >= self.snapshots.len()
    }
}

// ── Mock ActuatorSink ───────────────────────────────────────────

#[derive(Default)]
struct RecordingActuatorInner {
    pub throttle_log: Vec<f32>,
    pub steering_log: Vec<f32>,
    pub safe_state_calls: u32,
}

#[derive(Clone)]
struct RecordingActuator(Arc<Mutex<RecordingActuatorInner>>);

impl RecordingActuator {
    fn new() -> Self {
        Self(Arc::new(Mutex::new(RecordingActuatorInner::default())))
    }
}

impl ActuatorSink for RecordingActuator {
    fn set_throttle(&mut self, t: f32) -> Result<()> {
        self.0.lock().unwrap().throttle_log.push(t);
        Ok(())
    }
    fn set_steering(&mut self, a: f32) -> Result<()> {
        self.0.lock().unwrap().steering_log.push(a);
        Ok(())
    }
    fn safe_state(&mut self) -> Result<()> {
        self.0.lock().unwrap().safe_state_calls += 1;
        Ok(())
    }
    fn label(&self) -> &'static str {
        "test-recorder"
    }
}

fn corridor_snapshot(timestamp_us: u64, center_y: f32) -> SensorSnapshot {
    use nfe_car::types::LidarPoint;

    let mut points = Vec::new();
    for i in 1..=40 {
        let x = i as f32 * 0.05;
        for y in [center_y + 0.5, center_y - 0.5] {
            points.push(LidarPoint {
                x,
                y,
                dist_m: x.hypot(y),
                angle_rad: y.atan2(x),
                timestamp_us,
            });
        }
    }

    SensorSnapshot {
        lidar: Arc::new(LidarCloud {
            points,
            timestamp_us,
        }),
        imu: ImuSample {
            timestamp_us,
            ..Default::default()
        },
        sonar_m: [f32::MAX; 3],
        sensor_fault: false,
    }
}

fn core_to_car_snapshot(snapshot: nfe_core::sensors::SensorSnapshot) -> SensorSnapshot {
    SensorSnapshot {
        lidar: Arc::new(LidarCloud {
            timestamp_us: snapshot.lidar.timestamp_us,
            points: snapshot
                .lidar
                .points
                .into_iter()
                .map(|p| nfe_car::types::LidarPoint {
                    x: p.x,
                    y: p.y,
                    dist_m: p.dist_m,
                    angle_rad: p.angle_rad,
                    timestamp_us: p.timestamp_us,
                })
                .collect(),
        }),
        imu: ImuSample {
            ax: snapshot.imu.ax,
            ay: snapshot.imu.ay,
            az: snapshot.imu.az,
            gx: snapshot.imu.gx,
            gy: snapshot.imu.gy,
            gz: snapshot.imu.gz,
            timestamp_us: snapshot.imu.timestamp_us,
        },
        sonar_m: snapshot.sonar_m,
        sensor_fault: snapshot.sensor_fault,
    }
}

fn fixture_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(name)
}

fn micro_units(v: f32) -> i64 {
    (v * 1_000_000.0).round() as i64
}

// ── The actual tests ─────────────────────────────────────────────

#[tokio::test]
async fn new_pipeline_reactive_golden_baseline() {
    let scenario: Vec<_> = (0..12)
        .map(|i| corridor_snapshot(1_000_000 + i * 10_000, 0.10 - i as f32 * 0.01))
        .collect();
    let source: Box<dyn SensorSource> = Box::new(ScriptedSource::with_calibration(200, scenario));

    let actuator = RecordingActuator::new();
    let actuator_handle = actuator.clone();
    let actuator_box = Box::new(actuator);

    let cost_path = std::env::temp_dir().join(format!(
        "nfe-new-pipeline-golden-{}.json",
        std::process::id()
    ));
    let config = Config::default();
    control_loop::run(
        source,
        actuator_box,
        None,
        None,
        &config,
        &ControlLoopOptions {
            cost_out: Some(cost_path.to_string_lossy().into_owned()),
            csv_out: None,
            ..ControlLoopOptions::default()
        },
    )
    .await
    .unwrap();

    let inner = actuator_handle.0.lock().unwrap();
    let cost: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(&cost_path).unwrap()).unwrap();
    let _ = fs::remove_file(&cost_path);
    let actual = json!({
        "safe_state_calls": inner.safe_state_calls,
        "steering_urad": inner.steering_log.iter().copied().map(micro_units).collect::<Vec<_>>(),
        "throttle_micro": inner.throttle_log.iter().copied().map(micro_units).collect::<Vec<_>>(),
        "cost": {
            "cost": cost["cost"].clone(),
            "lateral_rms": cost["lateral_rms"].clone(),
            "heading_rms": cost["heading_rms"].clone(),
            "speed_rms": cost["speed_rms"].clone(),
            "jerk_rms": cost["jerk_rms"].clone(),
            "n_estop": cost["n_estop"].clone(),
            "n_watchdog_miss": cost["n_watchdog_miss"].clone(),
            "ticks": cost["ticks"].clone(),
        },
    });

    let path = fixture_path("new_pipeline_reactive_golden.json");
    if std::env::var_os("NFE_UPDATE_GOLDEN").is_some() {
        fs::create_dir_all(path.parent().unwrap()).unwrap();
        fs::write(&path, serde_json::to_string_pretty(&actual).unwrap()).unwrap();
    }

    let expected: serde_json::Value =
        serde_json::from_str(&fs::read_to_string(&path).unwrap()).unwrap();
    assert_eq!(actual, expected);
}

#[tokio::test]
async fn sim_mode_records_runtime_mcap_topics() {
    use nfe_sim::{
        telemetry::world_snapshot_event, KinematicBicycle, SimulatorSource, VehicleModel, World,
    };

    let dir = std::env::temp_dir().join(format!("nfe-sim-record-{}", std::process::id()));
    fs::create_dir_all(&dir).unwrap();
    let world_path = dir.join("world.json");
    let mcap_path = dir.join("sim.mcap");
    fs::write(
        &world_path,
        r#"{
          "inner_walls": [[2.0,-0.25],[6.0,-0.25],[6.0,0.25],[2.0,0.25]],
          "outer_walls": [[-1.0,-1.0],[8.0,-1.0],[8.0,1.0],[-1.0,1.0]],
          "start": {"x": 0.0, "y": 0.0, "yaw_rad": 0.0},
          "waypoints": []
        }"#,
    )
    .unwrap();

    let world = World::load(&world_path).unwrap();
    let model: Box<dyn VehicleModel> = Box::new(KinematicBicycle::default());
    let (source, actuator) = SimulatorSource::new_with_seed(world, model, 0.01, 1234);
    let bus = TelemetryBus::new();
    let sink = McapSink::start(&mcap_path, bus.subscribe(2048)).unwrap();
    nfe_runtime::session::publish_event(&bus, world_snapshot_event(source.world(), 0));
    let source: Box<dyn SensorSource> = Box::new(LimitedSource {
        inner: Box::new(CoreSimSourceAdapter::new(source)),
        max: 205,
        count: 0,
    });

    let config = Config::default();
    control_loop::run(
        source,
        Box::new(CoreSimActuatorAdapter {
            inner: actuator,
            command: nfe_core::control::ControlOutput::default(),
        }),
        None,
        Some(bus.clone()),
        &config,
        &ControlLoopOptions::default(),
    )
    .await
    .unwrap();
    drop(bus);
    sink.finish();

    let data = fs::read(&mcap_path).unwrap();
    let mut topics = std::collections::BTreeSet::new();
    let mut encodings = std::collections::BTreeMap::new();
    for msg in mcap::MessageStream::new(&data).unwrap() {
        let msg = msg.unwrap();
        topics.insert(msg.channel.topic.clone());
        encodings.insert(
            msg.channel.topic.clone(),
            msg.channel.message_encoding.clone(),
        );
    }
    assert!(topics.contains("/tf"));
    assert!(topics.contains("/tf_static"));
    assert!(topics.contains("/sensor/imu"));
    assert!(topics.contains("/sensor/lidar"));
    assert!(topics.contains("/sensor/sonar"));
    assert!(topics.contains("/control/command"));
    assert!(topics.contains("/control/scene"));
    assert!(topics.contains("/control/metrics"));
    assert!(topics.contains("/world/snapshot"));
    assert!(topics.contains("/world/walls"));
    assert!(topics.contains("/sim/ground_truth/state"));
    assert!(topics.contains("/sim/ground_truth/pose"));
    assert!(topics.contains("/sim/ground_truth/footprint"));
    assert!(topics.contains("/estimation/ekf/pose"));
    assert!(topics.contains("/control/start_gate"));
    assert!(topics.contains("/perception/reactive/scene"));
    assert_eq!(encodings["/tf"], "protobuf");
    assert_eq!(encodings["/tf_static"], "protobuf");
    assert_eq!(encodings["/sensor/lidar"], "protobuf");
    assert_eq!(encodings["/world/walls"], "protobuf");
    assert_eq!(encodings["/sim/ground_truth/pose"], "protobuf");
    assert_eq!(encodings["/sim/ground_truth/footprint"], "protobuf");
    assert_eq!(encodings["/estimation/ekf/pose"], "protobuf");
    assert_eq!(encodings["/sensor/imu"], "json");
    assert_eq!(encodings["/sensor/sonar"], "json");
    assert_eq!(encodings["/control/command"], "json");
    assert_eq!(encodings["/control/scene"], "protobuf");
    assert_eq!(encodings["/control/metrics"], "json");
    assert_eq!(encodings["/control/start_gate"], "json");
    assert_eq!(encodings["/perception/reactive/corridor"], "json");
    assert_eq!(encodings["/perception/reactive/scene"], "protobuf");
    assert_eq!(encodings["/estimation/ekf/state"], "json");
    assert_eq!(encodings["/sim/ground_truth/state"], "json");
    assert_eq!(encodings["/world/snapshot"], "json");
    let _ = fs::remove_dir_all(&dir);
}

#[tokio::test]
async fn estop_engages_when_obstacle_too_close_sonar() {
    // Create a lidar cloud with a point straight ahead at x=0.20 m
    use nfe_car::types::LidarPoint;
    let p = LidarPoint {
        x: 0.20,
        y: 0.0,
        dist_m: 0.20,
        angle_rad: 0.0,
        timestamp_us: 0,
    };
    let cloud = LidarCloud {
        points: vec![p],
        timestamp_us: 0,
    };

    let snapshot = SensorSnapshot {
        lidar: Arc::new(cloud),
        imu: ImuSample::default(),
        sonar_m: [f32::MAX; 3],
        sensor_fault: false,
    };

    let source: Box<dyn SensorSource> =
        Box::new(ScriptedSource::with_calibration(200, vec![snapshot; 5]));

    let actuator = RecordingActuator::new();
    let actuator_handle = actuator.clone();
    let actuator_box = Box::new(actuator);

    let config = Config::default(); // estop_dist_m = 0.30 by default

    control_loop::run(
        source,
        actuator_box,
        None,
        None,
        &config,
        &ControlLoopOptions::default(),
    )
    .await
    .unwrap();

    let inner = actuator_handle.0.lock().unwrap();
    assert!(inner.safe_state_calls > 0, "expected ESTOP to engage");
    assert!(
        inner
            .throttle_log
            .iter()
            .all(|&t| t == 0.0 || inner.safe_state_calls > 0),
        "throttle should never go positive while obstacle is close"
    );
}
