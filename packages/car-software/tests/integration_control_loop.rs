use std::sync::{Arc, Mutex};

use anyhow::Result;
use car::config::Config;
use car::control_loop::{self, ControlLoopOptions};
use car::hal::{ActuatorSink, SensorSource};
use car::state::SensorSnapshot;
use car::types::{ImuSample, LidarCloud};

// ── Mock SensorSource ───────────────────────────────────────────

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

// ── The actual test ─────────────────────────────────────────────

#[tokio::test]
async fn estop_engages_when_obstacle_too_close_sonar() {
    // Create a lidar cloud with a point straight ahead at x=0.20 m
    use car::types::LidarPoint;
    let p = LidarPoint { x: 0.20, y: 0.0, dist_m: 0.20, angle_rad: 0.0, timestamp_us: 0 };
    let cloud = LidarCloud { points: vec![p], timestamp_us: 0 };

    let snapshot = SensorSnapshot {
        lidar: Arc::new(cloud),
        imu: ImuSample::default(),
        sonar_m: [f32::MAX; 3],
        sensor_fault: false,
    };

    let source: Box<dyn SensorSource> = Box::new(ScriptedSource::with_calibration(
        200,
        vec![snapshot; 5],
    ));

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
