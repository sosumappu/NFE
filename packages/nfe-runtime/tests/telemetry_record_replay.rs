use nfe_core::estimation::ImuSample;
use nfe_core::io::SensorSource;
use nfe_core::sensors::{LidarCloud, LidarPoint, SensorSnapshot};
use nfe_core::telemetry::{SensorTelemetry, TelemetryEvent};
use nfe_runtime::input_replay::McapSensorReplaySource;
use nfe_runtime::sinks::mcap::McapSink;
use nfe_runtime::telemetry_bus::{TelemetryBus, TelemetrySink};

#[test]
fn mcap_sink_round_trips_sensor_input_replay() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("roundtrip.mcap");

    let bus = TelemetryBus::new();
    let rx = bus.subscribe(32);
    let sink = McapSink::start(&path, rx).unwrap();

    let snapshot = SensorSnapshot {
        lidar: LidarCloud {
            timestamp_us: 10_000,
            points: vec![LidarPoint {
                x: 1.0,
                y: 0.0,
                dist_m: 1.0,
                angle_rad: 0.0,
                timestamp_us: 10_000,
            }],
        },
        imu: ImuSample {
            timestamp_us: 9_500,
            ax: 0.1,
            ..Default::default()
        },
        sonar_m: [1.0, 2.0, 3.0],
        sensor_fault: false,
        start_line_crossed: false,
    };

    bus.publish(TelemetryEvent::Sensor(SensorTelemetry::Snapshot(
        snapshot.clone(),
    )));
    drop(bus);
    sink.finish();

    let mut replay = McapSensorReplaySource::open(&path).unwrap();
    let replayed = replay.next_snapshot().unwrap().unwrap();
    assert_eq!(replayed.lidar.timestamp_us, 10_000);
    assert_eq!(replayed.lidar.points.len(), 1);
    assert_eq!(replayed.lidar.points[0].timestamp_us, 10_000);
    assert_eq!(replayed.imu.timestamp_us, 9_500);
    assert_eq!(replayed.sonar_m, [1.0, 2.0, 3.0]);
    assert!(replay.next_snapshot().unwrap().is_none());
}
