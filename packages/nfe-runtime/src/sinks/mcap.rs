//! MCAP-backed telemetry sink with per-topic encodings.
//!
//! Semantic/control topics stay JSON for easy analysis. Heavy visualization
//! topics use Foxglove-compatible protobuf schemas so Foxglove can render them
//! natively without plugins.

use std::collections::HashMap;
use std::fs::File;
use std::io::BufWriter;
use std::path::Path;
use std::thread;
use std::time::Instant;

use anyhow::{Context, Result};
use mcap::{records::MessageHeader, Writer};
use nfe_core::estimation::StateEstimate;
use nfe_core::sensors::SensorSnapshot;
use nfe_core::telemetry::{
    EstimationTelemetry, EstimationTelemetryKind, GroundTruthTelemetry, LocalizationTelemetry,
    LocalizationTelemetryKind, MappingTelemetry, MappingTelemetryKind, PerceptionTelemetry,
    PerceptionTelemetryKind, PlanningTelemetry, PlanningTelemetryKind, RaceTelemetry,
    RaceTelemetryKind, SensorTelemetry, StartGateTelemetry, SupervisorTelemetry,
    SupervisorTelemetryKind, TelemetryEvent, TelemetryTopic, WallKind, WorldTelemetry,
};
use prost::Message;
use tracing::{info, warn};

use crate::telemetry_bus::{TelemetryReceiver, TelemetrySink};

pub mod pb_fg {
    include!(concat!(env!("OUT_DIR"), "/foxglove.rs"));
}

const FOXGLOVE_DESCRIPTOR: &[u8] =
    include_bytes!(concat!(env!("OUT_DIR"), "/foxglove_descriptor.bin"));

pub struct McapSink {
    handle: Option<thread::JoinHandle<()>>,
}

impl McapSink {
    pub fn start(path: impl AsRef<Path>, rx: TelemetryReceiver) -> Result<Self> {
        let path = path.as_ref().to_owned();
        let file = File::create(&path)
            .with_context(|| format!("cannot create telemetry recording: {}", path.display()))?;
        let writer = BufWriter::new(file);
        let path_str = path.display().to_string();
        let handle = thread::Builder::new()
            .name("nfe-mcap-sink".into())
            .spawn(move || mcap_loop(writer, rx, path_str))
            .context("failed to spawn nfe-mcap-sink thread")?;
        Ok(Self {
            handle: Some(handle),
        })
    }
}

impl TelemetrySink for McapSink {
    fn finish(mut self) {
        if let Some(handle) = self.handle.take() {
            if let Err(e) = handle.join() {
                warn!(?e, "mcap sink thread panicked");
            }
        }
    }
}

impl Drop for McapSink {
    fn drop(&mut self) {
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum TopicEncoding {
    Json,
    FoxgloveProtobuf { schema_name: &'static str },
}

#[derive(Clone, Copy, Debug)]
struct TopicSpec {
    topic: TelemetryTopic,
    encoding: TopicEncoding,
}

const TOPICS: &[TopicSpec] = &[
    TopicSpec {
        topic: TelemetryTopic::SensorImu,
        encoding: TopicEncoding::Json,
    },
    TopicSpec {
        topic: TelemetryTopic::SensorLidar,
        encoding: TopicEncoding::FoxgloveProtobuf {
            schema_name: "foxglove.PointCloud",
        },
    },
    TopicSpec {
        topic: TelemetryTopic::SensorSonar,
        encoding: TopicEncoding::Json,
    },
    TopicSpec {
        topic: TelemetryTopic::ControlCommand,
        encoding: TopicEncoding::Json,
    },
    TopicSpec {
        topic: TelemetryTopic::ControlMetrics,
        encoding: TopicEncoding::Json,
    },
    TopicSpec {
        topic: TelemetryTopic::ControlSafety,
        encoding: TopicEncoding::Json,
    },
    TopicSpec {
        topic: TelemetryTopic::ControlStartGate,
        encoding: TopicEncoding::Json,
    },
    TopicSpec {
        topic: TelemetryTopic::PerceptionReactiveCorridor,
        encoding: TopicEncoding::Json,
    },
    TopicSpec {
        topic: TelemetryTopic::PerceptionReactiveRansacWalls,
        encoding: TopicEncoding::Json,
    },
    TopicSpec {
        topic: TelemetryTopic::MappingRansacWalls,
        encoding: TopicEncoding::Json,
    },
    TopicSpec {
        topic: TelemetryTopic::EstimationEkfState,
        encoding: TopicEncoding::Json,
    },
    TopicSpec {
        topic: TelemetryTopic::EstimationEkfPose,
        encoding: TopicEncoding::FoxgloveProtobuf {
            schema_name: "foxglove.PosesInFrame",
        },
    },
    TopicSpec {
        topic: TelemetryTopic::EstimationEkfBias,
        encoding: TopicEncoding::Json,
    },
    TopicSpec {
        topic: TelemetryTopic::EstimationEkfCovariance,
        encoding: TopicEncoding::Json,
    },
    TopicSpec {
        topic: TelemetryTopic::MappingGlobalMapDelta,
        encoding: TopicEncoding::Json,
    },
    TopicSpec {
        topic: TelemetryTopic::MappingGlobalMapSnapshot,
        encoding: TopicEncoding::Json,
    },
    TopicSpec {
        topic: TelemetryTopic::MappingStatus,
        encoding: TopicEncoding::Json,
    },
    TopicSpec {
        topic: TelemetryTopic::MappingLoopClosure,
        encoding: TopicEncoding::Json,
    },
    TopicSpec {
        topic: TelemetryTopic::RaceStartLine,
        encoding: TopicEncoding::Json,
    },
    TopicSpec {
        topic: TelemetryTopic::RaceLap,
        encoding: TopicEncoding::Json,
    },
    TopicSpec {
        topic: TelemetryTopic::PlanningRaceLine,
        encoding: TopicEncoding::Json,
    },
    TopicSpec {
        topic: TelemetryTopic::PlanningRaceReference,
        encoding: TopicEncoding::Json,
    },
    TopicSpec {
        topic: TelemetryTopic::SupervisorState,
        encoding: TopicEncoding::Json,
    },
    TopicSpec {
        topic: TelemetryTopic::SupervisorTransition,
        encoding: TopicEncoding::Json,
    },
    TopicSpec {
        topic: TelemetryTopic::LocalizationScanMatch,
        encoding: TopicEncoding::Json,
    },
    TopicSpec {
        topic: TelemetryTopic::LocalizationParticleFilter,
        encoding: TopicEncoding::Json,
    },
    TopicSpec {
        topic: TelemetryTopic::LocalizationResult,
        encoding: TopicEncoding::Json,
    },
    TopicSpec {
        topic: TelemetryTopic::WorldSnapshot,
        encoding: TopicEncoding::Json,
    },
    TopicSpec {
        topic: TelemetryTopic::WorldWalls,
        encoding: TopicEncoding::FoxgloveProtobuf {
            schema_name: "foxglove.SceneUpdate",
        },
    },
    TopicSpec {
        topic: TelemetryTopic::SimGroundTruthState,
        encoding: TopicEncoding::Json,
    },
    TopicSpec {
        topic: TelemetryTopic::SimGroundTruthPose,
        encoding: TopicEncoding::FoxgloveProtobuf {
            schema_name: "foxglove.PosesInFrame",
        },
    },
];

fn mcap_loop(mut file_writer: BufWriter<File>, rx: TelemetryReceiver, path: String) {
    let mut writer = Writer::new(&mut file_writer).expect("failed to init mcap writer");
    let channels = register_topics(&mut writer);
    let mut frames = 0u64;
    let mut last_log = Instant::now();

    for event in &rx {
        if let Err(e) = write_event(&mut writer, &channels, &event) {
            warn!(error = %e, "mcap sink write failed");
        } else {
            frames += 1;
        }
        if last_log.elapsed().as_secs() >= 5 {
            info!(frames, path = %path, "mcap sink progress");
            last_log = Instant::now();
        }
    }

    writer.finish().expect("failed to finish mcap recording");
    info!(frames, path = %path, "mcap sink complete");
}

fn register_topics<W: std::io::Write + std::io::Seek>(
    writer: &mut Writer<W>,
) -> HashMap<&'static str, u16> {
    let json_schema = writer
        .add_schema("json", "jsonschema", br#"{"type":"object"}"#)
        .expect("json schema");
    let mut protobuf_schemas = HashMap::new();
    let mut channels = HashMap::new();

    for spec in TOPICS {
        let schema_id = match spec.encoding {
            TopicEncoding::Json => json_schema,
            TopicEncoding::FoxgloveProtobuf { schema_name } => {
                *protobuf_schemas.entry(schema_name).or_insert_with(|| {
                    writer
                        .add_schema(schema_name, "protobuf", FOXGLOVE_DESCRIPTOR)
                        .expect("protobuf schema")
                })
            }
        };
        let message_encoding = match spec.encoding {
            TopicEncoding::Json => "json",
            TopicEncoding::FoxgloveProtobuf { .. } => "protobuf",
        };
        let channel_id = writer
            .add_channel(
                schema_id,
                spec.topic.as_str(),
                message_encoding,
                &Default::default(),
            )
            .expect("failed to add mcap channel");
        channels.insert(spec.topic.as_str(), channel_id);
    }
    channels
}

fn write_event<W: std::io::Write + std::io::Seek>(
    writer: &mut Writer<W>,
    channels: &HashMap<&'static str, u16>,
    event: &TelemetryEvent,
) -> Result<()> {
    match event {
        TelemetryEvent::Sensor(SensorTelemetry::Snapshot(snapshot)) => {
            write_sensor_snapshot(writer, channels, snapshot)
        }
        TelemetryEvent::Control(e) => write_json(
            writer,
            channels,
            TelemetryTopic::ControlCommand,
            e.timestamp_us,
            e,
        ),
        TelemetryEvent::Metrics(e) => write_json(
            writer,
            channels,
            TelemetryTopic::ControlMetrics,
            e.timestamp_us,
            e,
        ),
        TelemetryEvent::Perception(e) => write_perception(writer, channels, e),
        TelemetryEvent::Estimation(e) => write_estimation(writer, channels, e),
        TelemetryEvent::Mapping(e) => write_mapping(writer, channels, e),
        TelemetryEvent::Localization(e) => write_localization(writer, channels, e),
        TelemetryEvent::Planning(e) => write_planning(writer, channels, e),
        TelemetryEvent::Supervisor(e) => write_supervisor(writer, channels, e),
        TelemetryEvent::Race(e) => write_race(writer, channels, e),
        TelemetryEvent::World(e) => write_world(writer, channels, e),
        TelemetryEvent::GroundTruth(e) => write_ground_truth(writer, channels, e),
        TelemetryEvent::StartGate(e) => write_start_gate(writer, channels, e),
    }
}

fn write_start_gate<W: std::io::Write + std::io::Seek>(
    writer: &mut Writer<W>,
    channels: &HashMap<&'static str, u16>,
    e: &StartGateTelemetry,
) -> Result<()> {
    write_json(
        writer,
        channels,
        TelemetryTopic::ControlStartGate,
        e.timestamp_us,
        e,
    )
}

fn write_sensor_snapshot<W: std::io::Write + std::io::Seek>(
    writer: &mut Writer<W>,
    channels: &HashMap<&'static str, u16>,
    snapshot: &SensorSnapshot,
) -> Result<()> {
    write_json(
        writer,
        channels,
        TelemetryTopic::SensorImu,
        snapshot.imu.timestamp_us,
        &snapshot.imu,
    )?;
    let sonar = SonarJson {
        timestamp_us: snapshot.lidar.timestamp_us,
        front_m: snapshot.sonar_m[0],
        left_m: snapshot.sonar_m[1],
        right_m: snapshot.sonar_m[2],
    };
    write_json(
        writer,
        channels,
        TelemetryTopic::SensorSonar,
        sonar.timestamp_us,
        &sonar,
    )?;
    write_point_cloud(writer, channels, snapshot)
}

#[derive(serde::Serialize, serde::Deserialize)]
struct SonarJson {
    timestamp_us: u64,
    front_m: f32,
    left_m: f32,
    right_m: f32,
}

fn write_perception<W: std::io::Write + std::io::Seek>(
    writer: &mut Writer<W>,
    channels: &HashMap<&'static str, u16>,
    e: &PerceptionTelemetry,
) -> Result<()> {
    match &e.kind {
        PerceptionTelemetryKind::ReactiveCorridor(v) => write_json(
            writer,
            channels,
            TelemetryTopic::PerceptionReactiveCorridor,
            e.timestamp_us,
            v,
        ),
        PerceptionTelemetryKind::ReactiveRansacWalls(v) => write_json(
            writer,
            channels,
            TelemetryTopic::PerceptionReactiveRansacWalls,
            e.timestamp_us,
            v,
        ),
        PerceptionTelemetryKind::MappingRansacWalls(v) => write_json(
            writer,
            channels,
            TelemetryTopic::MappingRansacWalls,
            e.timestamp_us,
            v,
        ),
    }
}

fn write_estimation<W: std::io::Write + std::io::Seek>(
    writer: &mut Writer<W>,
    channels: &HashMap<&'static str, u16>,
    e: &EstimationTelemetry,
) -> Result<()> {
    match &e.kind {
        EstimationTelemetryKind::EkfState(v) => {
            write_json(
                writer,
                channels,
                TelemetryTopic::EstimationEkfState,
                e.timestamp_us,
                v,
            )?;
            write_pose(
                writer,
                channels,
                TelemetryTopic::EstimationEkfPose,
                e.timestamp_us,
                "map",
                v.estimate,
            )?;
            write_json(
                writer,
                channels,
                TelemetryTopic::EstimationEkfBias,
                e.timestamp_us,
                v,
            )
        }
        EstimationTelemetryKind::EkfCovariance(v) => write_json(
            writer,
            channels,
            TelemetryTopic::EstimationEkfCovariance,
            e.timestamp_us,
            v,
        ),
    }
}

fn write_mapping<W: std::io::Write + std::io::Seek>(
    writer: &mut Writer<W>,
    channels: &HashMap<&'static str, u16>,
    e: &MappingTelemetry,
) -> Result<()> {
    match &e.kind {
        MappingTelemetryKind::Status(v) => write_json(
            writer,
            channels,
            TelemetryTopic::MappingStatus,
            e.timestamp_us,
            v,
        ),
        MappingTelemetryKind::GlobalMapDelta { .. } => write_json(
            writer,
            channels,
            TelemetryTopic::MappingGlobalMapDelta,
            e.timestamp_us,
            &e.kind,
        ),
        MappingTelemetryKind::GlobalMapSnapshot(v) => write_json(
            writer,
            channels,
            TelemetryTopic::MappingGlobalMapSnapshot,
            e.timestamp_us,
            v,
        ),
        MappingTelemetryKind::LoopClosure(v) => write_json(
            writer,
            channels,
            TelemetryTopic::MappingLoopClosure,
            e.timestamp_us,
            v,
        ),
    }
}

fn write_localization<W: std::io::Write + std::io::Seek>(
    writer: &mut Writer<W>,
    channels: &HashMap<&'static str, u16>,
    e: &LocalizationTelemetry,
) -> Result<()> {
    match &e.kind {
        LocalizationTelemetryKind::ScanMatch(v) => write_json(
            writer,
            channels,
            TelemetryTopic::LocalizationScanMatch,
            e.timestamp_us,
            v,
        ),
        LocalizationTelemetryKind::ParticleFilter(v) => write_json(
            writer,
            channels,
            TelemetryTopic::LocalizationParticleFilter,
            e.timestamp_us,
            v,
        ),
        LocalizationTelemetryKind::Result(v) => write_json(
            writer,
            channels,
            TelemetryTopic::LocalizationResult,
            e.timestamp_us,
            v,
        ),
    }
}

fn write_planning<W: std::io::Write + std::io::Seek>(
    writer: &mut Writer<W>,
    channels: &HashMap<&'static str, u16>,
    e: &PlanningTelemetry,
) -> Result<()> {
    match &e.kind {
        PlanningTelemetryKind::RaceLine(v) => write_json(
            writer,
            channels,
            TelemetryTopic::PlanningRaceLine,
            e.timestamp_us,
            v,
        ),
        PlanningTelemetryKind::RaceReference(v) => write_json(
            writer,
            channels,
            TelemetryTopic::PlanningRaceReference,
            e.timestamp_us,
            v,
        ),
    }
}

fn write_supervisor<W: std::io::Write + std::io::Seek>(
    writer: &mut Writer<W>,
    channels: &HashMap<&'static str, u16>,
    e: &SupervisorTelemetry,
) -> Result<()> {
    match &e.kind {
        SupervisorTelemetryKind::State(v) => write_json(
            writer,
            channels,
            TelemetryTopic::SupervisorState,
            e.timestamp_us,
            v,
        ),
        SupervisorTelemetryKind::Transition(v) => write_json(
            writer,
            channels,
            TelemetryTopic::SupervisorTransition,
            e.timestamp_us,
            v,
        ),
    }
}

fn write_race<W: std::io::Write + std::io::Seek>(
    writer: &mut Writer<W>,
    channels: &HashMap<&'static str, u16>,
    e: &RaceTelemetry,
) -> Result<()> {
    match &e.kind {
        RaceTelemetryKind::StartLineGate { .. } => write_json(
            writer,
            channels,
            TelemetryTopic::RaceStartLine,
            e.timestamp_us,
            &e.kind,
        ),
        RaceTelemetryKind::LapComplete { .. } => write_json(
            writer,
            channels,
            TelemetryTopic::RaceLap,
            e.timestamp_us,
            &e.kind,
        ),
    }
}

fn write_world<W: std::io::Write + std::io::Seek>(
    writer: &mut Writer<W>,
    channels: &HashMap<&'static str, u16>,
    e: &WorldTelemetry,
) -> Result<()> {
    match e {
        WorldTelemetry::Snapshot(v) => {
            write_json(
                writer,
                channels,
                TelemetryTopic::WorldSnapshot,
                v.timestamp_us,
                v,
            )?;
            write_world_walls(writer, channels, v)
        }
    }
}

fn write_ground_truth<W: std::io::Write + std::io::Seek>(
    writer: &mut Writer<W>,
    channels: &HashMap<&'static str, u16>,
    e: &GroundTruthTelemetry,
) -> Result<()> {
    match e {
        GroundTruthTelemetry::State(v) => {
            write_json(
                writer,
                channels,
                TelemetryTopic::SimGroundTruthState,
                v.timestamp_us,
                v,
            )?;
            let estimate = StateEstimate {
                pose: v.pose,
                motion: nfe_core::MotionState {
                    speed_ms: v.vx_ms,
                    yaw_rate_rad_s: v.yaw_rate_rad_s,
                },
                confidence: 1.0,
                consistency: 0.0,
                diverged: false,
                timestamp_us: v.timestamp_us,
            };
            write_pose(
                writer,
                channels,
                TelemetryTopic::SimGroundTruthPose,
                v.timestamp_us,
                &v.frame_id,
                estimate,
            )
        }
    }
}

fn write_point_cloud<W: std::io::Write + std::io::Seek>(
    writer: &mut Writer<W>,
    channels: &HashMap<&'static str, u16>,
    snapshot: &SensorSnapshot,
) -> Result<()> {
    let point_stride = 16u32;
    let mut data = Vec::with_capacity(snapshot.lidar.points.len() * point_stride as usize);
    for p in &snapshot.lidar.points {
        data.extend_from_slice(&p.x.to_le_bytes());
        data.extend_from_slice(&p.y.to_le_bytes());
        data.extend_from_slice(&p.dist_m.to_le_bytes());
        data.extend_from_slice(&p.angle_rad.to_le_bytes());
    }
    let msg = pb_fg::PointCloud {
        timestamp: snapshot.lidar.timestamp_us,
        frame_id: "lidar".to_string(),
        point_stride,
        fields: vec![
            field("x", 0),
            field("y", 4),
            field("distance", 8),
            field("angle", 12),
        ],
        data,
    };
    write_protobuf(
        writer,
        channels,
        TelemetryTopic::SensorLidar,
        snapshot.lidar.timestamp_us,
        &msg,
    )
}

fn field(name: &str, offset: u32) -> pb_fg::PackedElementField {
    pb_fg::PackedElementField {
        name: name.to_string(),
        offset,
        r#type: 7,
    }
}

fn write_pose<W: std::io::Write + std::io::Seek>(
    writer: &mut Writer<W>,
    channels: &HashMap<&'static str, u16>,
    topic: TelemetryTopic,
    timestamp_us: u64,
    frame_id: &str,
    estimate: StateEstimate,
) -> Result<()> {
    let msg = pb_fg::PosesInFrame {
        timestamp: Some(timestamp(timestamp_us)),
        frame_id: frame_id.to_string(),
        poses: vec![pose(estimate.pose)],
    };
    write_protobuf(writer, channels, topic, timestamp_us, &msg)
}

fn write_world_walls<W: std::io::Write + std::io::Seek>(
    writer: &mut Writer<W>,
    channels: &HashMap<&'static str, u16>,
    world: &nfe_core::telemetry::WorldSnapshotTelemetry,
) -> Result<()> {
    let mut entity = pb_fg::SceneEntity {
        id: "world_walls".to_string(),
        timestamp: Some(timestamp(world.timestamp_us)),
        frame_id: world.frame_id.clone(),
        frame_locked: 1,
        lines: Vec::new(),
    };
    for wall in &world.walls {
        let color = match wall.kind {
            WallKind::Inner => color(0.1, 0.8, 1.0, 1.0),
            WallKind::Outer => color(1.0, 0.5, 0.1, 1.0),
            WallKind::Unknown => color(0.8, 0.8, 0.8, 1.0),
        };
        entity.lines.push(pb_fg::LinePrimitive {
            r#type: 0,
            pose: Some(pose(nfe_core::Pose2::default())),
            thickness: 0.02,
            scale_invariant_thickness: 0.0,
            color: Some(color),
            points: vec![
                vector(wall.p0.x, wall.p0.y, 0.0),
                vector(wall.p1.x, wall.p1.y, 0.0),
            ],
            colors: Vec::new(),
            indices: Vec::new(),
        });
    }
    let msg = pb_fg::SceneUpdate {
        entities: vec![entity],
        deletions: Vec::new(),
    };
    write_protobuf(
        writer,
        channels,
        TelemetryTopic::WorldWalls,
        world.timestamp_us,
        &msg,
    )
}

fn timestamp(timestamp_us: u64) -> pb_fg::Timestamp {
    pb_fg::Timestamp {
        sec: (timestamp_us / 1_000_000) as i32,
        nsec: ((timestamp_us % 1_000_000) * 1_000) as u32,
    }
}

fn pose(p: nfe_core::Pose2) -> pb_fg::Pose {
    let half = 0.5 * p.yaw as f64;
    pb_fg::Pose {
        position: Some(vector(p.x, p.y, 0.0)),
        orientation: Some(pb_fg::Quaternion {
            x: 0.0,
            y: 0.0,
            z: half.sin(),
            w: half.cos(),
        }),
    }
}

fn vector(x: f32, y: f32, z: f32) -> pb_fg::Vector3 {
    pb_fg::Vector3 {
        x: x as f64,
        y: y as f64,
        z: z as f64,
    }
}

fn color(r: f32, g: f32, b: f32, a: f32) -> pb_fg::Color {
    pb_fg::Color { r, g, b, a }
}

fn write_json<W: std::io::Write + std::io::Seek, T: serde::Serialize>(
    writer: &mut Writer<W>,
    channels: &HashMap<&'static str, u16>,
    topic: TelemetryTopic,
    timestamp_us: u64,
    value: &T,
) -> Result<()> {
    let payload = serde_json::to_vec(value)?;
    write_raw(writer, channels, topic, timestamp_us, &payload)
}

fn write_protobuf<W: std::io::Write + std::io::Seek, T: Message>(
    writer: &mut Writer<W>,
    channels: &HashMap<&'static str, u16>,
    topic: TelemetryTopic,
    timestamp_us: u64,
    value: &T,
) -> Result<()> {
    let payload = value.encode_to_vec();
    write_raw(writer, channels, topic, timestamp_us, &payload)
}

fn write_raw<W: std::io::Write + std::io::Seek>(
    writer: &mut Writer<W>,
    channels: &HashMap<&'static str, u16>,
    topic: TelemetryTopic,
    timestamp_us: u64,
    payload: &[u8],
) -> Result<()> {
    let topic_str = topic.as_str();
    let channel_id = *channels
        .get(topic_str)
        .ok_or_else(|| anyhow::anyhow!("unregistered telemetry topic: {topic_str}"))?;
    let ts_ns = timestamp_us.saturating_mul(1_000);
    writer.write_to_known_channel(
        &MessageHeader {
            channel_id,
            sequence: 0,
            log_time: ts_ns,
            publish_time: ts_ns,
        },
        payload,
    )?;
    Ok(())
}
