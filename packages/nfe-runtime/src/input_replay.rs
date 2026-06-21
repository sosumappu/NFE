//! MCAP input replay: recorded sensor topics -> deterministic SensorSnapshot stream.
//!
//! This is pipeline input replay for golden tests/tuning. It is intentionally
//! separate from telemetry-output replay; it ignores recorded control/metrics
//! outputs and feeds only sensor inputs back into `Pipeline::step`.

use std::fs::File;
use std::path::Path;
use std::sync::mpsc;

use anyhow::{Context, Result};
use memmap2::MmapOptions;
use nfe_core::estimation::ImuSample;
use nfe_core::io::SensorSource;
use nfe_core::sensors::{LidarCloud, SensorSnapshot};
use nfe_core::telemetry::TelemetryTopic;
use prost::Message;

pub mod pb_fg {
    include!(concat!(env!("OUT_DIR"), "/foxglove.rs"));
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct SonarJson {
    timestamp_us: u64,
    front_m: f32,
    left_m: f32,
    right_m: f32,
}

#[derive(Clone, Debug)]
enum SensorFrame {
    Lidar(LidarCloud),
    Imu(ImuSample),
    Sonar(SonarJson),
}

pub struct McapSensorReplaySource {
    rx: mpsc::Receiver<SensorFrame>,
    exhausted: bool,
    lidar: Option<LidarCloud>,
    imu: ImuSample,
    sonar_m: [f32; 3],
    last_returned_lidar_ts: Option<u64>,
}

impl McapSensorReplaySource {
    pub fn open(path: impl AsRef<Path>) -> Result<Self> {
        let path = path.as_ref().to_owned();
        let file = File::open(&path)
            .with_context(|| format!("cannot open input replay: {}", path.display()))?;
        let (tx, rx) = mpsc::sync_channel(500);
        std::thread::Builder::new()
            .name("nfe-input-replay-decoder".into())
            .spawn(move || decode_worker(file, tx))
            .context("failed to spawn nfe-input-replay-decoder")?;
        Ok(Self {
            rx,
            exhausted: false,
            lidar: None,
            imu: ImuSample::default(),
            sonar_m: [f32::MAX; 3],
            last_returned_lidar_ts: None,
        })
    }

    fn apply(&mut self, frame: SensorFrame) {
        match frame {
            SensorFrame::Lidar(cloud) => self.lidar = Some(cloud),
            SensorFrame::Imu(imu) => self.imu = imu,
            SensorFrame::Sonar(sonar) => {
                self.sonar_m = [sonar.front_m, sonar.left_m, sonar.right_m]
            }
        }
    }
}

impl SensorSource for McapSensorReplaySource {
    fn next_snapshot(&mut self) -> Result<Option<SensorSnapshot>> {
        if self.exhausted {
            return Ok(None);
        }
        loop {
            match self.rx.recv() {
                Err(_) => {
                    self.exhausted = true;
                    let Some(lidar) = self.lidar.clone() else {
                        return Ok(None);
                    };
                    if self.last_returned_lidar_ts == Some(lidar.timestamp_us) {
                        return Ok(None);
                    }
                    self.last_returned_lidar_ts = Some(lidar.timestamp_us);
                    return Ok(Some(SensorSnapshot {
                        lidar,
                        imu: self.imu,
                        sonar_m: self.sonar_m,
                        sensor_fault: false,
                        start_line_crossed: false,
                    }));
                }
                Ok(frame) => {
                    let is_lidar = matches!(frame, SensorFrame::Lidar(_));
                    self.apply(frame);
                    if is_lidar {
                        let lidar = self.lidar.clone().expect("lidar just applied");
                        self.last_returned_lidar_ts = Some(lidar.timestamp_us);
                        return Ok(Some(SensorSnapshot {
                            lidar,
                            imu: self.imu,
                            sonar_m: self.sonar_m,
                            sensor_fault: false,
                            start_line_crossed: false,
                        }));
                    }
                }
            }
        }
    }
}

fn decode_lidar(data: &[u8], ts_us: u64) -> Option<SensorFrame> {
    if let Ok(mut cloud) = serde_json::from_slice::<LidarCloud>(data) {
        cloud.timestamp_us = ts_us;
        for p in &mut cloud.points {
            p.timestamp_us = ts_us;
        }
        return Some(SensorFrame::Lidar(cloud));
    }

    let cloud = pb_fg::PointCloud::decode(data).ok()?;
    let stride = cloud.point_stride as usize;
    if stride < 16 {
        return None;
    }
    let mut points = Vec::with_capacity(cloud.data.len() / stride);
    for chunk in cloud.data.chunks_exact(stride) {
        let x = f32::from_le_bytes(chunk[0..4].try_into().ok()?);
        let y = f32::from_le_bytes(chunk[4..8].try_into().ok()?);
        let dist_m = f32::from_le_bytes(chunk[8..12].try_into().ok()?);
        let angle_rad = f32::from_le_bytes(chunk[12..16].try_into().ok()?);
        points.push(nfe_core::sensors::LidarPoint {
            x,
            y,
            dist_m,
            angle_rad,
            timestamp_us: ts_us,
        });
    }
    Some(SensorFrame::Lidar(LidarCloud {
        points,
        timestamp_us: ts_us,
    }))
}

fn decode_worker(file: File, tx: mpsc::SyncSender<SensorFrame>) {
    let mmap = match unsafe { MmapOptions::new().map(&file) } {
        Ok(m) => m,
        Err(_) => return,
    };
    let messages = match mcap::MessageStream::new(&mmap) {
        Ok(s) => s,
        Err(_) => return,
    };
    for msg_result in messages {
        let Ok(msg) = msg_result else {
            continue;
        };
        let ts_us = msg.publish_time / 1_000;
        let frame = match msg.channel.topic.as_str() {
            t if t == TelemetryTopic::SensorImu.as_str() => {
                serde_json::from_slice::<ImuSample>(&msg.data)
                    .ok()
                    .map(|mut imu| {
                        imu.timestamp_us = ts_us;
                        SensorFrame::Imu(imu)
                    })
            }
            t if t == TelemetryTopic::SensorLidar.as_str() => decode_lidar(&msg.data, ts_us),
            t if t == TelemetryTopic::SensorSonar.as_str() => {
                serde_json::from_slice::<SonarJson>(&msg.data)
                    .ok()
                    .map(|mut sonar| {
                        sonar.timestamp_us = ts_us;
                        SensorFrame::Sonar(sonar)
                    })
            }
            _ => None,
        };
        if let Some(frame) = frame {
            if tx.send(frame).is_err() {
                break;
            }
        }
    }
}
