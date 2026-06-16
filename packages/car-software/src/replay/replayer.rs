/// replay/replayer.rs — MCAP Session Replayer (implements SensorSource)
///
/// Feeds recorded MCAP data back into the `SensorSource` trait seamlessly,
/// making the control loop completely unaware that it is running in a simulation.
use std::{
    fs::File,
    io::BufReader,
    path::Path,
    sync::{mpsc, Arc},
    time::{Duration, Instant},
};

use anyhow::{Context, Result};
use tracing::{debug, info, warn};

use prost::Message;

use crate::hal::{SensorFrame, SensorSource, TimestampedFrame};
use crate::state::SensorSnapshot;
use crate::types::{ImuSample, LidarCloud, LidarPoint};

// Depending on your build.rs config, include the generated schemas
pub mod pb_fg {
    include!(concat!(env!("OUT_DIR"), "/foxglove.rs"));
}
pub mod pb_car {
    include!(concat!(env!("OUT_DIR"), "/car_software.rs"));
}

#[derive(Clone, Copy, Debug)]
pub enum ReplayMode {
    Realtime,
    Fast,
}

pub struct McapReplayer {
    mode: ReplayMode,
    exhausted: bool,

    // Decoupling disk I/O from the control loop prevents OS-level file buffering
    // hiccups from artificially missing watchdog deadlines.
    rx: mpsc::Receiver<TimestampedFrame>,

    wall_start: Option<Instant>,
    ts_start_us: Option<u64>,

    lidar: LidarCloud,
    imu: ImuSample,
    sonar_m: [f32; 3],
}

impl McapReplayer {
    pub fn open(path: impl AsRef<Path>, mode: ReplayMode) -> Result<Self> {
        let path = path.as_ref().to_owned();
        let file = File::open(&path)
            .with_context(|| format!("cannot open replay file: {}", path.display()))?;
        let reader = BufReader::new(file);

        // A bounded channel enforces backpressure so the pre-fetch thread doesn't
        // gorge the RAM if we are stepping through the replay slowly.
        let (tx, rx) = mpsc::sync_channel(500);

        std::thread::Builder::new()
            .name("mcap-decoder".into())
            .spawn(move || Self::decode_worker(reader, tx))
            .context("failed to spawn mcap-decoder thread")?;

        info!("replay: opened {} in {:?} mode", path.display(), mode);

        Ok(Self {
            mode,
            exhausted: false,
            rx,
            wall_start: None,
            ts_start_us: None,
            lidar: LidarCloud::default(),
            imu: ImuSample::default(),
            sonar_m: [f32::MAX; 3],
        })
    }

    fn decode_worker(reader: BufReader<File>, tx: mpsc::SyncSender<TimestampedFrame>) {
        let messages = match mcap::MessageStream::new(reader) {
            Ok(s) => s,
            Err(e) => {
                warn!("replay: MCAP init failure — terminating pre-fetch ({e})");
                return;
            }
        };

        for msg_result in messages {
            let msg = match msg_result {
                Ok(m) => m,
                Err(e) => {
                    // A corrupted chunk shouldn't crash a massive analysis run,
                    // skipping ensures we salvage the remaining valid data.
                    warn!("replay: chunk parse error ({e}) — skipping message");
                    continue;
                }
            };

            // MCAP operates inherently on nanoseconds, but our embedded system
            // runs optimally on microsecond integers.
            let ts_us = msg.publish_time / 1_000;

            let frame_opt = match msg.channel.topic.as_str() {
                "/imu" => pb_car::ImuSample::decode(msg.data.as_ref()).ok().map(|s| {
                    SensorFrame::Imu(ImuSample {
                        timestamp_us: s.timestamp_us,
                        ax: s.ax,
                        ay: s.ay,
                        az: s.az,
                        gx: s.gx,
                        gy: s.gy,
                        gz: s.gz,
                    })
                }),

                "/sonar" => {
                    pb_car::SonarFrame::decode(msg.data.as_ref())
                        .ok()
                        .map(|s| SensorFrame::Sonar {
                            front: s.front_m,
                            left: s.left_m,
                            right: s.right_m,
                        })
                }

                "/lidar" => pb_fg::PointCloud::decode(msg.data.as_ref())
                    .ok()
                    .map(|cloud| {
                        // Pre-allocating prevents thousands of vector reallocations per tick.
                        let num_points = cloud.data.len() / cloud.point_stride as usize;
                        let mut points = Vec::with_capacity(num_points);

                        // Bypassing Protobuf's standard object repetition with this packed
                        // byte array strategy mirrors Foxglove's raw C++ memory mapping exactly.
                        for chunk in cloud.data.chunks_exact(cloud.point_stride as usize) {
                            points.push(LidarPoint {
                                x: f32::from_le_bytes(chunk[0..4].try_into().unwrap()),
                                y: f32::from_le_bytes(chunk[4..8].try_into().unwrap()),
                                dist_m: f32::from_le_bytes(chunk[8..12].try_into().unwrap()),
                                angle_rad: f32::from_le_bytes(chunk[12..16].try_into().unwrap()),
                                timestamp_us: ts_us,
                            });
                        }

                        SensorFrame::Lidar(LidarCloud {
                            timestamp_us: ts_us,
                            points,
                        })
                    }),

                // We ignore /metrics and /control channels because injecting post-processed
                // data back into the raw sensor feed would cause closed-loop feedback chaos.
                _ => None,
            };

            if let Some(frame) = frame_opt {
                if tx.send(TimestampedFrame { ts_us, frame }).is_err() {
                    // Receiver dropped means the control loop hit an ESTOP or finished.
                    break;
                }
            }
        }

        info!("replay: MCAP file fully decoded");
    }

    fn pace(&mut self, ts_us: u64) {
        if let ReplayMode::Fast = self.mode {
            return;
        }

        let wall_start = *self.wall_start.get_or_insert_with(Instant::now);
        let ts_start = *self.ts_start_us.get_or_insert(ts_us);
        let elapsed_us = ts_us.saturating_sub(ts_start);

        // Projecting the absolute offset from the start timestamp guarantees
        // perfect temporal synchronization, avoiding the drift caused by thread sleep overhead.
        let target = wall_start + Duration::from_micros(elapsed_us);
        let now = Instant::now();

        if target > now {
            std::thread::sleep(target - now);
        }
    }

    fn apply_frame(&mut self, frame: SensorFrame) {
        // Keeping an internal running state allows the snapshot method to merge
        // high-frequency IMU reads seamlessly into the lower-frequency LiDAR tick.
        match frame {
            SensorFrame::Lidar(cloud) => self.lidar = cloud,
            SensorFrame::Imu(sample) => self.imu = sample,
            SensorFrame::Sonar { front, left, right } => {
                self.sonar_m = [front, left, right];
            }
        }
    }
}

impl SensorSource for McapReplayer {
    fn next_snapshot(&mut self) -> Result<SensorSnapshot> {
        loop {
            match self.rx.recv() {
                Err(_) => {
                    self.exhausted = true;
                    // Yielding the final state handles the edge case where the file
                    // truncates abruptly before a full revolution completes.
                    return Ok(SensorSnapshot {
                        lidar: Arc::new(self.lidar.clone()),
                        imu: self.imu,
                        sonar_m: self.sonar_m,
                        sensor_fault: false,
                    });
                }
                Ok(tf) => {
                    self.pace(tf.ts_us);

                    let is_lidar = matches!(tf.frame, SensorFrame::Lidar(_));
                    self.apply_frame(tf.frame);

                    if is_lidar {
                        // Tying the master tick rate to the LiDAR spin matches the
                        // physical hardware constraints of the real world.
                        debug!(
                            "replay: lidar frame → snapshot ({} pts)",
                            self.lidar.points.len()
                        );
                        return Ok(SensorSnapshot {
                            lidar: Arc::new(self.lidar.clone()),
                            imu: self.imu,
                            sonar_m: self.sonar_m,
                            sensor_fault: false,
                        });
                    }
                }
            }
        }
    }

    fn is_exhausted(&self) -> bool {
        self.exhausted
    }
}
