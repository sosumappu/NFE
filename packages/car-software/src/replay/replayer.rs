/// replay/replayer.rs — Session replayer (implements SensorSource)
///
/// Reads a recording produced by `Recorder` and feeds frames through the
/// same `SensorSource` trait that the live `SharedState` implements.
///
/// Two playback modes
/// ──────────────────
///   Realtime   — sleeps between frames to match original timing.
///                Good for watching exactly what the car experienced.
///
///   Fast       — no sleep, processes frames as fast as the CPU allows.
///                Best for running many replay iterations during algo dev.
///
/// Usage (from main.rs or a dedicated replay binary)
/// ──────────────────────────────────────────────────
///   let source = ReplaySource::open("session.bin", ReplayMode::Realtime)?;
///   // use source as Box<dyn SensorSource> in the control loop
///
/// Error tolerance
/// ───────────────
/// Frames with bad length prefixes or bincode errors are skipped with a
/// warning. The replayer continues until it either runs out of data or
/// encounters the END_MAGIC sentinel.
use std::{
    fs::File,
    io::{BufReader, Read},
    path::Path,
    sync::Arc,
    time::{Duration, Instant},
};

use anyhow::{Context, Result};
use tracing::{debug, info, warn};

use crate::hal::{SensorFrame, SensorSource, TimestampedFrame};
use crate::state::SensorSnapshot;
use crate::types::{ImuSample, LidarCloud};

const FILE_MAGIC: u32 = 0xCFE5_5E55;
const FILE_VERSION: u32 = 1;
const END_MAGIC: u32 = 0xDEAD_BEEF;
const MAX_FRAME_LEN: u32 = 4 * 1024 * 1024; // sanity cap: 4 MiB per frame

// ── Playback mode ──────────────────────────────────────────────────────────

#[derive(Clone, Copy, Debug)]
pub enum ReplayMode {
    /// Replay at original speed (sleep between frames)
    Realtime,
    /// As fast as possible — for algorithmic iteration
    Fast,
}

// ── ReplaySource ───────────────────────────────────────────────────────────

pub struct ReplaySource {
    reader: BufReader<File>,
    mode: ReplayMode,
    exhausted: bool,
    /// Wall-clock instant when the first frame was replayed
    wall_start: Option<Instant>,
    /// Timestamp of the first frame (µs)
    ts_start_us: Option<u64>,
    /// Accumulated sensor state between ticks
    lidar: LidarCloud,
    imu: ImuSample,
    sonar_m: [f32; 3],
}

impl ReplaySource {
    pub fn open(path: impl AsRef<Path>, mode: ReplayMode) -> Result<Self> {
        let path = path.as_ref();
        let file = File::open(path)
            .with_context(|| format!("cannot open replay file: {}", path.display()))?;
        let mut reader = BufReader::new(file);

        // Validate header
        let magic = read_u32(&mut reader).context("reading magic")?;
        let version = read_u32(&mut reader).context("reading version")?;

        if magic != FILE_MAGIC {
            anyhow::bail!("replay: bad magic 0x{magic:08X} — not a car session file");
        }
        if version != FILE_VERSION {
            anyhow::bail!("replay: unsupported version {version} (expected {FILE_VERSION})");
        }

        info!("replay: opened {} in {:?} mode", path.display(), mode);

        Ok(Self {
            reader,
            mode,
            exhausted: false,
            wall_start: None,
            ts_start_us: None,
            lidar: LidarCloud::default(),
            imu: ImuSample::default(),
            sonar_m: [f32::MAX; 3],
        })
    }

    /// Read and decode the next frame from disk.
    /// Returns `None` on EOF or END_MAGIC.
    fn read_next_frame(&mut self) -> Option<TimestampedFrame> {
        loop {
            let len = match read_u32(&mut self.reader) {
                Ok(n) => n,
                Err(_) => return None, // clean EOF
            };

            // End sentinel
            if len == END_MAGIC {
                info!("replay: END_MAGIC reached");
                return None;
            }

            if len > MAX_FRAME_LEN {
                warn!("replay: frame length {len} exceeds sanity cap — file corrupt?");
                return None;
            }

            let mut buf = vec![0u8; len as usize];
            if self.reader.read_exact(&mut buf).is_err() {
                warn!("replay: truncated frame — skipping to EOF");
                return None;
            }

            match bincode::deserialize::<TimestampedFrame>(&buf) {
                Ok(f) => return Some(f),
                Err(e) => {
                    warn!("replay: deserialize error ({e}) — skipping frame");
                }
            }
        }
    }

    /// For realtime mode: sleep until this frame's original timestamp.
    fn pace(&mut self, ts_us: u64) {
        if let ReplayMode::Fast = self.mode {
            return;
        }

        let wall_start = *self.wall_start.get_or_insert_with(Instant::now);
        let ts_start = *self.ts_start_us.get_or_insert(ts_us);
        let elapsed_us = ts_us.saturating_sub(ts_start);
        let target = wall_start + Duration::from_micros(elapsed_us);
        let now = Instant::now();

        if target > now {
            std::thread::sleep(target - now);
        }
    }

    fn apply_frame(&mut self, frame: SensorFrame) {
        match frame {
            SensorFrame::Lidar(cloud) => self.lidar = cloud,
            SensorFrame::Imu(sample) => self.imu = sample,
            SensorFrame::Sonar { front, left, right } => {
                self.sonar_m = [front, left, right];
            }
        }
    }
}

impl SensorSource for ReplaySource {
    /// Returns the next snapshot by consuming frames until a Lidar frame
    /// is encountered (one control-loop tick = one LIDAR revolution).
    /// All sensor types seen before that Lidar frame are folded in.
    fn next_snapshot(&mut self) -> Result<SensorSnapshot> {
        loop {
            match self.read_next_frame() {
                None => {
                    self.exhausted = true;
                    // Return last known state so the control loop can finish cleanly
                    return Ok(SensorSnapshot {
                        lidar: Arc::new(self.lidar.clone()),
                        imu: self.imu,
                        sonar_m: self.sonar_m,
                        sensor_fault: false,
                    });
                }
                Some(tf) => {
                    self.pace(tf.ts_us);
                    let is_lidar = matches!(tf.frame, SensorFrame::Lidar(_));
                    self.apply_frame(tf.frame);

                    if is_lidar {
                        // Emit one snapshot per LIDAR revolution — same cadence as live
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

// ── Helper ──────────────────────────────────────────────────────────────────

fn read_u32(r: &mut impl Read) -> Result<u32> {
    let mut buf = [0u8; 4];
    r.read_exact(&mut buf)?;
    Ok(u32::from_le_bytes(buf))
}
