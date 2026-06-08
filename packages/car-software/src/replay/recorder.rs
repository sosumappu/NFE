/// replay/recorder.rs — Session recorder
///
/// Sits between the sensor threads and SharedState. A background thread
/// subscribes to sensor updates via a channel and serializes every
/// `TimestampedFrame` to disk using `bincode` (compact binary, fast).
///
/// Usage
/// ─────
///   let recorder = Recorder::start("session_2024_06_12.bin")?;
///   // give recorder.tx() to each sensor writer
///   recorder.finish(); // flushes and closes the file
///
/// File format
/// ───────────
///   [u32 magic] [u32 version] [frame...] [u32 END_MAGIC]
///
///   Each frame is length-prefixed so the replayer can seek/truncate
///   safely even if the file was not closed cleanly (e.g. after a crash):
///   [u32 len_bytes] [bincode(TimestampedFrame)]
///
///   Corrupt / truncated trailing frames are skipped by the replayer.
use std::{
    fs::File,
    io::{BufWriter, Write},
    path::Path,
    sync::mpsc,
    thread,
    time::Instant,
};

use anyhow::{Context, Result};

use tracing::{debug, info, warn};

use crate::hal::TimestampedFrame;

const FILE_MAGIC: u32 = 0xCFE5_5E55;
const FILE_VERSION: u32 = 1;
const END_MAGIC: u32 = 0xDEAD_BEEF;

// ── Public handle ──────────────────────────────────────────────────────────

pub struct Recorder {
    tx: Option<mpsc::SyncSender<TimestampedFrame>>,
    handle: Option<thread::JoinHandle<()>>,
}

impl Recorder {
    /// Open `path` for writing and start the background recorder thread.
    pub fn start(path: impl AsRef<Path>) -> Result<Self> {
        let path = path.as_ref().to_owned();
        let file = File::create(&path)
            .with_context(|| format!("cannot create recording file: {}", path.display()))?;

        let mut writer = BufWriter::new(file);

        // Write file header
        writer.write_all(&FILE_MAGIC.to_le_bytes())?;
        writer.write_all(&FILE_VERSION.to_le_bytes())?;

        let (tx, rx) = mpsc::sync_channel::<TimestampedFrame>(1024);

        let path_display = path.display().to_string();
        let handle = thread::Builder::new()
            .name("recorder".into())
            .spawn(move || record_loop(writer, rx, path_display))
            .context("failed to spawn recorder thread")?;

        info!(
            path = %path.display(),
            channel_capacity = 1024,
            magic = format!("0x{FILE_MAGIC:08X}"),
            version = FILE_VERSION,
            "recorder: started"
        );

        Ok(Self {
            tx: Some(tx),
            handle: Some(handle),
        })
    }

    /// Clone the sender to give to sensor threads / SharedState interceptors.
    pub fn sender(&self) -> mpsc::SyncSender<TimestampedFrame> {
        self.tx.as_ref().unwrap().clone()
    }

    /// Flush and close the file. Blocks until the recorder thread finishes.
    pub fn finish(mut self) {
        let tx = self.tx.take();
        info!("recorder: closing channel (waiting for thread to drain and exit)");
        drop(tx);
        if let Some(h) = self.handle.take() {
            match h.join() {
                Ok(()) => info!("recorder: thread exited cleanly"),
                Err(e) => warn!("recorder: thread panicked: {:?}", e),
            }
        }
    }
}

impl Drop for Recorder {
    fn drop(&mut self) {
        // If finish() was not called (e.g. on panic), drain the channel anyway.
        if self.tx.is_some() {
            warn!("recorder: Drop called without finish() — draining channel");
        }
        self.tx.take();
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

// ── Background thread ──────────────────────────────────────────────────────

fn record_loop(mut writer: BufWriter<File>, rx: mpsc::Receiver<TimestampedFrame>, path: String) {
    info!(path = %path, "recorder: background thread started — waiting for frames");

    let mut frames_written: u64 = 0;
    let mut bytes_written: u64 = 0;
    let mut frames_dropped: u64 = 0;
    let mut last_log = Instant::now();

    // Frame type counters for diagnostics
    let mut n_lidar: u64 = 0;
    let mut n_imu: u64 = 0;
    let mut n_sonar: u64 = 0;

    for frame in &rx {
        // Classify for diagnostics
        match &frame.frame {
            crate::hal::SensorFrame::Lidar(_) => n_lidar += 1,
            crate::hal::SensorFrame::Imu(_) => n_imu += 1,
            crate::hal::SensorFrame::Sonar { .. } => n_sonar += 1,
        }

        let encoded = match bincode::serialize(&frame) {
            Ok(b) => b,
            Err(e) => {
                warn!(error = %e, ts_us = frame.ts_us, "recorder: serialize error — frame dropped");
                frames_dropped += 1;
                continue;
            }
        };

        let len = encoded.len() as u32;
        let frame_bytes = 4 + encoded.len() as u64; // u32 len prefix + payload

        if writer.write_all(&len.to_le_bytes()).is_err() || writer.write_all(&encoded).is_err() {
            warn!("recorder: write error — frame dropped (disk full?)");
            frames_dropped += 1;
            continue;
        }

        frames_written += 1;
        bytes_written += frame_bytes;

        // Periodic progress log every 5 s
        if last_log.elapsed().as_secs() >= 5 {
            info!(
                frames_written,
                bytes_written, frames_dropped, n_lidar, n_imu, n_sonar, "recorder: progress"
            );
            last_log = Instant::now();
        }

        // Per-frame debug trace (very verbose — only useful with RUST_LOG=trace)
        debug!(
            ts_us = frame.ts_us,
            frame_number = frames_written,
            payload_bytes = encoded.len(),
            "recorder: frame written"
        );
    }

    // Channel closed — write end marker and flush
    match writer.write_all(&END_MAGIC.to_le_bytes()) {
        Ok(()) => {}
        Err(e) => warn!(error = %e, "recorder: failed to write END_MAGIC"),
    }
    match writer.flush() {
        Ok(()) => {}
        Err(e) => warn!(error = %e, "recorder: flush error on close"),
    }

    let total_kb = bytes_written / 1024;
    info!(
        path = %path,
        frames_written,
        frames_dropped,
        total_kb,
        n_lidar,
        n_imu,
        n_sonar,
        "recorder: session complete"
    );

    if frames_dropped > 0 {
        warn!(
            frames_dropped,
            "recorder: some frames were dropped — consider reducing sensor rate or increasing channel capacity"
        );
    }
}
