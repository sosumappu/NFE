/// sensors/lidar.rs — RPLiDAR A1 reader thread
///
/// Produces a 180° frontal point cloud with configurable angular resolution:
///   FRONT_ARC_DEG wide at DTHETA_FRONT_DEG resolution   (fine)
///   remaining arcs at DTHETA_SIDES_DEG / DTHETA_REAR_DEG (coarse)
///
/// The LIDAR_ROTATION_OFFSET_DEG constant compensates for physical mounting:
/// set it to the angle (degrees, clockwise) that the LIDAR motor/cable exit
/// faces relative to the car's true forward direction.
/// Example: cable exits toward the rear → offset = 180.0
///          cable exits toward the right → offset = 90.0
/// You can also flip it at runtime by changing the constant and redeploying.
///
/// Point cloud layout
/// ------------------
/// The published `LidarCloud` contains up to MAX_POINTS (x, y) pairs in the
/// car's local frame:  +x = forward, +y = left.
/// Points outside the 180° front arc (±90°) are discarded.
/// Within the arc, raw A1 samples are bucketed by their adjusted angle;
/// only the nearest return per bucket survives (min-distance filter).
///
/// Threading model
/// ---------------
/// Dedicated OS thread (blocking serial IO).  On each full revolution the
/// latest cloud is written into SharedState.  The control loop reads a
/// snapshot once per 10 ms tick with zero copying overhead beyond the
/// point array itself.

use std::{
    io::{self, Read, Write},
    sync::Arc,
    thread,
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use tracing::{debug, error, info, warn};

use crate::init::ReadySignal;
use crate::state::{LidarCloud, LidarPoint, SharedState};

// ═══════════════════════════════════════════════════════════════════════════
//  TUNE THESE CONSTANTS FOR YOUR PHYSICAL SETUP
// ═══════════════════════════════════════════════════════════════════════════

/// Clockwise angle offset (degrees) from the LIDAR's 0° reference to the
/// car's true forward direction.
/// Adjust until obstacles detected at the front read near 0°.
pub const LIDAR_ROTATION_OFFSET_DEG: f32 = 0.0;

/// Half-width of the "front" arc (centred on 0°) that gets fine resolution.
/// 45.0 → front arc spans -45°..+45°
pub const FRONT_HALF_ARC_DEG: f32 = 45.0;

/// Angular bucket size (degrees) inside the front arc.
/// Smaller = more points = more processing.  A1 resolution is ~1°.
pub const DTHETA_FRONT_DEG: f32 = 1.0;

/// Bucket size for the side arcs (|angle| between FRONT_HALF_ARC and 90°).
pub const DTHETA_SIDES_DEG: f32 = 5.0;

/// Bucket size for the rear arc (|angle| > 90°).
/// Rear samples are kept for obstacle avoidance but at coarse resolution.
pub const DTHETA_REAR_DEG: f32 = 10.0;

/// Minimum valid distance (metres).  Below this = LIDAR crosstalk / noise.
pub const DIST_MIN_M: f32 = 0.10;

/// Maximum valid distance (metres).  A1 range is ~6 m indoors.
pub const DIST_MAX_M: f32 = 6.0;

/// Maximum number of points in one published cloud.
/// ceil(90/1) + ceil(90/5)*2 + ceil(180/10) = 90 + 36 + 18 = 144  → 256 is safe.
pub const MAX_POINTS: usize = 256;

// ═══════════════════════════════════════════════════════════════════════════

// ── RPLiDAR A1 protocol constants ──────────────────────────────────────────

const BAUD_RATE: u32 = 115_200;
const CMD_START_SCAN: [u8; 2] = [0xA5, 0x20];
const CMD_STOP: [u8; 2]       = [0xA5, 0x25];
const CMD_RESET: [u8; 2]      = [0xA5, 0x40];
const RESPONSE_DESCRIPTOR_LEN: usize = 7;
const SAMPLE_LEN: usize = 5;

// Minimum time for one full revolution at 5.5 Hz (A1 nominal)
const MIN_REV_DURATION: Duration = Duration::from_millis(50);

// ── Bucket table built once at startup ────────────────────────────────────

/// Pre-computed mapping: adjusted_angle_deg (0..360) → Option<bucket_index>
/// None means the angle is not sampled (outside 180° front arc — disabled
/// for now; set ENABLE_REAR to true to include rear samples too).
///
/// We include the full 360° so rear data is available for obstacle avoidance,
/// but rear buckets are coarse (DTHETA_REAR_DEG).
struct BucketTable {
    /// angle_to_bucket[angle_deg_floor] = bucket index, or usize::MAX if skipped
    angle_to_bucket: Vec<usize>,
    /// Total number of buckets
    count: usize,
    /// Centre angle of each bucket (degrees, in car frame, -180..180)
    centres: Vec<f32>,
}

impl BucketTable {
    fn build() -> Self {
        // We walk 0..360 in 1° steps and assign each degree to a bucket.
        // Angle in car frame = adjusted - 180, so 0° A1 → depends on offset.
        // We label buckets by their car-frame centre angle (-180..+180).

        let mut angle_to_bucket = vec![usize::MAX; 360];
        let mut centres: Vec<f32> = Vec::with_capacity(MAX_POINTS);

        // Generate bucket centres in car-frame (-180..+180)
        // Front arc: -FRONT_HALF_ARC .. +FRONT_HALF_ARC at DTHETA_FRONT
        // Side arcs: FRONT_HALF_ARC .. 90  and  -90 .. -FRONT_HALF_ARC at DTHETA_SIDES
        // Rear arc:  90 .. 180  and  -180 .. -90  at DTHETA_REAR

        let mut angle = -180.0f32;
        while angle < 180.0 {
            let dt = dtheta_for(angle);
            let centre = angle + dt / 2.0;
            centres.push(centre);
            angle += dt;
        }

        let count = centres.len();

        // For each integer degree 0..360, find closest bucket centre
        for raw_deg in 0usize..360 {
            // Apply mounting offset and convert to car frame (-180..+180)
            let adjusted = ((raw_deg as f32 - LIDAR_ROTATION_OFFSET_DEG).rem_euclid(360.0));
            let car_frame = if adjusted > 180.0 { adjusted - 360.0 } else { adjusted };

            // Find bucket whose centre is nearest to car_frame
            if let Some((idx, _)) = centres
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| {
                    let da = (car_frame - *a).abs();
                    let db = (car_frame - *b).abs();
                    da.partial_cmp(&db).unwrap()
                })
            {
                angle_to_bucket[raw_deg] = idx;
            }
        }

        Self { angle_to_bucket, count, centres }
    }
}

fn dtheta_for(car_frame_angle: f32) -> f32 {
    let abs = car_frame_angle.abs();
    if abs <= FRONT_HALF_ARC_DEG {
        DTHETA_FRONT_DEG
    } else if abs <= 90.0 {
        DTHETA_SIDES_DEG
    } else {
        DTHETA_REAR_DEG
    }
}

// ── Thread entry point ──────────────────────────────────────────────────────

pub fn spawn(state: Arc<SharedState>, ready: ReadySignal, port_path: String) -> thread::JoinHandle<()> {
    thread::Builder::new()
        .name("lidar-reader".into())
        .stack_size(512 * 1024)
        .spawn(move || run(state, ready, port_path))
        .expect("failed to spawn lidar-reader thread")
}

fn run(state: Arc<SharedState>, ready: ReadySignal, port_path: String) {
    info!(
        "lidar: starting on {}  offset={:.1}°  front±{:.0}° dθ={:.0}°  sides dθ={:.0}°  rear dθ={:.0}°",
        port_path, LIDAR_ROTATION_OFFSET_DEG,
        FRONT_HALF_ARC_DEG, DTHETA_FRONT_DEG, DTHETA_SIDES_DEG, DTHETA_REAR_DEG,
    );

    let table = BucketTable::build();
    info!("lidar: bucket table built — {} buckets", table.count);

    let mut ready = Some(ready); // consumed after first published cloud

    loop {
        if state.is_shutdown() { break; }

        match open_and_scan(&state, &port_path, &table, &mut ready) {
            Ok(()) => break,
            Err(e) => {
                error!("lidar: {e} — retrying in 2s");
                state.sensor_fault.store(true, std::sync::atomic::Ordering::Relaxed);
                thread::sleep(Duration::from_secs(2));
                state.sensor_fault.store(false, std::sync::atomic::Ordering::Relaxed);
            }
        }
    }

    info!("lidar: thread exiting");
}

fn open_and_scan(
    state: &Arc<SharedState>,
    port_path: &str,
    table: &BucketTable,
    ready: &mut Option<ReadySignal>,
) -> io::Result<()> {
    let mut port = serialport::new(port_path, BAUD_RATE)
        .timeout(Duration::from_millis(500))
        .open()
        .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

    // Reset then start
    port.write_all(&CMD_RESET)?;
    thread::sleep(Duration::from_millis(100));
    port.write_all(&CMD_START_SCAN)?;
    port.flush()?;

    // Drain response descriptor (7 bytes)
    let mut desc = [0u8; RESPONSE_DESCRIPTOR_LEN];
    port.read_exact(&mut desc)?;
    debug!("lidar: descriptor {:02x?}", desc);

    // Per-revolution accumulator: min distance per bucket
    let mut buckets: Vec<f32> = vec![f32::MAX; table.count];
    let mut prev_start = false;
    let mut rev_start = std::time::Instant::now();
    let mut buf = [0u8; SAMPLE_LEN];

    loop {
        if state.is_shutdown() {
            port.write_all(&CMD_STOP).ok();
            break;
        }

        port.read_exact(&mut buf)?;

        // ── Parse 5-byte A1 response packet ─────────────────────
        // Byte 0 bit0 = new-scan flag, bits 1-7 = quality
        // Byte 1-2: angle Q6 little-endian (degrees × 64)
        // Byte 3-4: distance Q2 little-endian (mm × 4)
        let start_flag = (buf[0] & 0x01) != 0;
        let quality    = buf[0] >> 1;

        let angle_q6   = ((buf[2] as u16) << 7) | ((buf[1] as u16) >> 1);
        let angle_deg  = angle_q6 as f32 / 64.0;          // 0.0 .. 360.0

        let dist_q2    = (buf[4] as u16) << 8 | buf[3] as u16;
        let dist_m     = dist_q2 as f32 / 4000.0;          // Q2 mm → metres

        // New revolution: publish cloud, reset accumulators
        if start_flag && !prev_start {
            if rev_start.elapsed() >= MIN_REV_DURATION {
                publish_cloud(state, table, &buckets, ready);
                buckets.iter_mut().for_each(|b| *b = f32::MAX);
                rev_start = std::time::Instant::now();
            }
        }
        prev_start = start_flag;

        // Reject bad returns
        if quality == 0 || dist_m < DIST_MIN_M || dist_m > DIST_MAX_M {
            continue;
        }

        // Look up bucket for this raw angle
        let raw_idx = angle_deg as usize % 360;
        let bucket  = table.angle_to_bucket[raw_idx];
        if bucket == usize::MAX { continue; }

        // Keep nearest return (min-distance filter)
        if dist_m < buckets[bucket] {
            buckets[bucket] = dist_m;
        }
    }

    Ok(())
}

/// Convert the per-revolution bucket array into a `LidarCloud` and push it
/// into shared state.  Only populated buckets (dist < MAX) become points.
fn publish_cloud(
    state:  &Arc<SharedState>,
    table:  &BucketTable,
    buckets: &[f32],
    ready:  &mut Option<ReadySignal>,
) {
    let mut points: Vec<LidarPoint> = Vec::with_capacity(table.count);

    for (i, &dist_m) in buckets.iter().enumerate() {
        if dist_m >= DIST_MAX_M { continue; }          // no return in bucket

        let angle_deg = table.centres[i];              // car frame, -180..+180
        let angle_rad = angle_deg.to_radians();

        // Car-local frame: +x = forward (+cos), +y = left (-sin because CW positive)
        let x =  dist_m * angle_rad.cos();
        let y = -dist_m * angle_rad.sin();

        points.push(LidarPoint { x, y, dist_m, angle_deg });
    }

    let ts_us = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_micros() as u64;

    debug!(
        "lidar: cloud {} pts  front={:.2}m",
        points.len(),
        buckets[0].min(buckets[1]) // front two buckets as proxy
    );

    state.update_lidar(LidarCloud { points, timestamp_us: ts_us });

    // Signal readiness on first complete revolution
    if let Some(r) = ready.take() { r.signal(); }
}
