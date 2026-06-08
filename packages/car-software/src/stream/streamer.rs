/// stream/streamer.rs — UDP sensor broadcaster
///
/// Runs as an optional background task alongside the control loop.
/// Every time a new LiDAR revolution is published, the streamer grabs a
/// full `SensorSnapshot` and broadcasts it to all registered subscribers.
///
/// Wire format
/// ───────────
/// Each datagram is a length-prefixed, `bincode`-encoded `StreamFrame`.
/// The receiver (your Mac) parses these frames for live visualisation.
///
///   [u16 len_bytes] [bincode(StreamFrame)]
///
/// A `StreamFrame` is intentionally smaller than a full `SensorSnapshot`:
/// it omits the raw LiDAR point cloud by default (can be enabled) to keep
/// datagram sizes below MTU (~1500 bytes for most networks).
///
/// Subscriber management
/// ─────────────────────
/// Subscribers register by sending a single UDP datagram (any content) to
/// the server port. The server records the sender's address and streams
/// to it until it times out (no ACK for SUBSCRIBER_TIMEOUT).
///
/// Because UDP is connectionless and datagrams are best-effort, a crashed
/// subscriber does not block or affect the car's control loop in any way.
///
/// Usage
/// ─────
/// On the Pi:
///   STREAM=1 systemctl restart car   (or set via environment in the unit file)
///
/// On your Mac:
///   cargo run --bin car-monitor -- --pi pi.local --port 9200
///
/// Security note
/// ─────────────
/// This is a development tool — no authentication. Use only on a trusted
/// local network (e.g. your bench WiFi). Do not expose to the internet.
use std::{
    collections::HashMap,
    f32::consts::PI,
    net::{SocketAddr, UdpSocket},
    sync::{Arc, Mutex},
    thread,
    time::{Duration, Instant},
};

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use tracing::{debug, info, warn};

use crate::state::{SensorSnapshot, SharedState};
use crate::types::{ImuSample, LidarPoint};

// ── Configuration ─────────────────────────────────────────────────────────

/// Port the Pi listens on for subscriber registrations and frame sending.
pub const DEFAULT_PORT: u16 = 9200;

/// Drop a subscriber if it has not re-registered within this window.
const SUBSCRIBER_TIMEOUT: Duration = Duration::from_secs(10);

/// How often to push a frame even if LiDAR hasn't updated (IMU-only mode).
const PUSH_INTERVAL: Duration = Duration::from_millis(50); // 20 Hz

// ── Wire types ─────────────────────────────────────────────────────────────

/// Compact frame sent over UDP. Keep fields small — must fit under MTU.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct StreamFrame {
    pub ts_us: u64,
    pub imu: ImuSample,
    pub sonar_m: [f32; 3],
    /// Nearest point per 10° arc sector (-180..+180), 36 values.
    /// index 0 = -180°, index 35 = +170°
    #[serde(with = "serde_big_array::BigArray")]
    pub lidar_sectors: [f32; 36],
    /// Full point cloud (only populated when `full_cloud` flag is set)
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub lidar_cloud: Vec<LidarPoint>,
}

impl StreamFrame {
    fn from_snapshot(snap: &SensorSnapshot, full_cloud: bool) -> Self {
        // Bucket the cloud into 36 × 10° sectors (min distance per sector)
        let mut sectors = [f32::MAX; 36];
        for pt in &snap.lidar.points {
            let idx = ((pt.angle_rad + PI) / 10.0).floor() as usize;
            let idx = idx.min(35);
            if pt.dist_m < sectors[idx] {
                sectors[idx] = pt.dist_m;
            }
        }

        Self {
            ts_us: snap.lidar.timestamp_us,
            imu: snap.imu,
            sonar_m: snap.sonar_m,
            lidar_sectors: sectors,
            lidar_cloud: if full_cloud {
                snap.lidar.points.clone()
            } else {
                vec![]
            },
        }
    }
}

// ── Streamer ───────────────────────────────────────────────────────────────

pub struct Streamer {
    _handle: thread::JoinHandle<()>,
}

impl Streamer {
    /// Bind to `0.0.0.0:port` and start the background streamer thread.
    pub fn start(state: Arc<SharedState>, port: u16, full_cloud: bool) -> Result<Self> {
        let socket = UdpSocket::bind(format!("0.0.0.0:{port}"))
            .with_context(|| format!("streamer: cannot bind to port {port}"))?;
        socket.set_nonblocking(false)?;
        socket.set_read_timeout(Some(Duration::from_millis(50)))?;

        info!("streamer: listening on 0.0.0.0:{port}  full_cloud={full_cloud}");

        let handle = thread::Builder::new()
            .name("streamer".into())
            .spawn(move || run(socket, state, full_cloud))
            .context("failed to spawn streamer thread")?;

        Ok(Self { _handle: handle })
    }
}

// ── Background thread ──────────────────────────────────────────────────────

type Subscribers = Arc<Mutex<HashMap<SocketAddr, Instant>>>;

fn run(socket: UdpSocket, state: Arc<SharedState>, full_cloud: bool) {
    let subscribers: Subscribers = Arc::new(Mutex::new(HashMap::new()));
    let mut last_push = Instant::now();

    let mut recv_buf = [0u8; 64]; // registration datagrams are tiny

    loop {
        if state.is_shutdown() {
            break;
        }

        // ── Accept subscriber registrations ────────────────────────────────
        match socket.recv_from(&mut recv_buf) {
            Ok((_, addr)) => {
                let mut subs = subscribers.lock().unwrap();
                let is_new = !subs.contains_key(&addr);
                subs.insert(addr, Instant::now());
                if is_new {
                    info!("streamer: new subscriber {addr}");
                }
            }
            Err(ref e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => warn!("streamer: recv error: {e}"),
        }

        // ── Push frame at PUSH_INTERVAL ────────────────────────────────────
        if last_push.elapsed() < PUSH_INTERVAL {
            continue;
        }
        last_push = Instant::now();

        let snap = state.snapshot();
        let frame = StreamFrame::from_snapshot(&snap, full_cloud);

        let encoded = match bincode::serialize(&frame) {
            Ok(b) => b,
            Err(e) => {
                warn!("streamer: serialize error: {e}");
                continue;
            }
        };

        // Length prefix
        let len = encoded.len() as u16;
        let mut datagram = Vec::with_capacity(2 + encoded.len());
        datagram.extend_from_slice(&len.to_le_bytes());
        datagram.extend_from_slice(&encoded);

        // Evict timed-out subscribers and broadcast to the rest
        let mut subs = subscribers.lock().unwrap();
        subs.retain(|addr, last_seen| {
            if last_seen.elapsed() > SUBSCRIBER_TIMEOUT {
                info!("streamer: subscriber {addr} timed out");
                return false;
            }
            if let Err(e) = socket.send_to(&datagram, addr) {
                debug!("streamer: send to {addr} failed: {e}");
            }
            true
        });
    }

    info!("streamer: thread exiting");
}
