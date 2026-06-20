/// stream/foxglove_bridge.rs — live Foxglove Studio WebSocket bridge
///
/// Sensor data (IMU, LiDAR, sonar) is read from SharedState via snapshot()
/// because the bridge polls on a fixed interval and only needs the latest
/// reading — it doesn't matter if it sees the same frame twice. SharedState
/// is the right abstraction for "current sensor state".
///
/// TickMetrics is different: it is a control-loop output, not a sensor input,
/// and it is produced at a known rate (one per tick). Reading it from
/// SharedState would mean storing control outputs alongside sensor inputs,
/// which conflates two opposite data flows and makes SharedState harder to
/// reason about. Instead, the bridge subscribes to the TelemetryBus and
/// drains its metrics channel on each push interval, keeping the latest value.
/// This way SharedState stays a pure sensor-input cache.
use std::sync::Arc;
use tracing::{info, warn};

use crate::metrics::TickMetrics;
use crate::state::SharedState;
use crate::telemetry::{TelemetryBus, TelemetryEvent};

// ── Generated Protobuf Modules ─────────────────────────────────────────────

#[cfg(feature = "foxglove")]
pub mod pb_fg {
    include!(concat!(env!("OUT_DIR"), "/foxglove.rs"));
}
#[cfg(feature = "foxglove")]
pub mod pb_car {
    include!(concat!(env!("OUT_DIR"), "/car_software.rs"));
}

#[cfg(feature = "foxglove")]
const DESCRIPTOR_BYTES: &[u8] =
    include_bytes!(concat!(env!("OUT_DIR"), "/messages_descriptor.bin"));

// ── Foxglove Encode Implementations ────────────────────────────────────────
//
// These impls let the foxglove SDK derive the schema and encoding automatically
// from the type, so Channel::<T>::new() needs no manual schema string.

#[cfg(feature = "foxglove")]
impl foxglove::Encode for pb_car::ImuSample {
    type Error = prost::EncodeError;
    fn get_schema() -> Option<foxglove::Schema> {
        Some(foxglove::Schema::new(
            "car_software.ImuSample",
            "protobuf",
            DESCRIPTOR_BYTES,
        ))
    }
    fn get_message_encoding() -> String {
        "protobuf".to_string()
    }
    fn encode(&self, buf: &mut impl prost::bytes::BufMut) -> Result<(), Self::Error> {
        prost::Message::encode(self, buf)?;
        Ok(())
    }
}

#[cfg(feature = "foxglove")]
impl foxglove::Encode for pb_fg::PointCloud {
    type Error = prost::EncodeError;
    fn get_schema() -> Option<foxglove::Schema> {
        Some(foxglove::Schema::new(
            "foxglove.PointCloud",
            "protobuf",
            DESCRIPTOR_BYTES,
        ))
    }
    fn get_message_encoding() -> String {
        "protobuf".to_string()
    }
    fn encode(&self, buf: &mut impl prost::bytes::BufMut) -> Result<(), Self::Error> {
        prost::Message::encode(self, buf)?;
        Ok(())
    }
}

#[cfg(feature = "foxglove")]
impl foxglove::Encode for pb_car::SonarFrame {
    type Error = prost::EncodeError;
    fn get_schema() -> Option<foxglove::Schema> {
        Some(foxglove::Schema::new(
            "car_software.SonarFrame",
            "protobuf",
            DESCRIPTOR_BYTES,
        ))
    }
    fn get_message_encoding() -> String {
        "protobuf".to_string()
    }
    fn encode(&self, buf: &mut impl prost::bytes::BufMut) -> Result<(), Self::Error> {
        prost::Message::encode(self, buf)?;
        Ok(())
    }
}

#[cfg(feature = "foxglove")]
impl foxglove::Encode for pb_car::TickMetrics {
    type Error = prost::EncodeError;
    fn get_schema() -> Option<foxglove::Schema> {
        Some(foxglove::Schema::new(
            "car_software.TickMetrics",
            "protobuf",
            DESCRIPTOR_BYTES,
        ))
    }
    fn get_message_encoding() -> String {
        "protobuf".to_string()
    }
    fn encode(&self, buf: &mut impl prost::bytes::BufMut) -> Result<(), Self::Error> {
        prost::Message::encode(self, buf)?;
        Ok(())
    }
}

// ── Bridge Implementation ──────────────────────────────────────────────────

pub struct FoxgloveBridge {
    _inner: BridgeInner,
}

pub const DEFAULT_PORT: u16 = 8765;

impl FoxgloveBridge {
    /// `bus` is used to subscribe for TickMetrics events. The subscription
    /// is created here, before the thread spawns, so no metrics are missed
    /// between start() returning and the thread's first iteration.
    pub async fn start(
        state: Arc<SharedState>,
        bus: &TelemetryBus,
        port: u16,
        push_interval_ms: u64,
    ) -> Result<Self, anyhow::Error> {
        #[cfg(feature = "foxglove")]
        {
            use foxglove::{Channel, WebSocketServer};
            use std::{sync::mpsc, thread, time::Duration};

            // Subscribe before spawning the thread so the channel is ready to
            // receive events the moment the control loop starts publishing.
            // A capacity of 64 is sufficient because the bridge drains the
            // entire channel on each 50 ms interval — at 100 Hz that is at
            // most 5 metrics events between drains.
            let metrics_rx = bus.subscribe(64);

            let server = WebSocketServer::new()
                .name("NFE")
                .bind("0.0.0.0", port)
                .start()
                .await
                .expect("foxglove: failed to start WebSocket server");

            let ch_imu = Channel::<pb_car::ImuSample>::new("/imu");
            let ch_lidar = Channel::<pb_fg::PointCloud>::new("/lidar");
            let ch_sonar = Channel::<pb_car::SonarFrame>::new("/sonar");
            let ch_metrics = Channel::<pb_car::TickMetrics>::new("/metrics");

            let state2 = state.clone();

            thread::Builder::new()
                .name("foxglove-bridge".into())
                .spawn(move || {
                    let interval = Duration::from_millis(push_interval_ms);

                    // Start with default metrics so the channel exists in
                    // Foxglove Studio immediately, even before the first tick.
                    let mut last_metrics = Arc::new(TickMetrics::default());

                    loop {
                        thread::sleep(interval);
                        if state2.is_shutdown() {
                            break;
                        }

                        // Drain every pending metrics event and keep only the
                        // latest. Earlier events in the queue are superseded —
                        // the bridge is a live view, not a recorder, so
                        // displaying stale intermediate values would be
                        // misleading rather than informative.
                        while let Ok(event) = metrics_rx.try_recv() {
                            if let TelemetryEvent::Metrics(m) = event {
                                last_metrics = m;
                            }
                        }

                        let snap = state2.snapshot();
                        let ts_ns = snap.lidar.timestamp_us * 1_000;

                        // IMU — always from latest snapshot; the sensor thread
                        // writes at 500 Hz so this is never stale by more than 2 ms.
                        let imu_msg = pb_car::ImuSample {
                            timestamp_us: snap.imu.timestamp_us,
                            ax: snap.imu.ax,
                            ay: snap.imu.ay,
                            az: snap.imu.az,
                            gx: snap.imu.gx,
                            gy: snap.imu.gy,
                            gz: snap.imu.gz,
                        };
                        ch_imu.log_with_time(&imu_msg, ts_ns);

                        // LiDAR — pack as raw bytes for the same reason as the
                        // recorder: Foxglove's 3D panel reads the data field as
                        // a vertex buffer and interprets it via the fields array.
                        let point_stride = 16u32;
                        let mut lidar_data =
                            Vec::with_capacity(snap.lidar.points.len() * point_stride as usize);
                        for p in &snap.lidar.points {
                            lidar_data.extend_from_slice(&p.x.to_le_bytes());
                            lidar_data.extend_from_slice(&p.y.to_le_bytes());
                            lidar_data.extend_from_slice(&p.dist_m.to_le_bytes());
                            lidar_data.extend_from_slice(&p.angle_rad.to_le_bytes());
                        }
                        let lidar_msg = pb_fg::PointCloud {
                            timestamp: snap.lidar.timestamp_us,
                            frame_id: "lidar".into(),
                            point_stride,
                            fields: vec![
                                pb_fg::PackedElementField {
                                    name: "x".into(),
                                    offset: 0,
                                    r#type: 7,
                                },
                                pb_fg::PackedElementField {
                                    name: "y".into(),
                                    offset: 4,
                                    r#type: 7,
                                },
                                pb_fg::PackedElementField {
                                    name: "distance".into(),
                                    offset: 8,
                                    r#type: 7,
                                },
                                pb_fg::PackedElementField {
                                    name: "angle".into(),
                                    offset: 12,
                                    r#type: 7,
                                },
                            ],
                            data: lidar_data,
                        };
                        ch_lidar.log_with_time(&lidar_msg, ts_ns);

                        // Sonar
                        let sonar_msg = pb_car::SonarFrame {
                            timestamp_us: snap.lidar.timestamp_us,
                            front_m: snap.sonar_m[0],
                            left_m: snap.sonar_m[1],
                            right_m: snap.sonar_m[2],
                        };
                        ch_sonar.log_with_time(&sonar_msg, ts_ns);

                        // Metrics — from the bus channel rather than recomputed
                        // from SharedState, because the values (lateral_error,
                        // steering, throttle, etc.) are control-loop outputs that
                        // do not live in SharedState and cannot be recomputed
                        // without re-running the controller.
                        let metrics_msg = pb_car::TickMetrics {
                            tick: last_metrics.tick,
                            timestamp_us: last_metrics.timestamp_us,
                            loop_us: last_metrics.loop_us as u64,
                            lateral_error_m: last_metrics.lateral_error_m,
                            heading_error_rad: last_metrics.heading_error_rad,
                            steering_rad: last_metrics.steering_rad,
                            throttle: last_metrics.throttle,
                            target_speed_ms: last_metrics.target_speed_ms,
                            current_speed_ms: last_metrics.current_speed_ms,
                            nearest_obstacle_m: last_metrics.nearest_obstacle_m,
                            gz_rad_s: last_metrics.gz_rad_s,
                            vy_ms: last_metrics.vy_ms,
                            estop: last_metrics.estop,
                            watchdog_miss: last_metrics.watchdog_miss,
                        };
                        ch_metrics.log_with_time(&metrics_msg, ts_ns);
                    }

                    info!("foxglove-bridge: thread exiting");
                })
                .expect("failed to spawn foxglove-bridge thread");

            info!("foxglove-bridge: listening on ws://0.0.0.0:{port}");
            info!("foxglove-bridge: connect via Foxglove Studio → Open connection → Foxglove WebSocket → ws://<pi-ip>:{port}");

            Ok(Self {
                _inner: BridgeInner::Live(server),
            })
        }

        #[cfg(not(feature = "foxglove"))]
        {
            // bus subscription is skipped entirely when the feature is off so
            // the channel slot is not wasted on a dead consumer.
            let _ = (state, bus, push_interval_ms);
            warn!(
                "foxglove-bridge: foxglove feature not enabled — live streaming unavailable.\n\
                 Add `foxglove = \"0.25\"` to Cargo.toml and build with --features foxglove"
            );
            Ok(Self {
                _inner: BridgeInner::Disabled,
            })
        }
    }
}

impl Drop for FoxgloveBridge {
    fn drop(&mut self) {
        let inner = std::mem::replace(&mut self._inner, BridgeInner::Disabled);
        match inner {
            #[cfg(feature = "foxglove")]
            BridgeInner::Live(server_handle) => {
                tracing::info!("foxglove-bridge: stopping WebSocket server...");
                server_handle.stop();
            }
            BridgeInner::Disabled => {}
        }
    }
}

// ── Internal discriminant ──────────────────────────────────────────────────

enum BridgeInner {
    #[cfg(feature = "foxglove")]
    Live(foxglove::WebSocketServerHandle),
    Disabled,
}
