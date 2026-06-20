use std::sync::Arc;
use std::f32::consts::PI;

use anyhow::Result;
use tokio::{
    signal::unix::{signal, SignalKind},
    time::{interval, MissedTickBehavior},
};
use tracing::{error, info, warn};

use crate::{
    config::Config,
    control::{
        lqr::{Lqr, LqrState},
        mpt::Mpt,
        pid::Pid,
        speed::SpeedPlanner,
        watchdog::Watchdog,
    },
    hal::{ActuatorSink, SensorFrame, SensorSource, TimestampedFrame},
    metrics::{MetricsLog, TickMetrics},
    replay::recorder::ControlFrame,
    state::SensorStateWriter,
    telemetry::{TelemetryBus, TelemetryEvent},
    types::{ImuBias, LidarPoint},
    time::monotonic_us,
};

#[derive(Clone, Debug, Default)]
pub struct ControlLoopOptions {
    pub cost_out: Option<String>,
    pub csv_out: Option<String>,
}

/// `bus` is Option because replay and sim modes have no subscribers — passing
/// None avoids the Arc allocation on every tick in those paths. In live mode
/// the bus is always Some so the recorder and bridge both receive events.
pub async fn run(
    mut source: Box<dyn SensorSource>,
    mut actuator: Box<dyn ActuatorSink>,
    state: Option<Arc<dyn SensorStateWriter>>,
    bus: Option<TelemetryBus>,
    config: &Config,
    options: &ControlLoopOptions,
) -> Result<()> {
    if let Some(ref s) = state {
        let s2 = s.clone();
        tokio::spawn(async move {
            let mut sigterm = signal(SignalKind::terminate()).unwrap();
            let mut sigint = signal(SignalKind::interrupt()).unwrap();
            tokio::select! {
                _ = sigterm.recv() => {},
                _ = sigint.recv()  => {},
            }
            warn!("car: shutdown signal");
            s2.set_shutdown();
        });
    }

    let mut ticker = interval(config.control_period());
    ticker.set_missed_tick_behavior(MissedTickBehavior::Skip);

    let bias = match calibrate_imu(
        source.as_mut(),
        actuator.as_mut(),
        state.as_ref(),
        &mut ticker,
    )
    .await
    {
        Ok(b) => b,
        Err(_) => return Ok(()),
    };

    let mut kin =
        crate::fusion::kinematics::Kinematics::new(config.control.kinematics_horizon, bias);
    let speed = SpeedPlanner::new(
        config.control.speed.v_max,
        config.control.speed.k_dist,
        config.control.speed.k_heading,
    );
    let lqr = Lqr::new_with_gains(config.control.lqr);
    let mpt = Mpt::new();
    let mut pid = Pid::new_with_dt(
        config.control.pid.kp,
        config.control.pid.ki,
        config.control.pid.kd,
        config.control_dt(),
    );
    let watchdog = Watchdog::new();
    let mut log = MetricsLog::new();
    let mut deskew_buf: Vec<LidarPoint> = Vec::with_capacity(256);
    let mut filter_buf: Vec<LidarPoint> = Vec::with_capacity(256);
    let mut tick: u64 = 0;
    let mut last_lidar_ts = 0u64;
    // Safety and stateful counters
    let safety = &config.safety;
    let mut last_steering: f32 = 0.0;
    let mut blind_state = crate::control::safety::BlindState::new();
    let mut estop_active: bool = false;
    let mut estop_clean_count: u32 = 0;
    let start = std::time::Instant::now();

    loop {
        if state.is_some() {
            ticker.tick().await;
        }

        let done = state
            .as_ref()
            .map_or(source.is_exhausted(), |s| s.is_shutdown());
        if done {
            actuator.safe_state()?;
            info!(
                "car: shutdown after {tick} ticks ({:.1}s)",
                start.elapsed().as_secs_f32()
            );
            break;
        }

        let t0 = std::time::Instant::now();
        let snap = source.next_snapshot()?;


        let lidar_age_ms: u64 = if state.is_some() {
            let now = monotonic_us();
            now.saturating_sub(snap.lidar.timestamp_us)
        } else {
            0
        };

        let imu_age_ms: u64 = if state.is_some() {
            let now = monotonic_us();
            now.saturating_sub(snap.imu.timestamp_us)
        } else {
            0
        };

        let imu_stale = imu_age_ms > safety.imu_stale_ms;
        if imu_stale {
            warn!(age_ms = imu_age_ms, "car: IMU stale — degrading behaviour (no velocity trust, force creep)");
        }

        let front_points = snap
            .lidar
            .points
            .iter()
            .filter(|p| p.angle_rad.abs() <= PI / 2.0)
            .count() as u32;

        let blind_now = lidar_age_ms > safety.lidar_stale_ms || front_points < safety.min_front_points;
        let tick_dt_ms = (config.control_dt() * 1000.0) as u64;
        match blind_state.update(blind_now, tick_dt_ms, safety.blind_grace_ms, last_steering) {
            crate::control::safety::BlindAction::Normal => {
                // continue to normal operation
            }
            crate::control::safety::BlindAction::Coast { last_steering: ls } => {
                // HOLD last steering, coast
                let _ = actuator.set_steering(ls);
                let _ = actuator.set_throttle(0.0);
                let ts = snap.lidar.timestamp_us;
                let m = TickMetrics {
                    tick,
                    timestamp_us: ts,
                    loop_us: t0.elapsed().as_micros() as u32,
                    estop: false,
                    nearest_obstacle_m: snap.sonar_m.iter().cloned().fold(f32::MAX, f32::min),
                    sensor_fault: snap.sensor_fault,
                    ..Default::default()
                };
                log.push(m);
                publish_metrics(bus.as_ref(), m);
                tick += 1;
                continue;
            }
            crate::control::safety::BlindAction::SafeState => {
                actuator.safe_state()?;
                warn!("car: BLIND — safe_state");
                let ts = snap.lidar.timestamp_us;
                let m = TickMetrics {
                    tick,
                    timestamp_us: ts,
                    loop_us: t0.elapsed().as_micros() as u32,
                    estop: true,
                    sensor_fault: snap.sensor_fault,
                    ..Default::default()
                };
                log.push(m);
                publish_metrics(bus.as_ref(), m);
                tick += 1;
                continue;
            }
        }

        // Not blind: update kinematics and deskew
        let handle = kin.update(&snap.imu);
        let cloud = kin.deskew(&snap.lidar, &mut deskew_buf);
        let filtered = cloud.median_filtered(&mut filter_buf, 5);

        // --- ESTOP threat check on deskewed+filtered cloud (hysteresis)
        let v_current = kin.current_speed();
        let (threat, estop_len) = crate::control::safety::estop_threat_cloud(&filtered, v_current, last_steering, safety);
        let curv = last_steering.tan() / (2.0 * safety.wheelbase_m);

        if threat {
            // Any observed threat resets the clean counter
            estop_clean_count = 0;

            if !estop_active {
                actuator.safe_state()?;
                watchdog.kick();
                estop_active = true;
                warn!("car: ESTOP threat — safe_state");
            }

            let ts = snap.lidar.timestamp_us;
            let m = TickMetrics {
                tick,
                timestamp_us: ts,
                loop_us: t0.elapsed().as_micros() as u32,
                nearest_obstacle_m: filtered.points.iter().map(|p| p.dist_m).fold(f32::MAX, f32::min),
                estop: true,
                sensor_fault: snap.sensor_fault,
                ..Default::default()
            };
            log.push(m);
            publish_metrics(bus.as_ref(), m);
            tick += 1;
            continue;
        } else if estop_active {
            // compute minimum forward distance of in-channel points (same gate used by threat check)
            let min_forward = filtered
                .points
                .iter()
                .filter(|p| p.x > 0.0 && p.x <= estop_len && (p.y - curv * p.x * p.x).abs() <= safety.half_channel_w_m)
                .map(|p| p.x)
                .fold(f32::INFINITY, f32::min);

            if min_forward > estop_len + safety.rearm_gap_m {
                estop_clean_count = estop_clean_count.saturating_add(1);
            } else {
                estop_clean_count = 0;
            }

            if estop_clean_count >= safety.n_clean_ticks {
                estop_active = false;
                estop_clean_count = 0;
                info!("car: ESTOP cleared after clean ticks");
            } else {
                // still active — keep safe_state and skip control
                let ts = snap.lidar.timestamp_us;
                let m = TickMetrics {
                    tick,
                    timestamp_us: ts,
                    loop_us: t0.elapsed().as_micros() as u32,
                    nearest_obstacle_m: filtered.points.iter().map(|p| p.dist_m).fold(f32::MAX, f32::min),
                    estop: true,
                    sensor_fault: snap.sensor_fault,
                    ..Default::default()
                };
                log.push(m);
                publish_metrics(bus.as_ref(), m);
                tick += 1;
                continue;
            }
        }

        // ESTOP check already handled above; now continue controller work
        let (err_dist, heading_err) = mpt.compute(&filtered);
        let lateral_m = err_dist * heading_err.sin();
        kin.record_lateral_error(handle, lateral_m);

        let lqr_state = LqrState {
            lateral_error_m: lateral_m,
            lateral_rate_m_s: kin.lateral_rate(),
            heading_error_rad: heading_err,
            yaw_rate_rad_s: kin.current_yaw_rate(),
        };
        let steering = lqr.compute_lateral(&lqr_state);
        let v_target = speed.compute(&cloud, err_dist, heading_err);
        let v_current = kin.current_speed();
        let throttle = pid.compute_longitudinal(v_target - v_current);

        actuator.set_steering(steering)?;
        actuator.set_throttle(throttle)?;
        last_steering = steering;

        // Watchdog/leaky-bucket handling
        let loop_us = t0.elapsed().as_micros() as u32;
        if state.is_some() {
            if loop_us as u128 > config.control_period().as_micros() {
                let _ = watchdog.miss();
                if watchdog.should_escalate() {
                    error!("car: watchdog — safe state");
                    actuator.safe_state()?;
                }
            } else {
                let _ = watchdog.kick();
            }
        }

        let ts = snap.lidar.timestamp_us;
        let nearest_m = filtered.points.iter().map(|p| p.dist_m).fold(f32::MAX, f32::min);
        let m = TickMetrics {
            tick,
            timestamp_us: ts,
            loop_us,
            lateral_error_m: lateral_m,
            heading_error_rad: heading_err,
            steering_rad: steering,
            throttle,
            target_speed_ms: v_target,
            current_speed_ms: v_current,
            nearest_obstacle_m: nearest_m,
            gz_rad_s: snap.imu.gz,
            vy_ms: kin.lateral_rate() * config.control_dt(),
            estop: false,
            watchdog_miss: watchdog.should_escalate(),
            sensor_fault: snap.sensor_fault,
        };
        log.push(m);

        // Publish all telemetry to the bus in one block so the ordering in the
        // MCAP file reflects the causal sequence: sensors first, then the
        // controller decisions that resulted from them.
        if let Some(ref bus) = bus {
            bus.publish(TelemetryEvent::Sensor(TimestampedFrame {
                ts_us: ts,
                frame: SensorFrame::Imu(snap.imu),
            }));

            // Lidar frames arrive at ~10 Hz while the control loop ticks at
            // 100 Hz. Guard on the timestamp so the recorder does not write
            // the same lidar frame ten times between revolutions.
            if snap.lidar.timestamp_us != last_lidar_ts {
                bus.publish(TelemetryEvent::Sensor(TimestampedFrame {
                    ts_us: snap.lidar.timestamp_us,
                    frame: SensorFrame::Lidar((*snap.lidar).clone()),
                }));
                last_lidar_ts = snap.lidar.timestamp_us;
            }

            bus.publish(TelemetryEvent::Sensor(TimestampedFrame {
                ts_us: ts,
                frame: SensorFrame::Sonar {
                    front: snap.sonar_m[0],
                    left: snap.sonar_m[1],
                    right: snap.sonar_m[2],
                },
            }));

            bus.publish(TelemetryEvent::Control(ControlFrame {
                timestamp_us: ts,
                steering_rad: steering,
                throttle,
                target_speed: v_target,
                current_speed: v_current,
            }));

            // Metrics last: the Arc wrapping here is the only allocation per
            // tick on the hot path. It is deferred to after all sensor publishes
            // so that a slow subscriber draining the channel sees sensors and
            // metrics in the same causal order they were produced.
            publish_metrics(Some(bus), m);
        }

        tick += 1;
        if tick.is_multiple_of(10) {
            tracing::info!(
                tick,
                loop_us,
                steering = format!("{steering:.3}"),
                throttle = format!("{throttle:.3}"),
                lat_err = format!("{lateral_m:.3}"),
                nearest = format!("{nearest_m:.2}"),
                "tick"
            );
        }
    }

    let cost = log.summarise();
    info!("car: {cost}");
    if let Some(ref p) = options.cost_out {
        log.cost_to_json(p)?;
    }
    if let Some(ref p) = options.csv_out {
        log.to_csv(p)?;
    }

    Ok(())
}

// ── Helpers ────────────────────────────────────────────────────────────────

async fn calibrate_imu(
    source: &mut dyn SensorSource,
    actuator: &mut dyn ActuatorSink,
    state: Option<&Arc<dyn SensorStateWriter>>,
    ticker: &mut tokio::time::Interval,
) -> Result<ImuBias> {
    info!("car: IMU calibration — keep vehicle stationary");
    actuator.safe_state()?;
    const N: usize = 200;
    let mut samples = Vec::with_capacity(N);
    while samples.len() < N {
        if state.is_some() {
            ticker.tick().await;
        }
        if state.map_or(source.is_exhausted(), |s| s.is_shutdown()) {
            anyhow::bail!("calibration aborted");
        }
        samples.push(source.next_snapshot()?.imu);
        actuator.safe_state()?;
    }
    let bias = ImuBias::estimate(&samples);
    info!("car: calibration complete — {bias:?}");
    Ok(bias)
}

/// Wraps the TickMetrics in an Arc and publishes to the bus. The Arc is
/// created here rather than at the call site because publish_metrics is called
/// from two places (normal tick and ESTOP path) and the Arc allocation should
/// only happen when there are subscribers to receive it.
#[inline]
fn publish_metrics(bus: Option<&TelemetryBus>, m: TickMetrics) {
    if let Some(bus) = bus {
        if !bus.is_empty() {
            bus.publish(TelemetryEvent::Metrics(Arc::new(m)));
        }
    }
}
