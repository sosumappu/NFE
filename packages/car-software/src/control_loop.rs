use std::sync::Arc;

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
        watchdog::{Watchdog, WATCHDOG_MAX_MISSED},
    },
    hal::{ActuatorSink, SensorSource},
    metrics::{MetricsLog, TickMetrics},
    observability::Observability,
    replay::recorder::{ControlFrame, McapMessage},
    state::SensorStateWriter,
    types::{ImuBias, LidarPoint},
};

#[derive(Clone, Debug, Default)]
pub struct ControlLoopOptions {
    pub cost_out: Option<String>,
    pub csv_out: Option<String>,
}

pub async fn run(
    mut source: Box<dyn SensorSource>,
    mut actuator: Box<dyn ActuatorSink>,
    state: Option<Arc<dyn SensorStateWriter>>,
    mcap_tx: Option<std::sync::mpsc::SyncSender<McapMessage>>,
    config: &Config,
    _observability: &Observability,
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
    let speed = SpeedPlanner::new(crate::control::actuate::THROTTLE_MAX, 1.0, 1.0);
    let lqr = Lqr::new();
    let mpt = Mpt::new();
    let mut pid = Pid::new_with_dt(1.5, 0.05, 0.2, config.control_dt());
    let watchdog = Watchdog::new();
    let mut log = MetricsLog::new();
    let mut deskew_buf: Vec<LidarPoint> = Vec::with_capacity(256);
    let mut tick: u64 = 0;
    let mut last_lidar_ts = 0;
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

        let nearest_m = {
            let s = snap.sonar_m.iter().cloned().fold(f32::MAX, f32::min);
            let l = snap
                .lidar
                .nearest_in_arc(0.0, 30.0_f32.to_radians())
                .map_or(f32::MAX, |p| p.dist_m);
            s.min(l)
        };

        if snap.obstacle_closer_than(config.control.estop_dist_m) {
            actuator.safe_state()?;
            watchdog.kick();
            warn!("car: ESTOP nearest={nearest_m:.2}m");

            let m = TickMetrics {
                tick,
                timestamp_us: snap.lidar.timestamp_us,
                loop_us: t0.elapsed().as_micros() as u32,
                nearest_obstacle_m: nearest_m,
                estop: true,
                ..Default::default()
            };
            log.push(m);
            send_metrics(&mcap_tx, m);
            tick += 1;
            continue;
        }

        let handle = kin.update(&snap.imu);
        let cloud = kin.deskew(&snap.lidar, &mut deskew_buf);
        let (err_dist, heading_err) = mpt.compute(&cloud);
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

        let loop_us = t0.elapsed().as_micros() as u32;
        let wd_miss = if state.is_some() {
            if loop_us as u128 > config.control_period().as_micros() {
                if watchdog.miss() >= WATCHDOG_MAX_MISSED {
                    error!("car: watchdog — safe state");
                    actuator.safe_state()?;
                }
                true
            } else {
                watchdog.kick();
                false
            }
        } else {
            false
        };

        let ts = snap.lidar.timestamp_us;
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
            watchdog_miss: wd_miss,
            sensor_fault: snap.sensor_fault,
        };
        log.push(m);

        if let Some(ref tx) = mcap_tx {
            use crate::hal::{SensorFrame, TimestampedFrame};
            let _ = tx.try_send(McapMessage::Sensor(TimestampedFrame {
                ts_us: ts,
                frame: SensorFrame::Imu(snap.imu),
            }));
            if snap.lidar.timestamp_us != last_lidar_ts {
                let _ = tx.try_send(McapMessage::Sensor(TimestampedFrame {
                    ts_us: snap.lidar.timestamp_us,
                    frame: SensorFrame::Lidar((*snap.lidar).clone()),
                }));
                last_lidar_ts = snap.lidar.timestamp_us;
            }
            let _ = tx.try_send(McapMessage::Sensor(TimestampedFrame {
                ts_us: ts,
                frame: SensorFrame::Sonar {
                    front: snap.sonar_m[0],
                    left: snap.sonar_m[1],
                    right: snap.sonar_m[2],
                },
            }));
            let _ = tx.try_send(McapMessage::Control(ControlFrame {
                timestamp_us: ts,
                steering_rad: steering,
                throttle,
                target_speed: v_target,
                current_speed: v_current,
            }));
            send_metrics(tx, m);
        }

        tick += 1;
        if tick % 10 == 0 {
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

#[inline]
fn send_metrics(tx: &Option<std::sync::mpsc::SyncSender<McapMessage>>, m: TickMetrics) {
    if let Some(ref tx) = tx {
        let _ = tx.try_send(McapMessage::Metrics(Box::new(m)));
    }
}
