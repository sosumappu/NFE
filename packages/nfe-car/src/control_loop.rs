use std::f32::consts::PI;
use std::sync::Arc;

use anyhow::Result;
use nfe_core::{
    telemetry::{MetricsTelemetry, TelemetryEvent},
    Pose2,
};
use nfe_runtime::{
    config::RuntimeConfig,
    pipeline::{EstimatorMode, Pipeline},
    start_gate::{ArmSignalConfig, StartGateConfig, StartGateMode, StartGateRuntime},
    telemetry_bus::TelemetryBus,
};
use tokio::{
    signal::unix::{signal, SignalKind},
    time::{interval, MissedTickBehavior},
};
use tracing::{error, info, warn};

use crate::{
    config::Config,
    control::{safety::BlindAction, watchdog::Watchdog},
    hal::{ActuatorSink, SensorSource},
    metrics::{MetricsLog, TickMetrics},
    state::SensorStateWriter,
    time::monotonic_us,
    types::{ImuBias, LidarCloudView},
};

#[derive(Clone, Debug)]
pub struct ControlLoopOptions {
    pub cost_out: Option<String>,
    pub csv_out: Option<String>,
    pub start_gate_mode: StartGateMode,
    pub start_gate_config: StartGateConfig,
    pub arm_udp_bind: Option<String>,
    pub arm_udp_token: Option<String>,
    pub arm_gpio_enabled: bool,
    pub arm_gpio_pin: Option<u8>,
}

impl Default for ControlLoopOptions {
    fn default() -> Self {
        Self {
            cost_out: None,
            csv_out: None,
            start_gate_mode: StartGateMode::Replay,
            start_gate_config: StartGateConfig::for_mode(StartGateMode::Replay),
            arm_udp_bind: None,
            arm_udp_token: None,
            arm_gpio_enabled: false,
            arm_gpio_pin: None,
        }
    }
}

/// Mode-neutral runtime loop used by live, sim, and replay entry points.
///
/// Sensor/actuator selection stays outside this function. The control decision
/// path is now `nfe_runtime::Pipeline::step`; this wrapper only handles Tokio
/// pacing, shutdown, actuator writes, and hardware-facing safety overrides.
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

    let mut gate = StartGateRuntime::new(
        options.start_gate_mode,
        options.start_gate_config.clone(),
        ArmSignalConfig {
            udp_bind: options.arm_udp_bind.clone(),
            udp_token: options.arm_udp_token.clone(),
            gpio_enabled: options.arm_gpio_enabled,
            gpio_pin: options.arm_gpio_pin,
        },
    )?;

    if calibrate_imu(
        source.as_mut(),
        actuator.as_mut(),
        state.as_ref(),
        &mut ticker,
    )
    .await
    .is_err()
    {
        return Ok(());
    }

    let mut pipeline = Pipeline::new(runtime_config_from_car(config), EstimatorMode::DeadReckon);
    pipeline.set_telemetry(bus.clone());
    pipeline.reset(Pose2::default(), 0);

    let mut log = MetricsLog::new();
    let watchdog = Watchdog::new();
    let mut blind_state = crate::control::safety::BlindState::new();
    let mut estop_active = false;
    let mut estop_clean_count = 0u32;
    let mut last_steering = 0.0f32;
    let mut tick = 0u64;
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
        let source_events = source.telemetry_events();
        let runtime_snap = to_runtime_snapshot(&snap);
        let output = pipeline.step(runtime_snap);
        pipeline.publish_events(source_events);
        let command = output.command;
        let ts = snap.lidar.timestamp_us.max(snap.imu.timestamp_us);

        let safety = &config.safety;
        let lidar_age_us = if state.is_some() {
            monotonic_us().saturating_sub(snap.lidar.timestamp_us)
        } else {
            0
        };
        let imu_age_us = if state.is_some() {
            monotonic_us().saturating_sub(snap.imu.timestamp_us)
        } else {
            0
        };
        let imu_stale = imu_age_us > safety.imu_stale_ms.saturating_mul(1_000);
        if imu_stale {
            warn!(
                age_us = imu_age_us,
                "car: IMU stale — forcing creep-safe actuation"
            );
        }

        let front_points = snap
            .lidar
            .points
            .iter()
            .filter(|p| p.angle_rad.abs() <= PI / 2.0)
            .count() as u32;
        let blind_now = lidar_age_us > safety.lidar_stale_ms.saturating_mul(1_000)
            || front_points < safety.min_front_points;
        let tick_dt_ms = (config.control_dt() * 1000.0) as u64;

        let mut steering = command.steering_rad;
        let mut throttle = command.throttle;
        let mut estop = false;
        let mut watchdog_miss = false;
        let mut skip_normal_actuation = false;

        let (gate_decision, gate_telemetry) = gate.observe_tick(ts, false, true)?;
        if let Some(event) = gate_telemetry {
            pipeline.publish_event(TelemetryEvent::StartGate(event));
        }
        if !gate_decision.allow_actuation {
            actuator.safe_state()?;
            steering = 0.0;
            throttle = 0.0;
            skip_normal_actuation = true;
        }

        if !skip_normal_actuation {
            match blind_state.update(blind_now, tick_dt_ms, safety.blind_grace_ms, last_steering) {
                BlindAction::Normal => {}
                BlindAction::Coast { last_steering: ls } => {
                    steering = ls;
                    throttle = 0.0;
                    actuator.set_steering(steering)?;
                    actuator.set_throttle(throttle)?;
                    skip_normal_actuation = true;
                }
                BlindAction::SafeState => {
                    actuator.safe_state()?;
                    steering = 0.0;
                    throttle = 0.0;
                    estop = true;
                    skip_normal_actuation = true;
                    warn!("car: BLIND — safe_state");
                }
            }
        }

        if !skip_normal_actuation {
            let cloud = LidarCloudView {
                points: &snap.lidar.points,
                timestamp_us: snap.lidar.timestamp_us,
            };
            let current_speed = if imu_stale {
                0.0
            } else {
                output.estimate.motion.speed_ms
            };
            let (threat, estop_len) = crate::control::safety::estop_threat_cloud(
                &cloud,
                current_speed,
                last_steering,
                safety,
            );

            if threat {
                estop_clean_count = 0;
                estop_active = true;
                actuator.safe_state()?;
                steering = 0.0;
                throttle = 0.0;
                estop = true;
                skip_normal_actuation = true;
                warn!(estop_len, "car: ESTOP threat — safe_state");
            } else if estop_active {
                let curv = last_steering.tan() / (2.0 * safety.wheelbase_m);
                let min_forward = snap
                    .lidar
                    .points
                    .iter()
                    .filter(|p| {
                        p.x > 0.0
                            && p.x <= estop_len
                            && (p.y - curv * p.x * p.x).abs() <= safety.half_channel_w_m
                    })
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
                    actuator.safe_state()?;
                    steering = 0.0;
                    throttle = 0.0;
                    estop = true;
                    skip_normal_actuation = true;
                }
            }
        }

        if !skip_normal_actuation {
            if imu_stale {
                throttle = throttle.max(0.05);
            }
            if !steering.is_finite() || !throttle.is_finite() {
                actuator.safe_state()?;
                steering = 0.0;
                throttle = 0.0;
                estop = true;
                error!("car: non-finite pipeline command — safe_state");
            } else {
                actuator.set_steering(steering)?;
                actuator.set_throttle(throttle)?;
                last_steering = steering;
            }
        }

        let loop_us = t0.elapsed().as_micros() as u32;
        if state.is_some() {
            if loop_us as u128 > config.control_period().as_micros() {
                let _ = watchdog.miss();
                watchdog_miss = watchdog.should_escalate();
                if watchdog_miss {
                    error!("car: watchdog — safe state");
                    actuator.safe_state()?;
                }
            } else {
                let _ = watchdog.kick();
            }
        }

        let metrics = TickMetrics {
            tick,
            timestamp_us: ts,
            loop_us,
            lateral_error_m: output.corridor.lateral_error_m,
            heading_error_rad: output.corridor.heading_error_rad,
            steering_rad: steering,
            throttle,
            target_speed_ms: command.target_speed_ms,
            current_speed_ms: output.estimate.motion.speed_ms,
            nearest_obstacle_m: output.corridor.nearest_obstacle_m,
            gz_rad_s: snap.imu.gz,
            // Legacy TickMetrics exposed lateral velocity from the old kinematics
            // buffer. The current run-cost formula does not use it, and the new
            // estimator surface does not yet expose a trusted lateral velocity;
            // keep the CSV compatibility field neutral until estimation owns it.
            vy_ms: 0.0,
            estop,
            watchdog_miss,
            sensor_fault: snap.sensor_fault,
        };
        log.push(metrics);
        pipeline.publish_run_metrics(MetricsTelemetry {
            tick: metrics.tick,
            timestamp_us: metrics.timestamp_us,
            loop_us: metrics.loop_us,
            lateral_error_m: metrics.lateral_error_m,
            heading_error_rad: metrics.heading_error_rad,
            steering_rad: metrics.steering_rad,
            throttle: metrics.throttle,
            target_speed_ms: metrics.target_speed_ms,
            current_speed_ms: metrics.current_speed_ms,
            nearest_obstacle_m: metrics.nearest_obstacle_m,
            estop: metrics.estop,
            watchdog_miss: metrics.watchdog_miss,
            sensor_fault: metrics.sensor_fault,
        });

        tick += 1;
        if tick.is_multiple_of(10) {
            tracing::info!(
                tick,
                loop_us,
                steering = format!("{steering:.3}"),
                throttle = format!("{throttle:.3}"),
                lat_err = format!("{:.3}", output.corridor.lateral_error_m),
                nearest = format!("{:.2}", output.corridor.nearest_obstacle_m),
                mode = format!("{:?}", output.drive_mode),
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

    drop(bus);
    Ok(())
}

fn runtime_config_from_car(config: &Config) -> RuntimeConfig {
    let mut runtime = RuntimeConfig {
        hz: config.control.hz,
        mapping: nfe_runtime::config::MappingRuntimeConfig {
            enabled: false,
            ..Default::default()
        },
        ..Default::default()
    };
    runtime.algo.reactive.lqr.k_lateral = config.control.lqr[0];
    runtime.algo.reactive.lqr.k_lateral_rate = config.control.lqr[1];
    runtime.algo.reactive.lqr.k_heading = config.control.lqr[2];
    runtime.algo.reactive.lqr.k_yaw_rate = config.control.lqr[3];
    runtime.algo.reactive.pid.kp = config.control.pid.kp;
    runtime.algo.reactive.pid.ki = config.control.pid.ki;
    runtime.algo.reactive.pid.kd = config.control.pid.kd;
    runtime.algo.reactive.speed.v_max = config.control.speed.v_max;
    runtime.algo.reactive.speed.k_lateral = config.control.speed.k_dist;
    runtime.algo.reactive.speed.k_heading = config.control.speed.k_heading;
    runtime
}

fn to_runtime_snapshot(
    snapshot: &crate::state::SensorSnapshot,
) -> nfe_core::sensors::SensorSnapshot {
    nfe_core::sensors::SensorSnapshot {
        lidar: nfe_core::sensors::LidarCloud {
            timestamp_us: snapshot.lidar.timestamp_us,
            points: snapshot
                .lidar
                .points
                .iter()
                .map(|p| nfe_core::sensors::LidarPoint {
                    x: p.x,
                    y: p.y,
                    dist_m: p.dist_m,
                    angle_rad: p.angle_rad,
                    timestamp_us: p.timestamp_us,
                })
                .collect(),
        },
        imu: nfe_core::estimation::ImuSample {
            ax: snapshot.imu.ax,
            ay: snapshot.imu.ay,
            az: snapshot.imu.az,
            gx: snapshot.imu.gx,
            gy: snapshot.imu.gy,
            gz: snapshot.imu.gz,
            timestamp_us: snapshot.imu.timestamp_us,
        },
        sensor_fault: snapshot.sensor_fault,
        sonar_m: snapshot.sonar_m,
        start_line_crossed: false,
    }
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
