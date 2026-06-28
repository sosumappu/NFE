# Car configuration parameters

This table documents the TOML parameters loaded by `packages/nfe-car/src/config.rs`. The `[sim]` section is owned by the car config, while the nested simulator parameter structs live in `nfe-sim` so the simulator can consume them without depending on `nfe-car`. Defaults listed here are the Rust defaults; `packages/nfe-car/nfe.toml` may override them for deployment.

## Control loop and legacy control fields

| TOML path | Default | Meaning | How to get or tune it |
| --- | ---: | --- | --- |
| `control.hz` | `100` | Main control-loop frequency in hertz. | Start from the slowest sensor/control path that can reliably meet deadlines. Verify loop timing telemetry has low misses on the Pi; keep `dt = 1 / hz` consistent with sim/tuning runs. |
| `control.kinematics_horizon` | `500` | Legacy planning horizon for older kinematic control paths. | Currently not wired into the active runtime pipeline; keep default unless re-enabling the old horizon-based controller, then set from desired lookahead ticks. |
| `control.estop_dist_m` | `0.30` | Legacy emergency-stop distance. | Currently superseded by `[safety]` arc/channel settings in the active loop; keep default unless re-enabling older safety logic. |
| `control.watchdog_max_missed` | `3` | Legacy count of missed control ticks before watchdog action. | Active watchdog behavior is mostly driven in the control loop and safety config; choose from acceptable missed-loop time, e.g. `max_missed / hz`. |
| `control.lqr[0]` | `0.80` | Legacy LQR lateral-error gain. | Currently not mapped into the active reactive Stanley controller; if re-enabling LQR, tune in sim/replay from lateral RMS and steering smoothness. |
| `control.lqr[1]` | `0.30` | Legacy LQR lateral-rate gain. | Same as above; fit/tune against lateral velocity or lateral-error derivative logs. |
| `control.lqr[2]` | `1.20` | Legacy LQR heading-error gain. | Same as above; increase until heading convergence is fast without oscillation. |
| `control.lqr[3]` | `0.40` | Legacy LQR yaw-rate damping gain. | Same as above; increase to damp oscillation, decrease if the car becomes sluggish. |

## Reactive speed controller

| TOML path | Default | Meaning | How to get or tune it |
| --- | ---: | --- | --- |
| `control.speed.v_max` | `1.8` | Maximum target speed emitted by the reactive speed planner. | Start below the speed where perception and safety remain stable, then raise in sim/replay and confirm live logs do not show frequent estop, watchdog, or localization degradation. |
| `control.speed.k_lateral` | `1.0` | Speed reduction per meter of lateral corridor error. | Tune with `car-tune` or replay sweeps; increase if the car is too fast when off-center, decrease if it slows excessively on small offsets. |
| `control.speed.k_heading` | `5.0` | Quadratic speed reduction for heading error. | Tune from corner-entry behavior; increase if the car enters turns too fast while misaligned, decrease if it crawls on mild heading error. |
| `control.speed.obstacle_slowdown_m` | `3.0` | Distance at which nearest obstacle starts scaling target speed down. | Set from braking distance plus perception margin. Validate with obstacle logs so slowdown starts before safety estop would trigger. |

## PID throttle controller

| TOML path | Default | Meaning | How to get or tune it |
| --- | ---: | --- | --- |
| `control.pid.kp` | `1.8` | Proportional throttle response to speed error. | Tune with step responses in sim/replay first, then live low-speed runs. Increase until response is quick but not oscillatory. |
| `control.pid.ki` | `0.15` | Integral correction for steady-state speed error. | Increase only enough to remove persistent bias from drag/deadband. If throttle winds up after stops or turns, reduce it. |
| `control.pid.kd` | `0.4` | Derivative damping on speed-error changes. | Increase to reduce overshoot, decrease if throttle becomes noisy from speed-estimate noise. |
| `control.pid.windup_limit` | `0.62` | Clamp on accumulated integral term. | Set from the largest acceptable integral contribution; lower it if throttle stays biased after target speed changes. |
| `control.pid.max_throttle` | `1.0` | Absolute throttle command limit for the PID output. | Set lower for bench/first live tests; raise toward `1.0` only after acceleration, braking, and safety margins are validated. |

## Stanley steering controller

| TOML path | Default | Meaning | How to get or tune it |
| --- | ---: | --- | --- |
| `control.stanley.k_cross_track` | `1.0` | Cross-track gain converting lateral corridor error into steering. | Tune in sim/replay for low lateral RMS without weave. Increase if it cuts back to center too slowly; decrease if it oscillates. |
| `control.stanley.softening_speed_ms` | `1.0` | Speed softening term that prevents excessive cross-track steering near zero speed. | Increase if low-speed steering snaps to large angles; decrease if low-speed recentering is too weak. |
| `control.stanley.max_steering_rad` | `0.38` | Clamp on commanded front-wheel steering angle. | Measure physical steering limit at the wheels and set below the reliable/safe limit; validate with servo current and tyre scrub. |

## Perception mode and RANSAC corridor perception

| TOML path | Default | Meaning | How to get or tune it |
| --- | ---: | --- | --- |
| `control.perception.mode` | `"corridor"` | Selects runtime perception path: `"corridor"` for RANSAC wall/corridor or `"apex"` for discontinuity/apex perception. | Choose from track geometry and logs. Use `apex` when corridor walls are discontinuous; use `corridor` when two wall lines are reliably visible. |
| `control.perception.ransac.inlier_dist_m` | `0.2` | Maximum point-to-line distance for RANSAC wall inliers. | Estimate from LiDAR range noise, wall roughness, and pose distortion. Sweep in replay; too low loses walls, too high merges clutter. |
| `control.perception.ransac.min_inliers` | `9` | Minimum LiDAR points needed to accept a wall candidate. | Set from expected point density at operating range. Increase to reject noise; decrease when walls are sparse. |
| `control.perception.ransac.iterations` | `80` | Number of random RANSAC hypotheses per scan. | Raise until detection quality stops improving in replay, bounded by control-loop CPU budget. |
| `control.perception.ransac.max_walls` | `4` | Maximum wall segments RANSAC should return. | Set from expected scene complexity. Two is often enough for a corridor; more helps junctions/clutter but costs CPU and filtering. |
| `control.perception.ransac.min_pair_sep_m` | `0.02` | Minimum separation between paired wall lines. | Set below the narrowest plausible corridor width but above duplicate-line noise; inspect wall telemetry. |

## Apex perception

| TOML path | Default | Meaning | How to get or tune it |
| --- | ---: | --- | --- |
| `control.perception.apex.median_window` | `5` | Median filter window for smoothing range discontinuities. | Use a small odd value. Increase for noisy scans; decrease if apex detection lags or misses sharp features. |
| `control.perception.apex.min_points` | `8` | Minimum number of points required for an apex/gap candidate. | Set from LiDAR angular density at the farthest useful lookahead. Lower for sparse scans; raise to reject isolated returns. |
| `control.perception.apex.min_forward_m` | `0.05` | Ignore candidate points closer than this in front of the car. | Set above bumper/mount self-returns and near-field artifacts. Confirm with raw LiDAR plots. |
| `control.perception.apex.min_range_jump_m` | `0.08` | Minimum adjacent range discontinuity to identify a gap/apex. | Estimate from scan noise and expected wall edges. Sweep in recorded scans; too low finds noise, too high misses openings. |
| `control.perception.apex.max_opposite_dist_error_m` | `0.75` | Maximum mismatch between opposite-side distances for a valid corridor/gap interpretation. | Tune from representative track scans. Tighten when false openings appear; loosen when real asymmetric corridors are rejected. |
| `control.perception.apex.max_lookahead_m` | `8.0` | Maximum forward lookahead distance used by apex targeting. | Set to useful LiDAR range on the track; reduce if far returns are noisy or cause late apex choices. |
| `control.perception.apex.min_lookahead_m` | `0.5` | Minimum forward lookahead distance used by apex targeting. | Set far enough that steering commands are not dominated by bumper-near noise; validate low-speed tight turns. |
| `control.perception.apex.lookahead_sensitivity` | `5.0` | Scaling between observed geometry and chosen lookahead distance. | Tune in sim/replay for smooth corner entry. Increase for farther lookahead at confidence; decrease if the car cuts corners or reacts late. |
| `control.perception.apex.side_lookahead_fov_deg` | `80.0` | Width of the side field-of-view window used for lookahead cues. | Set from LiDAR mounting and wall visibility. Narrow to reject rear/side clutter; widen when side walls are frequently missed. |
| `control.perception.apex.side_lookahead_center_deg` | `90.0` | Center angle of the side lookahead window, in degrees from forward. | Start near `90` for direct side returns. Shift if LiDAR mounting or body occlusion makes a different side sector more reliable. |

## Live hardware

| TOML path | Default | Meaning | How to get or tune it |
| --- | ---: | --- | --- |
| `live.lidar_port` | `"/dev/lidar"` | Serial device path used by the live RPLiDAR reader. | Use the udev-stable device symlink from deployment, or discover with `ls /dev/serial/by-id` and set the service config accordingly. |

## Simulator footprint

| TOML path | Default | Meaning | How to get or tune it |
| --- | ---: | --- | --- |
| `sim.length_m` | `0.42` | Full rectangular vehicle footprint length used for simulator wall collision and Foxglove footprint visualization. | Measure the assembled race-ready car nose-to-tail, including bodywork, bumpers, mounts, and any protruding parts that should count as a crash. |
| `sim.width_m` | `0.26` | Full rectangular vehicle footprint width used for simulator wall collision and Foxglove footprint visualization. | Measure the widest assembled width, usually outside tyre-to-tyre or body edge-to-body edge. Include protruding mounts/sensors if they should be protected. |

## Simulator kinematic model

| TOML path | Default | Meaning | How to get or tune it |
| --- | ---: | --- | --- |
| `sim.kinematic.wheelbase_m` | `0.33` | Axle-to-axle distance used by the kinematic bicycle model. | Measure front axle center to rear axle center on the assembled car. |
| `sim.kinematic.motor_gain_ms2` | `8.0` | Effective acceleration per full throttle command for the kinematic model. | Run straight throttle steps and fit low-speed `ax ≈ throttle * motor_gain`, using filtered IMU acceleration or velocity derivative. |
| `sim.kinematic.drag_k` | `0.5` | Quadratic drag/deceleration coefficient in `m/s² per (m/s)²`. | Coast down from several speeds with throttle zero and fit `dv/dt = -drag_k * v²`. |
| `sim.kinematic.accel_max_ms2` | `15.0` | Maximum longitudinal acceleration clamp for the kinematic model. | Use the 95th percentile of filtered full-throttle acceleration and braking deceleration, not single-sample spikes. |

## Simulator dynamic model geometry and inertia

| TOML path | Default | Meaning | How to get or tune it |
| --- | ---: | --- | --- |
| `sim.dynamic.wheelbase_m` | `0.33` | Axle-to-axle distance for the dynamic bicycle model. | Measure front axle center to rear axle center on the assembled car. |
| `sim.dynamic.lf_m` | `0.165` | Distance from center of mass to front axle. | Put front and rear axles on two scales. With wheelbase `L`, front weight `Wf`, rear weight `Wr`: `lr = L * Wf / (Wf + Wr)`, then `lf = L - lr`. |
| `sim.dynamic.lr_m` | `0.165` | Distance from center of mass to rear axle. | Use the same two-scale method: `lr = L * Wf / (Wf + Wr)`. |
| `sim.dynamic.track_width_m` | `0.18` | Left/right wheel-center spacing used to place individual tyre contact patches for force and yaw-moment calculations. | Measure center-to-center distance between the left and right tyres on an axle. Use the average of front and rear track widths unless they differ enough to justify extending the model. |
| `sim.dynamic.mass_kg` | `1.5` | Complete vehicle mass used in force equations. | Weigh the race-ready car with battery, body, mounts, sensors, and tyres installed. |
| `sim.dynamic.iz_kg_m2` | `0.04` | Yaw moment of inertia about the vertical center-of-mass axis. | Best: bifilar pendulum. Suspend by two strings of length `l` separated by `d`, measure yaw period `T`, use `Iz = m * g * (d/2)^2 * T^2 / (4π² * l)`. Rough fallback: `m * (length² + width²) / 12`. |
| `sim.dynamic.cf_n_per_rad` | `12.0` | Front axle cornering stiffness near zero slip. | Drive small steering sine/sweep runs below saturation and fit lateral/yaw equations from `ay`, `yaw_rate`, and steering logs. Use only small-slip data. |
| `sim.dynamic.cr_n_per_rad` | `10.0` | Rear axle cornering stiffness near zero slip. | Fit jointly with `cf_n_per_rad` from the same small-slip runs; rear stiffness is mostly identified through yaw-rate/yaw-acceleration residuals. |
| `sim.dynamic.motor_gain_ms2` | `8.0` | Dynamic-model acceleration per full effective throttle. | Fit straight-line logs with `ax = motor_gain * effective_throttle - drag_k * v²` after command latency and motor lag have settled. |
| `sim.dynamic.drag_k` | `0.5` | Dynamic-model quadratic drag/deceleration coefficient. | Coast-down fit with throttle zero, or joint fit with motor gain from straight-line acceleration/coast data. |
| `sim.dynamic.accel_max_ms2` | `20.0` | Clamp on dynamic-model longitudinal acceleration. | Use filtered max physical forward and braking acceleration from straight runs; set high enough not to clip normal operation, low enough to reject unrealistic fits. |

## Simulator actuator dynamics

| TOML path | Default | Meaning | How to get or tune it |
| --- | ---: | --- | --- |
| `sim.dynamic.servo.tau_s` | `0.05` | First-order steering lag time constant after any pure latency. | Command a small steering step and record actual wheel angle with high-speed video or an angle jig. `tau` is time to reach about 63% of final angle after movement starts. |
| `sim.dynamic.servo.rate_limit_rad_s` | `8.0` | Maximum actual steering slew rate. | Command a large steering step and measure maximum wheel-angle slope `Δangle / Δtime`; convert deg/s to rad/s. |
| `sim.dynamic.servo.backlash_rad` | `0.01` | Half-width of steering command deadband/backlash around the current wheel angle. | Slowly sweep steering left/right and measure the command range where wheel angle does not move. Use about half that dead zone. |
| `sim.dynamic.motor.tau_s` | `0.03` | First-order motor/ESC acceleration lag after any pure latency. | Command throttle steps on the ground and record IMU `ax`. `tau` is time to reach about 63% of settled acceleration after response starts. |
| `sim.dynamic.motor.deadband` | `0.05` | Symmetric throttle magnitude below which no drive/brake acceleration is applied. | Slowly ramp throttle from zero and find the smallest command that consistently moves the car or produces `ax` above IMU noise. Repeat forward and brake; choose a safe average. |
| `sim.dynamic.motor.brake_gain` | `1.4` | Ratio of braking acceleration authority to drive acceleration authority at equal effective command. | Compare filtered braking deceleration to forward acceleration at equal command after subtracting drag: `brake_gain ≈ abs(a_brake) / a_drive`. |

## Simulator drivetrain and low-speed stabilization

| TOML path | Default | Meaning | How to get or tune it |
| --- | ---: | --- | --- |
| `sim.dynamic.drivetrain.front_drive_fraction` | `0.5` | Fraction of drive/brake force applied at the front wheels before per-wheel traction limiting. `0.5` models a balanced 4WD drivetrain. | Use drivetrain layout or torque split. For a normal 4WD RC car start at `0.5`; bias upward/downward only if logs show persistent throttle-on understeer/oversteer that matches real behavior. |
| `sim.dynamic.drivetrain.traction_circle` | `true` | Enables per-wheel combined-slip limiting so drive/brake force consumes some lateral grip. | Keep `true` for realistic 4WD behavior. Temporarily disable only when fitting linear tyre stiffness from low-throttle, small-slip data. |
| `sim.dynamic.low_speed.blend_start_ms` | `0.05` | Speed below which dynamic yaw fully falls back to kinematic bicycle yaw. | Set near the lowest reliable rolling speed. Increase if the car still pivots near rest; decrease if very-low-speed turning looks too constrained. |
| `sim.dynamic.low_speed.blend_end_ms` | `0.35` | Speed above which the dynamic model is used without low-speed yaw blending. | Set above the speed where tyre slip-angle equations become numerically well-behaved. Raise if near-rest spin remains; lower if normal slow turns feel overly kinematic. |
| `sim.dynamic.low_speed.yaw_rate_margin` | `1.6` | Maximum allowed yaw rate as a margin over geometric bicycle yaw rate. | Increase only if real logs show the car can rotate faster than the steering-geometry prediction without sliding unrealistically; decrease if sim still spins too easily. |
| `sim.dynamic.low_speed.lateral_damping` | `10.0` | Extra lateral-velocity damping near rest, in `1/s`. | Increase if the car slides sideways or pivots at low speed; decrease if slow transitions look artificially sticky. |

## Simulator tyre, chassis, and latency

| TOML path | Default | Meaning | How to get or tune it |
| --- | ---: | --- | --- |
| `sim.dynamic.tyre.mu` | `1.2` | Tyre/surface friction coefficient used for lateral force saturation. | Run constant-radius/skidpad tests and increase speed until sliding: `mu ≈ v² / (R * 9.806)`, or use peak filtered lateral acceleration divided by `9.806`. |
| `sim.dynamic.tyre.pacejka_shape` | `1.3` | Shape factor for the simplified saturating tyre curve. | Leave near `1.3` unless fitting cornering data into saturation. Fit by minimizing lateral/yaw residuals from skidpad or aggressive slalom data. |
| `sim.dynamic.tyre.saturating` | `true` | Enables the saturating tyre model; `false` keeps linear `Fy = C * alpha` behavior. | Use `true` for realistic sim. Temporarily set `false` only when debugging or fitting linear `cf/cr` from small-slip data. |
| `sim.dynamic.chassis.cg_height_m` | `0.06` | Center-of-gravity height for longitudinal load transfer. | Estimate from CAD/component heights weighted by mass, or use a tilt-table rollover approximation `h ≈ half_track / tan(rollover_angle)`. Typical RC values are roughly `0.04–0.08 m`. |
| `sim.latency.latency_us` | `0` | Pure delay between command publication and model application, before servo/motor lag. | Record MCAP and cross-correlate command steps with response: steering command vs gyro/yaw response, throttle command vs IMU `ax`. Expected real path is often `20_000–50_000 µs`. |

## Start gate and arming

| TOML path | Default | Meaning | How to get or tune it |
| --- | ---: | --- | --- |
| `start_gate.udp_bind` | `"0.0.0.0"` | Interface/address for the UDP arming listener. | Use `0.0.0.0` for all interfaces, or bind to a specific car network interface for tighter exposure. |
| `start_gate.udp_port` | `4578` | UDP port for arm/disarm commands. | Pick a fixed port allowed by firewall/routing and keep it in sync with `nfe-arm` operator commands. |
| `start_gate.udp_token` | `"nfe"` | Shared token required in UDP arm/disarm payloads. | Set a race-day token known to operators. Avoid committing real secrets if the repo/config is shared. |
| `start_gate.gpio_enabled` | `false` | Enables GPIO arming input in addition to UDP when live mode allows it. | Enable only after wiring and testing the physical arming switch. Validate fail-safe behavior when disconnected. |
| `start_gate.gpio_pin` | `None` | Optional GPIO pin number for the arming input. | Set from the Pi wiring diagram and verify with `car-diag`/manual GPIO tests before enabling. |
| `start_gate.sim_start_delay_ms` | `100` | Delay before sim mode permits actuation unless overridden by CLI. | Keep small for fast tests, increase if recordings need a stable pre-roll before motion. |
| `start_gate.replay_start_delay_ms` | `0` | Delay before replay mode permits actuation unless overridden by CLI. | Usually `0` because replay is deterministic input playback; increase only when comparing startup transients. |

## Safety and degraded-sensor handling

| TOML path | Default | Meaning | How to get or tune it |
| --- | ---: | --- | --- |
| `safety.wheelbase_m` | `0.215` | Wheelbase used by safety arc projection. | Measure the physical car and keep this aligned with the safety model, even if sim uses a different experimental wheelbase. |
| `safety.half_channel_w_m` | `0.13` | Half-width of the collision corridor around the predicted path. | Use half vehicle width plus localization/perception margin. Validate by placing obstacles near the path edge. |
| `safety.t_lookahead_s` | `0.4` | Time horizon for forward safety lookahead. | Set from stopping time plus compute/sensor margin. Increase for higher speeds; decrease only if false positives dominate. |
| `safety.estop_min_m` | `0.25` | Minimum emergency-stop lookahead distance. | Set above bumper/sensor near-field noise and enough for low-speed stopping. Validate with static obstacle tests. |
| `safety.estop_max_m` | `1.5` | Maximum emergency-stop lookahead distance. | Set from useful LiDAR range and braking distance at maximum allowed speed. Too high may trip on irrelevant far clutter. |
| `safety.c_arc` | `0.5` | Curvature factor used by the parabolic arc safety approximation. | Keep default unless validating safety geometry against measured swept paths; adjust so projected channel covers the car footprint in turns. |
| `safety.tan_min` | `0.0717` | Lower bound on `tan(steering)` magnitude to avoid divide-by-zero/numerical instability. | Derive from the smallest steering angle where curved-path treatment is useful; keep near default unless safety arc math changes. |
| `safety.rearm_gap_m` | `0.20` | Extra clear distance required beyond estop length before clearing an active estop. | Set from desired hysteresis. Increase if estop chatters near obstacles; decrease if recovery is too slow after the path clears. |
| `safety.n_clean_ticks` | `5` | Consecutive clean ticks required to clear an active estop. | Convert desired clear dwell to ticks: `n_clean_ticks ≈ dwell_seconds * control.hz`. |
| `safety.min_front_points` | `4` | Minimum number of front-facing LiDAR points before the loop considers the scan usable. | Estimate from the minimum expected front-wall point count in normal operation; verify with recorded scans. |
| `safety.lidar_stale_ms` | `350` | LiDAR timestamp age threshold before blind/degraded handling. | Set several times the normal LiDAR scan period but below the time where driving blind is unsafe. Confirm with sensor dropout tests. |
| `safety.blind_grace_ms` | `350` | Grace window for coasting/holding steering before full safe state when blind. | Set from acceptable blind travel distance: `distance ≈ speed * grace_ms / 1000`. |
| `safety.imu_stale_ms` | `20` | IMU timestamp age threshold before creep-safe actuation. | Set from expected IMU sample period plus jitter. At `100 Hz`, `20 ms` allows one missed period. |
| `safety.sonar_stale_ms` | `100` | Sonar timestamp age threshold. | Set from sonar polling rate plus jitter if sonars are enabled; otherwise keep conservative. |
| `safety.escalate_at` | `6` | Leaky-bucket threshold for escalating repeated degraded/safety observations. | Tune from logs so one-off glitches do not escalate, but sustained faults do within the desired reaction time. |

## Initialization

| TOML path | Default | Meaning | How to get or tune it |
| --- | ---: | --- | --- |
| `init.timeout_secs` | `5` | Maximum time live mode waits for sensor readiness before failing init. | Set from worst-case sensor startup time on the Pi plus margin. Keep short enough that systemd/operator failures are obvious. |
