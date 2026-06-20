# NFE Development Workflow

## Quick reference

```bash
# On the car (Pi)
car                                    # normal run
car --record sessions/$(date +%s).bin  # record a session
car --stream                           # broadcast sensors over UDP
car --stream --record sessions/now.bin # both at once

# On your Mac
cargo run --bin car-monitor -- --pi nfe.local              # live dashboard
cargo run --bin car-monitor -- --pi nfe.local --full-cloud # + raw cloud

cargo run --release -- --replay sessions/1234.bin        # replay at real speed
cargo run --release -- --replay sessions/1234.bin --fast # replay as fast as possible
```

---

## Development tiers

### Tier 1 — Algorithm dev (fully offline, fastest iteration)

Record a session on the car, copy it to your Mac, iterate:

```bash
# On Pi: record 60 seconds
ssh nfe.local 'car --record /tmp/session.bin'
# wait ~60s, then Ctrl+C or SIGTERM

# Copy to Mac
scp nfe.local:/tmp/session.bin sessions/

# Iterate on your Mac — no hardware needed
cargo run -- --replay sessions/session.bin --fast
```

Your `control_loop` runs exactly as it does live. Tune LQR gains, PID constants,
ESTOP thresholds — every change is testable without touching the Pi. `--fast`
removes all timing sleeps so you can run thousands of iterations quickly, or add
`cargo test` with a `ReplaySource` for automated regression.

### Tier 2 — Live observation (Pi running, you watching)

```bash
# On Pi — start with streaming enabled
ssh pi 'systemctl stop car && car --stream'

# On Mac — open the live dashboard
cargo run --bin car-monitor -- --pi nfe.local
```

The dashboard shows IMU, sonar distances, and a 36-sector LiDAR overview
refreshed at 20 Hz. The Pi's control loop is unaffected — the streamer runs in a
separate thread with non-blocking channel sends.

### Tier 3 — Fast binary deploy (code change → on-car in ~30s)

Skip full NixOS rebuild for code-only changes:

```bash
# Cross-compile on Mac
cross build --release --target aarch64-unknown-linux-gnu

# Deploy just the binary
rsync -avz target/aarch64-unknown-linux-gnu/release/car pi@car.local:/run/car/car-new
ssh pi@car.local 'mv /run/car/car-new /run/car/car && systemctl restart car'
```

### Tier 4 — Full NixOS deploy

Only when changing: kernel params, udev rules, new system deps, flake.

```bash
deploy-rs .#car
```

---

## Actuator modes

The `ActuatorFactory` auto-detects hardware at startup:

| Situation                             | Actuator chosen  | Log output                                                    |
| ------------------------------------- | ---------------- | ------------------------------------------------------------- |
| PWM HAT present, dtoverlay configured | `RealActuator`   | `actuator: PWM hardware detected`                             |
| No PWM HAT / Mac / replay             | `DryRunActuator` | `actuator: PWM hardware not available — using DryRunActuator` |

Either way, every `set_throttle` / `set_steering` / `safe_state` call goes
through `LoggingActuator`, which traces commands at debug level. During replay
or dry-run you'll see exactly what the car would have done.

Change `ActuatorFactory::build(10)` to `build(1)` in `main.rs` to log every
single actuation (useful for debugging the control loop outputs).

---

## Sensor degradation

If a sensor is not connected, the factory skips it gracefully:

| Sensor missing                 | Behaviour                                                            |
| ------------------------------ | -------------------------------------------------------------------- |
| LiDAR (/dev/lidar not present) | Cloud is empty; `obstacle_closer_than` returns false for LiDAR arc   |
| IMU (I2C open fails)           | `ImuSample` stays at zero; gz=0 feeds LQR zero yaw rate              |
| Individual sonar pin fails     | That slot stays at `f32::MAX` (no obstacle); other sonars unaffected |
| GPIO unavailable entirely      | All 3 sonar slots stay at `f32::MAX`                                 |

The ESTOP check (`obstacle_closer_than(0.30)`) is safe in all cases: missing
sensors appear as "no obstacle" (`f32::MAX > 0.30`). This is a fail-open design
— if you want fail-safe, add a `sensor_fault` check before the control loop
starts.

---

## Replay file format

```
[u32 magic=0xCAR55E55] [u32 version=1]
[u32 frame_len] [bincode(TimestampedFrame)]
[u32 frame_len] [bincode(TimestampedFrame)]
...
[u32 end_magic=0xDEADBEEF]
```

Frames are independent — a truncated file (crash, power loss) is recoverable up
to the last complete frame. The replayer skips corrupt frames with a warning.

`TimestampedFrame.frame` is a `SensorFrame` enum: `Lidar(LidarCloud)`,
`Imu(ImuSample)`, or `Sonar { front, left, right }`.

The replayer emits one `SensorSnapshot` per `Lidar` frame (one revolution ≈ 6
Hz), folding in the most recent IMU and sonar readings. This matches the
effective update rate of the live control loop.

---

## UDP stream format

Each datagram: `[u16 len_bytes][bincode(StreamFrame)]`

`StreamFrame` contains:

- Full `ImuSample` (ax, ay, az, gx, gy, gz)
- `sonar_m: [f32; 3]`
- `lidar_sectors: [f32; 36]` — nearest distance per 10° arc (min-filtered)
- `lidar_cloud: Vec<LidarPoint>` — only when `--full-cloud` flag is set

Subscribe by sending any UDP datagram to port 9200. Re-send every ~5s to avoid
the 10s subscriber timeout.
