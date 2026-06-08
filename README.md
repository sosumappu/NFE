# NFE — Autonomous RC Car

**RPi 5 · NixOS PREEMPT_RT · Rust/Tokio control loop · deploy-rs**

```
Chassis: CARTEN M210 1/10
ESC:     Hobbywing QUICRUN 10BL120 120A
Motor:   Rocket 540 V3 4.5T Sensored
Servo:   INJORA INJS022 22KG Digital
LIDAR:   Slamtec RPLiDAR A1 (UART/USB)
IMU:     6-DOF (I2C)
Sonar:   HC-SR04 (GPIO)
UPS:     Geekworm X1200 (5.1V 5A, 2-cell 18650)
```

---

## Repository layout

```
flake.nix                       — Inputs, nixosConfigurations, deploy-rs, devShell
hosts/
  nfe/
    configuration.nix           — Host config: hostname=nfe, user=localhost, SSH, car.service, wifi, dt-params, dt-overlays
modules/
  preempt-rt.nix                — PREEMPT_RT kernel overlay + isolcpus=3, HZ=1000
  car-service.nix               — systemd car.service (SCHED_FIFO, core 3, watchdog)
packages/
  car-software/
    Cargo.toml                  — Rust deps: tokio, rppal, tracing-journald, libsystemd…
    default.nix                 — Nix derivation (cross-compiled aarch64)
    src/
      main.rs                   — Control loop: SensorFusion → LQR + PID → Actuate → Watchdog
```

---

## First-time setup

### 1. Test the package

```bash
nix develop
nix build ".#car-software"
```

```nix
users.users.localhost.openssh.authorizedKeys.keys = [
  "ssh-ed25519 AAAA... you@devmachine"
];
users.users.root.openssh.authorizedKeys.keys = [
  "ssh-ed25519 AAAA... you@devmachine"
];
```

### 2. Build the SD card image

```bash
nix build .#nixosConfigurations.nfe.config.system.build.sdImage
```

The image ends up at `result/sd-image/nixos-*.img.zst`. Decompress and flash:

```bash
zstd -d result/sd-image/nixos-*.img.zst -o nfe.img
sudo dd if=nfe.img of=/dev/sdX bs=4M status=progress conv=fsync
```

### 3. First boot

Insert SD, power on. The Pi will appear on your network as `nfe.local` via mDNS.

```bash
ssh localhost@nfe.local # or ssh localhost@<IP>
```

---

## Day-to-day workflow

### Enter the dev shell

```bash
nix develop
```

All tools (deploy, cargo, cyclictest, nixpkgs-fmt) are in PATH.

### Build only the car binary

```bash
nix build .#car-software
# result/bin/car  — aarch64-linux ELF
```

### Deploy to the Pi

```bash
deploy .#nfe
```

deploy-rs will:

1. Build the NixOS closure on your machine (cross-compiling to aarch64)
2. Copy the closure to nfe via SSH
3. Activate it (`nixos-rebuild switch`)
4. Wait up to 60 s for confirmation
5. **Auto-rollback** if activation fails or the Pi becomes unreachable

### Dry run (no changes applied)

```bash
deploy .#nfe -- --dry-activate
```

---

## Service lifecycle

All commands run **from your dev machine** via SSH:

```bash
# Status
ssh localhost@nfe.local 'systemctl status car'

# Start / stop / restart
ssh localhost@nfe.local 'systemctl start car'
ssh localhost@nfe.local 'systemctl stop car'
ssh localhost@nfe.local 'systemctl restart car'

# Live logs (tail)
ssh localhost@nfe.local 'journalctl -u car -f'

# Last 200 lines
ssh localhost@nfe.local 'journalctl -u car -n 200'

# Since last deploy
ssh localhost@nfe.local 'journalctl -u car --since "10 min ago"'

# Kernel RT + isolation messages
ssh localhost@nfe.local 'journalctl -k --grep="isolcpus\|PREEMPT_RT\|nohz"'
```

---

## Rollback

### deploy-rs automatic rollback

If the Pi doesn't respond within `confirmTimeout = 60` seconds, deploy-rs rolls
back automatically.

### Manual rollback over SSH

```bash
ssh root@nfe.local 'nixos-rebuild --rollback'
```

### Pick a specific generation

```bash
ssh localhost@nfe.local 'nix-env --list-generations --profile /nix/var/nix/profiles/system'
ssh root@nfe.local '/nix/var/nix/profiles/system-N-link/bin/switch-to-configuration switch'
```

---

## RT verification

Once deployed, verify the kernel and latency:

```bash
# Confirm PREEMPT_RT is active
ssh localhost@nfe.local 'uname -a'
# → Linux nfe 6.x.x-rt... #1 SMP PREEMPT_RT ...

# Confirm core 3 is isolated
ssh localhost@nfe.local 'cat /sys/devices/system/cpu/isolated'
# → 3

# Latency test on the isolated core (run for 60 s, SCHED_FIFO prio 80)
ssh localhost@nfe.local 'cyclictest -p 80 -t 1 -a 3 -n -q -D 60'
# Target: max latency < 100 µs; typical < 30 µs on RPi5 RT
```

---

## Tuning the controllers

Both the LQR gains and PID gains live in `packages/car-software/src/main.rs`:

```rust
// LQR gains [lateral_error, lateral_rate, heading_error, heading_rate]
Self { k: [0.8, 0.3, 1.2, 0.4] }

// PID (kp, ki, kd)
Pid::new(1.5, 0.05, 0.2)
```

Edit → `nix build .#car-software` → `deploy .#nfe` → `journalctl -u car -f`.

---

## Hardware wiring reference

| Signal        | BCM pin | Header pin | Notes          |
| ------------- | ------- | ---------- | -------------- |
| ESC PWM       | 18      | 12         | Hardware PWM0  |
| Servo PWM     | 19      | 35         | Hardware PWM1  |
| Sonar TRIG    | 23      | 16         |                |
| Sonar ECHO    | 24      | 18         | 3.3 V tolerant |
| IMU SDA (I2C) | 2       | 3          | 4.7 kΩ pull-up |
| IMU SCL (I2C) | 3       | 5          | 4.7 kΩ pull-up |
| LIDAR UART TX | 14      | 8          | RPLiDAR A1 RX  |
| LIDAR UART RX | 15      | 10         | RPLiDAR A1 TX  |

LIDAR can also use the USB adapter — it will appear as `/dev/lidar` via the udev
symlink rule in `modules/car-service.nix`.

---

## Safety notes

- The **watchdog** (3 missed 100 Hz ticks) puts ESC to neutral and servo to
  center.
- The **obstacle check** (< 0.3 m front LIDAR / sonar) triggers `safe_state()`
  immediately.
- systemd `WatchdogSec=5s` kills and restarts the service if it stops calling
  `sd_notify`.
- `deploy-rs` auto-rollback prevents a bad deploy from leaving the car
  unreachable.
