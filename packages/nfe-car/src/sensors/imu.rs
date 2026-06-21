#![cfg(target_os = "linux")]
/// sensors/imu.rs — IMU reader thread (I2C, MPU-6050 / MPU-6500 compatible)
///
/// Spawns a dedicated OS thread that:
///   1. Opens /dev/i2c-1 via rppal::i2c
///   2. Initialises the IMU (wake from sleep, set FSR, DLPF)
///   3. Reads accel + gyro registers in a tight loop at ~1 kHz
///   4. Converts raw counts to SI units (m/s², rad/s)
///   5. Writes the result into SharedState via update_imu()
///
/// The thread runs faster than the 100 Hz control loop on purpose — the
/// control loop always reads the *latest* sample, not a queued one.
/// On RPi5 a blocking I2C read of 14 bytes at 400 kHz takes ~350 µs.
use std::{sync::Arc, thread, time::Duration};

use crate::time::monotonic_us;
use tracing::{error, info, warn};

use rppal::i2c::I2c;

use crate::init::ReadySignal;
use crate::state::SensorStateWriter;
use crate::types::ImuSample;

// ── MPU-6050 register map (also valid for MPU-6500) ────────────────────────

const IMU_ADDR: u16 = 0x68; // AD0 low; use 0x69 if AD0 pulled high
const REG_PWR_MGMT_1: u8 = 0x6B;
const REG_CONFIG: u8 = 0x1A;
const REG_GYRO_CFG: u8 = 0x1B;
const REG_ACCEL_CFG: u8 = 0x1C;
const REG_ACCEL_XOUT: u8 = 0x3B; // first of 14 consecutive bytes
const REG_WHO_AM_I: u8 = 0x75;

// Full-scale ranges we configure:
//   Accel ±4g  → sensitivity 8192 LSB/g
//   Gyro  ±500°/s → sensitivity 65.5 LSB/(°/s)
const ACCEL_SENSITIVITY: f32 = 8192.0; // LSB per g
const GYRO_SENSITIVITY: f32 = 65.5; // LSB per °/s
const G_TO_MS2: f32 = 9.80665;
const DEG_TO_RAD: f32 = std::f32::consts::PI / 180.0;

// Read loop target: 500 Hz (2 ms sleep) — fast enough that the control loop
// always has a fresh sample without burning a full CPU core.
const LOOP_SLEEP: Duration = Duration::from_millis(2);

// ── Thread entry point ──────────────────────────────────────────────────────

pub fn spawn(state: Arc<dyn SensorStateWriter>, ready: ReadySignal) -> thread::JoinHandle<()> {
    thread::Builder::new()
        .name("imu-reader".into())
        .stack_size(128 * 1024)
        .spawn(move || run(state, ready))
        .expect("failed to spawn imu-reader thread")
}

fn run(state: Arc<dyn SensorStateWriter>, ready: ReadySignal) {
    info!("imu: starting on /dev/i2c-1 addr=0x{:02X}", IMU_ADDR);

    let mut ready = Some(ready); // consumed on first valid sample

    loop {
        if state.is_shutdown() {
            break;
        }

        // Warn : might return Ok() for other reason than state.is_shutdown
        match open_and_read(&state, &mut ready) {
            Ok(()) => break,
            Err(e) => {
                error!("imu: error — {e} — retrying in 1s");
                state
                    .sensor_health()
                    .imu
                    .store(true, std::sync::atomic::Ordering::Relaxed);
                thread::sleep(Duration::from_secs(1));
                state
                    .sensor_health()
                    .imu
                    .store(false, std::sync::atomic::Ordering::Relaxed);
            }
        }
    }

    info!("imu: thread exiting");
}

fn open_and_read(
    state: &Arc<dyn SensorStateWriter>,
    ready: &mut Option<ReadySignal>,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut i2c = I2c::new()?;
    i2c.set_slave_address(IMU_ADDR)?;

    // Verify device identity
    let who = i2c.smbus_read_byte(REG_WHO_AM_I)?;
    if who != 0x68 && who != 0x70 && who != 0x19 {
        return Err(format!("imu: unexpected WHO_AM_I=0x{who:02X}").into());
    }
    info!("imu: WHO_AM_I=0x{who:02X} — device confirmed");

    // Wake up (clear SLEEP bit), use internal oscillator
    i2c.smbus_write_byte(REG_PWR_MGMT_1, 0x00)?;
    thread::sleep(Duration::from_millis(100));

    // DLPF bandwidth 44 Hz (CONFIG register = 3) — balances noise vs latency
    i2c.smbus_write_byte(REG_CONFIG, 0x03)?;

    // Gyro FSR ±500 °/s (FS_SEL = 1)
    i2c.smbus_write_byte(REG_GYRO_CFG, 0x08)?;

    // Accel FSR ±4g (AFS_SEL = 1)
    i2c.smbus_write_byte(REG_ACCEL_CFG, 0x08)?;

    info!("imu: initialised — accel ±4g, gyro ±500°/s, DLPF 44Hz");

    let mut buf = [0u8; 14]; // ACCEL_X..ACCEL_Z, TEMP, GYRO_X..GYRO_Z (all 16-bit BE)

    loop {
        if state.is_shutdown() {
            break;
        }

        // Burst-read 14 bytes starting at ACCEL_XOUT_H
        i2c.block_read(REG_ACCEL_XOUT, &mut buf)?;

        let ax_raw = i16::from_be_bytes([buf[0], buf[1]]);
        let ay_raw = i16::from_be_bytes([buf[2], buf[3]]);
        let az_raw = i16::from_be_bytes([buf[4], buf[5]]);
        // buf[6..7] = temperature, skip
        let gx_raw = i16::from_be_bytes([buf[8], buf[9]]);
        let gy_raw = i16::from_be_bytes([buf[10], buf[11]]);
        let gz_raw = i16::from_be_bytes([buf[12], buf[13]]);

        let ts_us = monotonic_us();

        state.update_imu(ImuSample {
            ax: (ax_raw as f32 / ACCEL_SENSITIVITY) * G_TO_MS2,
            ay: (ay_raw as f32 / ACCEL_SENSITIVITY) * G_TO_MS2,
            az: (az_raw as f32 / ACCEL_SENSITIVITY) * G_TO_MS2,
            gx: (gx_raw as f32 / GYRO_SENSITIVITY) * DEG_TO_RAD,
            gy: (gy_raw as f32 / GYRO_SENSITIVITY) * DEG_TO_RAD,
            gz: (gz_raw as f32 / GYRO_SENSITIVITY) * DEG_TO_RAD,
            timestamp_us: ts_us,
        });

        // Signal readiness on first successful sample (Option::take is a no-op after that)
        if let Some(r) = ready.take() {
            r.signal();
        }

        thread::sleep(LOOP_SLEEP);
    }

    Ok(())
}
