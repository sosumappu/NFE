/// bin/diag.rs — NFE hardware diagnostic tool
///
/// Run on the RPi after first boot to verify each sensor before ever starting
/// the control loop.
///
/// Usage:
///   car-diag imu              continuous IMU readings at 10 Hz
///   car-diag imu --once       single reading + pass/fail verdict
///   car-diag lidar            live point cloud stats (rev count, nearest, spread)
///   car-diag lidar --once     one revolution + point cloud dump
///   car-diag sonar            all 3 HC-SR04 pinging continuously
///   car-diag sonar --once     one ping per sensor + pass/fail verdict
///   car-diag all              run all sensors for 3 s, print summary pass/fail
///
/// Exit codes:
///   0  all checked sensors pass
///   1  one or more sensors failed or timed out

use std::{
    f32::consts::PI,
    io::{self, Write},
    thread,
    time::{Duration, Instant},
};

use rppal::{gpio::Gpio, i2c::I2c, pwm::Pwm};
use serialport::SerialPort;

// ── Shared constants (mirror the main binary) ──────────────────────────────

const IMU_ADDR:        u16 = 0x68;
const LIDAR_PORT:      &str = "/dev/lidar";
const LIDAR_BAUD:      u32 = 115_200;
const SONAR_TIMEOUT:   Duration = Duration::from_millis(30);
const SOUND_SPEED:     f32 = 343.0;

// HC-SR04 pin pairs [TRIG, ECHO]
const SONAR_PINS: [(u8, u8, &str); 3] = [
    (23, 24, "front"),
    (25,  8, "front-left"),
    ( 7,  1, "front-right"),
];

// ── Entry ──────────────────────────────────────────────────────────────────

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let cmd   = args.get(1).map(String::as_str).unwrap_or("help");
    let once  = args.iter().any(|a| a == "--once");

    match cmd {
        "imu"   => run_imu(once),
        "lidar" => run_lidar(once),
        "sonar" => run_sonar(once),
        "all"   => run_all(),
        _       => {
            eprintln!("Usage: car-diag <imu|lidar|sonar|all> [--once]");
            std::process::exit(1);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  IMU diagnostic
// ═══════════════════════════════════════════════════════════════════════════

fn run_imu(once: bool) {
    println!("── IMU diagnostic (I2C-1, addr=0x{:02X}) ──", IMU_ADDR);

    let mut i2c = match I2c::new() {
        Ok(i) => i,
        Err(e) => { fail(&format!("cannot open /dev/i2c-1: {e}")); }
    };

    if let Err(e) = i2c.set_slave_address(IMU_ADDR) {
        fail(&format!("set_slave_address: {e}"));
    }

    // Read WHO_AM_I
    let who = match i2c.smbus_read_byte(0x75) {
        Ok(v) => v,
        Err(e) => { fail(&format!("WHO_AM_I read failed: {e}\n→ check wiring, pull-ups, I2C enabled in config.txt")); }
    };
    println!("  WHO_AM_I = 0x{who:02X}  (expected 0x68 or 0x70)");
    if who != 0x68 && who != 0x70 && who != 0x19 {
        fail(&format!("unexpected WHO_AM_I=0x{who:02X} — wrong device or address"));
    }

    // Wake device
    i2c.smbus_write_byte(0x6B, 0x00).expect("PWR_MGMT_1 write");
    thread::sleep(Duration::from_millis(100));
    // Accel ±4g, Gyro ±500°/s
    i2c.smbus_write_byte(0x1C, 0x08).ok();
    i2c.smbus_write_byte(0x1B, 0x08).ok();

    println!("  Device initialised — reading samples:");
    println!("  {:>8} {:>8} {:>8}   {:>8} {:>8} {:>8}", "ax", "ay", "az", "gx", "gy", "gz");

    let mut buf = [0u8; 14];
    let mut count = 0u32;

    loop {
        i2c.block_read(0x3B, &mut buf).expect("burst read");

        let ax = i16::from_be_bytes([buf[0], buf[1]]) as f32 / 8192.0 * 9.806;
        let ay = i16::from_be_bytes([buf[2], buf[3]]) as f32 / 8192.0 * 9.806;
        let az = i16::from_be_bytes([buf[4], buf[5]]) as f32 / 8192.0 * 9.806;
        let gx = i16::from_be_bytes([buf[8], buf[9]]) as f32 / 65.5 * (PI / 180.0);
        let gy = i16::from_be_bytes([buf[10],buf[11]]) as f32 / 65.5 * (PI / 180.0);
        let gz = i16::from_be_bytes([buf[12],buf[13]]) as f32 / 65.5 * (PI / 180.0);

        print!("\r  {:>8.3} {:>8.3} {:>8.3}   {:>8.4} {:>8.4} {:>8.4}  ",
               ax, ay, az, gx, gy, gz);
        io::stdout().flush().ok();

        // Sanity checks
        let gravity_ok = (az.abs() - 9.81).abs() < 2.0;  // az ≈ ±9.81 when flat
        let noise_ok   = ax.abs() < 5.0 && ay.abs() < 5.0;

        if once {
            println!();
            if gravity_ok && noise_ok {
                pass("IMU");
            } else if !gravity_ok {
                fail(&format!("az={az:.2} m/s² — expected ≈ ±9.81 (is the IMU flat? is it powered?)"));
            } else {
                fail(&format!("ax={ax:.2} ay={ay:.2} — high lateral noise (loose connection?)"));
            }
            return;
        }

        count += 1;
        thread::sleep(Duration::from_millis(100));

        if count % 50 == 0 {
            println!();
            println!("  [gravity_ok={gravity_ok}  noise_ok={noise_ok}]");
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  LIDAR diagnostic
// ═══════════════════════════════════════════════════════════════════════════

fn run_lidar(once: bool) {
    println!("── LIDAR diagnostic ({LIDAR_PORT} @ {LIDAR_BAUD}baud) ──");

    let mut port: Box<dyn SerialPort> = match serialport::new(LIDAR_PORT, LIDAR_BAUD)
        .timeout(Duration::from_millis(500))
        .open()
    {
        Ok(p) => p,
        Err(e) => { fail(&format!("cannot open {LIDAR_PORT}: {e}\n→ check USB cable, udev rule, /dev/lidar symlink")); }
    };

    // Reset + start scan
    port.write_all(&[0xA5, 0x40]).ok();
    thread::sleep(Duration::from_millis(100));
    port.write_all(&[0xA5, 0x20]).ok();
    port.flush().ok();

    // Drain 7-byte descriptor
    let mut desc = [0u8; 7];
    if port.read_exact(&mut desc).is_err() {
        fail("no response to START_SCAN — is the motor spinning? check 5V supply");
    }
    println!("  Response descriptor: {:02x?}", desc);

    let mut buf = [0u8; 5];
    let mut rev = 0u32;
    let mut total_points = 0u32;
    let mut min_dist = f32::MAX;
    let mut prev_start = false;
    let mut sectors = [f32::MAX; 8];
    let start = Instant::now();

    println!("  {:>5} {:>8} {:>8} {:>10} {:>10}", "rev", "points", "min_m", "front_m", "elapsed_s");

    loop {
        if port.read_exact(&mut buf).is_err() { continue; }

        let start_flag = (buf[0] & 0x01) != 0;
        let quality    = buf[0] >> 1;
        let angle_q6   = ((buf[2] as u16) << 7) | ((buf[1] as u16) >> 1);
        let angle_deg  = angle_q6 as f32 / 64.0;
        let dist_q2    = (buf[4] as u16) << 8 | buf[3] as u16;
        let dist_m     = dist_q2 as f32 / 4000.0;

        if start_flag && !prev_start && rev > 0 {
            println!("  {:>5} {:>8} {:>8.3} {:>10.3} {:>10.1}",
                     rev, total_points, min_dist, sectors[0], start.elapsed().as_secs_f32());
            sectors    = [f32::MAX; 8];
            min_dist   = f32::MAX;
            total_points = 0;

            if once && rev >= 1 {
                println!();
                // Basic sanity: did we get any points and is there a plausible return?
                if rev > 0 && min_dist < 6.0 {
                    pass("LIDAR");
                } else {
                    fail("LIDAR returned no valid points — check motor power and USB connection");
                }
                return;
            }
        }
        if start_flag { rev += 1; }
        prev_start = start_flag;

        if quality > 0 && dist_m > 0.1 && dist_m < 6.0 {
            total_points += 1;
            if dist_m < min_dist { min_dist = dist_m; }
            let sector = ((angle_deg + 22.5) / 45.0) as usize % 8;
            if dist_m < sectors[sector] { sectors[sector] = dist_m; }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Sonar diagnostic
// ═══════════════════════════════════════════════════════════════════════════

fn run_sonar(once: bool) {
    println!("── Sonar diagnostic (3× HC-SR04) ──");

    let gpio = match Gpio::new() {
        Ok(g) => g,
        Err(e) => { fail(&format!("GPIO init failed: {e}")); }
    };

    // Print header
    println!("  {:>12} {:>6} {:>6} {:>6}   verdict",
             "sensor", "trig", "echo", "dist_m");

    let iterations = if once { 1 } else { usize::MAX };
    let mut all_ok = true;

    for _ in 0..iterations {
        for &(trig_pin, echo_pin, label) in &SONAR_PINS {
            let dist = ping_sonar(&gpio, trig_pin, echo_pin);
            let verdict = match dist {
                Some(d) if d >= 0.02 && d <= 4.0 => "OK",
                Some(_) => "OUT_OF_RANGE",
                None    => { all_ok = false; "TIMEOUT" }
            };
            println!("  {:>12} {:>6} {:>6} {:>6}   {}",
                     label, trig_pin, echo_pin,
                     dist.map_or("  ---".into(), |d| format!("{d:.3}")),
                     verdict);
        }

        if once {
            println!();
            if all_ok { pass("all sonars"); } else { fail("one or more sonars timed out"); }
            return;
        }

        print!("  ---\r");
        io::stdout().flush().ok();
        thread::sleep(Duration::from_millis(100));
    }
}

/// Returns measured distance in metres, or None on timeout.
fn ping_sonar(gpio: &Gpio, trig_pin: u8, echo_pin: u8) -> Option<f32> {
    let mut trig = gpio.get(trig_pin).ok()?.into_output();
    let echo     = gpio.get(echo_pin).ok()?.into_input();

    // 15 µs trigger pulse
    trig.set_high();
    spin_us(15);
    trig.set_low();

    // Wait for ECHO high
    let t0 = Instant::now();
    while echo.is_low() {
        if t0.elapsed() > SONAR_TIMEOUT { return None; }
    }
    let rise = Instant::now();

    // Wait for ECHO low
    while echo.is_high() {
        if rise.elapsed() > SONAR_TIMEOUT { return None; }
    }
    let fall = Instant::now();

    let echo_s = (fall - rise).as_secs_f32();
    Some(echo_s * SOUND_SPEED / 2.0)
}

fn spin_us(us: u64) {
    let d = Instant::now() + Duration::from_micros(us);
    while Instant::now() < d { std::hint::spin_loop(); }
}

// ═══════════════════════════════════════════════════════════════════════════
//  All sensors summary
// ═══════════════════════════════════════════════════════════════════════════

fn run_all() {
    println!("══ NFE hardware diagnostic ══");
    println!("Running all sensors for 3 s each...\n");

    // We run each subtest in --once mode and capture exit via a child process
    // approach. Simpler: just call each function and catch panics.

    let imu_ok   = std::panic::catch_unwind(|| run_imu(true)).is_ok();
    println!();
    let lidar_ok = std::panic::catch_unwind(|| run_lidar(true)).is_ok();
    println!();
    let sonar_ok = std::panic::catch_unwind(|| run_sonar(true)).is_ok();
    println!();

    println!("══ Summary ══");
    println!("  IMU:   {}", if imu_ok   { "✓ PASS" } else { "✗ FAIL" });
    println!("  LIDAR: {}", if lidar_ok { "✓ PASS" } else { "✗ FAIL" });
    println!("  Sonar: {}", if sonar_ok { "✓ PASS" } else { "✗ FAIL" });
    println!();

    if imu_ok && lidar_ok && sonar_ok {
        println!("All sensors PASS — safe to start car.service");
        std::process::exit(0);
    } else {
        println!("One or more sensors FAILED — do not start car.service");
        std::process::exit(1);
    }
}

// ── Helpers ────────────────────────────────────────────────────────────────

fn pass(label: &str) {
    println!("  ✓ PASS  [{label}]");
    std::process::exit(0);
}

fn fail(msg: &str) -> ! {
    println!("  ✗ FAIL  {msg}");
    std::process::exit(1);
}
