/// bin/monitor.rs — Live sensor monitor (runs on your Mac)
///
/// Connects to the Pi's streamer, receives `StreamFrame` datagrams, and
/// renders a live terminal dashboard using crossterm.
///
/// Usage
/// ─────
///   cargo run --bin car-monitor -- --pi car.local --port 9200
///   cargo run --bin car-monitor -- --pi 192.168.1.42
///
/// Display
/// ───────
///   ┌─ NFE Car Monitor ─────────────────────────────────────────────┐
///   │ IMU  ax:  0.012  ay: -0.003  az:  9.806  │ gz:  0.003 rad/s  │
///   │ Sonar  front: 1.23m  left: 2.10m  right: 0.87m               │
///   │ LiDAR  nearest: 0.45m @ -12°   points: 144                   │
///   │                                                               │
///   │  LIDAR sectors (each bar = 10°, height = distance)           │
///   │  ◄ 180°                          0°                   180° ► │
///   │  [sector visualisation]                                       │
///   └───────────────────────────────────────────────────────────────┘
///   Ctrl+C to exit

use std::{
    io::{self, Write},
    net::UdpSocket,
    time::Duration,
};

use anyhow::{Context, Result};

use car::stream::streamer::StreamFrame;

const REFRESH_MS: u64 = 100; // 10 Hz display refresh

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();

    let pi_host = args.windows(2)
        .find(|w| w[0] == "--pi")
        .map(|w| w[1].as_str())
        .unwrap_or("nfe.local");

    let port: u16 = args.windows(2)
        .find(|w| w[0] == "--port")
        .and_then(|w| w[1].parse().ok())
        .unwrap_or(car::stream::streamer::DEFAULT_PORT);

    let server_addr = format!("{pi_host}:{port}");

    let socket = UdpSocket::bind("0.0.0.0:0").context("bind")?;
    socket.set_read_timeout(Some(Duration::from_millis(REFRESH_MS)))?;

    // Register with the server
    socket.send_to(b"hello", &server_addr)
        .with_context(|| format!("cannot reach {server_addr}"))?;

    println!("\x1B[2J\x1B[H"); // clear screen
    println!("NFE Car Monitor — connected to {server_addr}");
    println!("Press Ctrl+C to exit\n");

    let mut recv_buf = vec![0u8; 65535];
    let mut frame_count: u64 = 0;

    loop {
        // Re-register periodically so the server doesn't time us out
        if frame_count % 50 == 0 {
            let _ = socket.send_to(b"ping", &server_addr);
        }

        let n = match socket.recv(&mut recv_buf) {
            Ok(n) => n,
            Err(ref e) if e.kind() == io::ErrorKind::WouldBlock
                       || e.kind() == io::ErrorKind::TimedOut => {
                print!("\r[waiting for data...]");
                io::stdout().flush().ok();
                continue;
            }
            Err(e) => return Err(e.into()),
        };

        if n < 2 { continue; }

        let len = u16::from_le_bytes([recv_buf[0], recv_buf[1]]) as usize;
        if n < 2 + len { continue; }

        let frame: StreamFrame = match bincode::deserialize(&recv_buf[2..2 + len]) {
            Ok(f) => f,
            Err(e) => { eprintln!("deserialize error: {e}"); continue; }
        };

        frame_count += 1;
        render(&frame, frame_count);
    }
}

fn render(f: &StreamFrame, n: u64) {
    let nearest = f.lidar_sectors.iter().enumerate()
        .filter(|(_, &d)| d < f32::MAX)
        .min_by(|a, b| a.1.partial_cmp(b.1).unwrap());

    let (nearest_deg, nearest_m) = nearest
        .map(|(i, d)| (i as f32 * 10.0 - 180.0, *d))
        .unwrap_or((0.0, f32::MAX));

    // Move cursor to top-left without clearing (avoids flicker)
    print!("\x1B[H");

    println!("╔══ NFE Car Monitor  frame #{n:<8} ══════════════════════════════╗");
    println!("║                                                               ║");
    println!("║  IMU   ax:{:>7.3}  ay:{:>7.3}  az:{:>7.3}  gz:{:>7.3} r/s  ║",
             f.imu.ax, f.imu.ay, f.imu.az, f.imu.gz);
    println!("║                                                               ║");
    println!("║  Sonar front:{:>6.2}m  left:{:>6.2}m  right:{:>6.2}m          ║",
             f.sonar_m[0], f.sonar_m[1], f.sonar_m[2]);
    println!("║                                                               ║");
    if nearest_m < f32::MAX {
        println!("║  LiDAR nearest: {:>5.2}m @ {:>6.1}°                            ║",
                 nearest_m, nearest_deg);
    } else {
        println!("║  LiDAR nearest: ---                                           ║");
    }
    println!("║                                                               ║");
    println!("║  Sectors (-180° ←──────────── 0° ────────────→ +180°)        ║");
    print!  ("║  ");
    render_sectors(&f.lidar_sectors);
    println!("  ║");
    println!("╚═══════════════════════════════════════════════════════════════╝");

    io::stdout().flush().ok();
}

fn render_sectors(sectors: &[f32; 36]) {
    for &dist in sectors.iter() {
        let bar = if dist >= f32::MAX {
            ' '
        } else if dist < 0.3 {
            '█' // danger zone
        } else if dist < 1.0 {
            '▓'
        } else if dist < 2.5 {
            '▒'
        } else {
            '░'
        };
        print!("{bar}");
    }
}
