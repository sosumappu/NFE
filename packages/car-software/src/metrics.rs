/// metrics.rs — Per-tick telemetry ring buffer + post-run cost computation
///
/// CMA-ES integration
/// ──────────────────
/// CMA-ES runs entirely in Rust via the `cmaes` crate — no Python required.
/// The objective function is in `bin/tune.rs`:
///
///   fn objective(theta: &[f64]) -> f64 {
///       let params = TuningParams::from_slice(theta);
///       let log = run_sim_episode(&world, params);
///       log.summarise().cost as f64
///   }
use std::{
    fs::File,
    io::{BufWriter, Write},
    path::Path,
    time::Instant,
};

#[derive(Clone, Copy, Debug, Default)]
pub struct TickMetrics {
    pub tick: u64,
    pub timestamp_us: u64,
    pub loop_us: u32,
    pub lateral_error_m: f32,
    pub heading_error_rad: f32,
    pub steering_rad: f32,
    pub throttle: f32,
    pub target_speed_ms: f32,
    pub current_speed_ms: f32,
    pub nearest_obstacle_m: f32,
    pub gz_rad_s: f32,
    pub vy_ms: f32,
    pub estop: bool,
    pub watchdog_miss: bool,
    pub sensor_fault: bool,
}

const RING_CAP: usize = 60_000;

pub struct MetricsLog {
    buf: Box<[TickMetrics]>,
    head: usize,
    count: usize,
    start: Instant,
}

impl Default for MetricsLog {
    fn default() -> Self {
        Self::new()
    }
}

impl MetricsLog {
    pub fn new() -> Self {
        // Avoid allocating a large array on the stack (which can cause a
        // stack overflow when running tests). Allocate on the heap via a
        // Vec and convert to a boxed slice instead.
        let vec = vec![TickMetrics::default(); RING_CAP];
        Self {
            buf: vec.into_boxed_slice(),
            head: 0,
            count: 0,
            start: Instant::now(),
        }
    }

    #[inline]
    pub fn push(&mut self, m: TickMetrics) {
        self.buf[self.head] = m;
        self.head = (self.head + 1) % RING_CAP;
        self.count += 1;
    }

    pub fn iter(&self) -> impl Iterator<Item = &TickMetrics> {
        let len = self.count.min(RING_CAP);
        let start = if self.count > RING_CAP { self.head } else { 0 };
        (0..len).map(move |i| &self.buf[(start + i) % RING_CAP])
    }

    pub fn len(&self) -> usize {
        self.count.min(RING_CAP)
    }

    pub fn is_empty(&self) -> bool {
        self.count == 0
    }
    pub fn total_ticks(&self) -> usize {
        self.count
    }
    pub fn elapsed_s(&self) -> f32 {
        self.start.elapsed().as_secs_f32()
    }

    pub fn summarise(&self) -> RunCost {
        let mut sum_lat2 = 0.0f64;
        let mut sum_head2 = 0.0f64;
        let mut sum_spd2 = 0.0f64;
        let mut sum_jerk2 = 0.0f64;
        let mut prev_steer = 0.0f32;
        let mut n_estop = 0u32;
        let mut n_miss = 0u32;
        let mut total_us = 0u64;
        let mut max_loop = 0u32;
        let mut n = 0u64;

        for m in self.iter() {
            sum_lat2 += (m.lateral_error_m as f64).powi(2);
            sum_head2 += (m.heading_error_rad as f64).powi(2);
            sum_spd2 += ((m.target_speed_ms - m.current_speed_ms) as f64).powi(2);
            sum_jerk2 += ((m.steering_rad - prev_steer) as f64).powi(2);
            prev_steer = m.steering_rad;
            if m.estop {
                n_estop += 1;
            }
            if m.watchdog_miss {
                n_miss += 1;
            }
            total_us += m.loop_us as u64;
            if m.loop_us > max_loop {
                max_loop = m.loop_us;
            }
            n += 1;
        }

        if n == 0 {
            return RunCost::default();
        }
        let fn_ = n as f64;

        let lateral_rms = (sum_lat2 / fn_).sqrt() as f32;
        let heading_rms = (sum_head2 / fn_).sqrt() as f32;
        let speed_rms = (sum_spd2 / fn_).sqrt() as f32;
        let jerk_rms = (sum_jerk2 / fn_).sqrt() as f32;
        let mean_loop = (total_us / n) as f32;

        let cost = 1.0 * lateral_rms
            + 0.5 * heading_rms
            + 0.3 * speed_rms
            + 0.2 * jerk_rms
            + 5.0 * (n_estop as f32 / fn_ as f32)
            + 2.0 * (n_miss as f32 / fn_ as f32);

        RunCost {
            cost,
            lateral_rms,
            heading_rms,
            speed_rms,
            jerk_rms,
            mean_loop_us: mean_loop,
            max_loop_us: max_loop,
            n_estop,
            n_watchdog_miss: n_miss,
            ticks: n,
        }
    }

    pub fn to_csv(&self, path: impl AsRef<Path>) -> std::io::Result<()> {
        let mut w = BufWriter::new(File::create(path)?);
        writeln!(
            w,
            "tick,timestamp_us,loop_us,lateral_error_m,heading_error_rad,\
                     steering_rad,throttle,target_speed_ms,current_speed_ms,\
                     nearest_obstacle_m,gz_rad_s,vy_ms,estop,watchdog_miss,sensor_fault"
        )?;
        for m in self.iter() {
            writeln!(
                w,
                "{},{},{},{:.4},{:.4},{:.4},{:.4},{:.3},{:.3},{:.3},{:.4},{:.4},{},{},{}",
                m.tick,
                m.timestamp_us,
                m.loop_us,
                m.lateral_error_m,
                m.heading_error_rad,
                m.steering_rad,
                m.throttle,
                m.target_speed_ms,
                m.current_speed_ms,
                m.nearest_obstacle_m,
                m.gz_rad_s,
                m.vy_ms,
                m.estop as u8,
                m.watchdog_miss as u8,
                m.sensor_fault as u8
            )?;
        }
        Ok(())
    }

    pub fn cost_to_json(&self, path: impl AsRef<Path>) -> std::io::Result<()> {
        let c = self.summarise();
        let mut w = BufWriter::new(File::create(path)?);
        writeln!(
            w,
            r#"{{"cost":{:.6},"lateral_rms":{:.6},"heading_rms":{:.6},"speed_rms":{:.6},"jerk_rms":{:.6},"mean_loop_us":{:.1},"max_loop_us":{},"n_estop":{},"n_watchdog_miss":{},"ticks":{}}}"#,
            c.cost,
            c.lateral_rms,
            c.heading_rms,
            c.speed_rms,
            c.jerk_rms,
            c.mean_loop_us,
            c.max_loop_us,
            c.n_estop,
            c.n_watchdog_miss,
            c.ticks
        )?;
        Ok(())
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct RunCost {
    pub cost: f32,
    pub lateral_rms: f32,
    pub heading_rms: f32,
    pub speed_rms: f32,
    pub jerk_rms: f32,
    pub mean_loop_us: f32,
    pub max_loop_us: u32,
    pub n_estop: u32,
    pub n_watchdog_miss: u32,
    pub ticks: u64,
}

impl std::fmt::Display for RunCost {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "cost={:.4}  lat={:.3}m  head={:.3}rad  spd={:.3}m/s  \
             jerk={:.4}  loop_mean={:.0}µs  loop_max={}µs  estop={}  wd_miss={}  ticks={}",
            self.cost,
            self.lateral_rms,
            self.heading_rms,
            self.speed_rms,
            self.jerk_rms,
            self.mean_loop_us,
            self.max_loop_us,
            self.n_estop,
            self.n_watchdog_miss,
            self.ticks
        )
    }
}
