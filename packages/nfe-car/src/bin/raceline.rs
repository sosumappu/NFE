use std::path::PathBuf;

use anyhow::{anyhow, bail, Context, Result};
use nfe_algo::raceline::solver::{solve_min_curvature, RaceLineSolverParams};
use nfe_algo::raceline::velocity::{compute_velocity_profile, VelocityProfileParams};
use nfe_core::raceline::RaceLine;
use nfe_runtime::foxglove_scenes::{build_map_scene, build_raceline_scene};
use nfe_runtime::raceline_preview::{load_track_map, TrackFrame};
use nfe_runtime::sinks::mcap::McapSceneWriter;

struct Args {
    world: PathBuf,
    out: PathBuf,
    frame: TrackFrame,
    params: RaceLineSolverParams,
    stats: bool,
}

fn main() -> Result<()> {
    let args = Args::parse()?;
    let map = load_track_map(&args.world, args.frame)
        .with_context(|| format!("failed to load track map {}", args.world.display()))?;
    let raceline = solve_min_curvature(&map, &args.params).map_err(|e| {
        anyhow!(
            "raceline solver failed for {} in {} frame: {:?}",
            args.world.display(),
            args.frame.as_str(),
            e
        )
    })?;

    if args.stats {
        print_velocity_stats(&raceline, &args.params)?;
    }

    let timestamp_us = 1_000_000;
    let map_scene = build_map_scene(timestamp_us, "map", &map);
    let raceline_scene = build_raceline_scene(timestamp_us, "map", &raceline);

    let mut writer = McapSceneWriter::create(&args.out)?;
    writer.write_map_scene(timestamp_us, &map_scene)?;
    writer.write_raceline_scene(timestamp_us, &raceline_scene)?;
    writer.finish()?;

    println!(
        "wrote {} with {} map walls and {} raceline points",
        args.out.display(),
        map.boundaries.walls.len(),
        raceline.points.len()
    );
    Ok(())
}

impl Args {
    fn parse() -> Result<Self> {
        let mut raw = std::env::args().skip(1);
        let Some(first) = raw.next() else {
            print_help();
            bail!("missing world path");
        };
        if first == "--help" || first == "-h" {
            print_help();
            std::process::exit(0);
        }

        let mut out = None;
        let mut args = Args {
            world: PathBuf::from(first),
            out: PathBuf::new(),
            frame: TrackFrame::Start,
            params: RaceLineSolverParams::default(),
            stats: false,
        };

        while let Some(flag) = raw.next() {
            match flag.as_str() {
                "--out" => out = Some(PathBuf::from(next_value(&mut raw, &flag)?)),
                "--frame" => {
                    args.frame = match next_value(&mut raw, &flag)?.as_str() {
                        "world" => TrackFrame::World,
                        "start" => TrackFrame::Start,
                        other => bail!("unknown frame {other:?}; expected world or start"),
                    };
                }
                "--bin-width" => {
                    args.params.bin_width_m = parse_f32(&next_value(&mut raw, &flag)?, &flag)?
                }
                "--v-max" => {
                    args.params.v_max_ms = parse_f32(&next_value(&mut raw, &flag)?, &flag)?
                }
                "--curvature-slowdown" => {
                    args.params.curvature_slowdown =
                        parse_f32(&next_value(&mut raw, &flag)?, &flag)?
                }
                "--smoothing-passes" => {
                    args.params.smoothing_passes =
                        parse_usize(&next_value(&mut raw, &flag)?, &flag)?
                }
                "--stats" => args.stats = true,
                "--help" | "-h" => {
                    print_help();
                    std::process::exit(0);
                }
                other => bail!("unknown argument {other:?}"),
            }
        }

        args.out = out.ok_or_else(|| anyhow!("--out <path.mcap> is required"))?;
        Ok(args)
    }
}

fn next_value(raw: &mut impl Iterator<Item = String>, flag: &str) -> Result<String> {
    raw.next().ok_or_else(|| anyhow!("{flag} requires a value"))
}

fn parse_f32(value: &str, flag: &str) -> Result<f32> {
    value
        .parse()
        .with_context(|| format!("invalid {flag} value {value:?}"))
}

fn parse_usize(value: &str, flag: &str) -> Result<usize> {
    value
        .parse()
        .with_context(|| format!("invalid {flag} value {value:?}"))
}

fn print_velocity_stats(raceline: &RaceLine, params: &RaceLineSolverParams) -> Result<()> {
    let curvatures: Vec<_> = raceline
        .points
        .iter()
        .map(|point| point.curvature)
        .collect();
    let mut segments = Vec::with_capacity(if raceline.closed {
        raceline.points.len()
    } else {
        raceline.points.len().saturating_sub(1)
    });
    for pair in raceline.points.windows(2) {
        segments.push(pair[0].p.dist(&pair[1].p));
    }
    if raceline.closed && !raceline.points.is_empty() {
        segments.push(
            raceline.points[raceline.points.len() - 1]
                .p
                .dist(&raceline.points[0].p),
        );
    }
    let profile = compute_velocity_profile(
        &curvatures,
        &segments,
        raceline.closed,
        &VelocityProfileParams {
            top_speed_ms: params.v_max_ms,
            lateral_accel_limit_ms2: params.velocity_lateral_accel_limit_ms2,
            accel_limit_ms2: params.velocity_accel_limit_ms2,
            brake_limit_ms2: params.velocity_brake_limit_ms2,
            curvature_epsilon_m_inv: params.velocity_curvature_epsilon_m_inv,
            closed_passes: params.velocity_closed_passes,
        },
    )
    .map_err(|error| anyhow!("velocity-profile validation failed: {error:?}"))?;
    let speed = stats(raceline.points.iter().map(|point| point.speed_ms));
    let accel = stats(raceline.points.iter().map(|point| point.accel_x_ms2));
    let corner_caps: Vec<_> = raceline
        .points
        .iter()
        .map(|point| corner_cap_unclipped(point.curvature, params))
        .collect();
    let finite_corner_caps = corner_caps.iter().copied().filter(|cap| cap.is_finite());
    let corner_cap_stats = stats(finite_corner_caps);
    let top_speed_limited_points = raceline
        .points
        .iter()
        .filter(|point| (point.speed_ms - params.v_max_ms).abs() <= 1.0e-4)
        .count();
    let tightest_index = raceline
        .points
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| {
            a.curvature
                .abs()
                .partial_cmp(&b.curvature.abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|(index, _)| index);
    let straight_index = raceline
        .points
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| {
            a.curvature
                .abs()
                .partial_cmp(&b.curvature.abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|(index, _)| index);
    let seam_adjacent_speed_delta_ms = if raceline.closed && raceline.points.len() >= 2 {
        (raceline.points[0].speed_ms - raceline.points[raceline.points.len() - 1].speed_ms).abs()
    } else {
        0.0
    };
    let seam_accel_x_ms2 = if raceline.closed && raceline.points.len() >= 2 {
        raceline.points[raceline.points.len() - 1].accel_x_ms2
    } else {
        0.0
    };
    let max_speed_mismatch = raceline
        .points
        .iter()
        .zip(&profile.speed_ms)
        .map(|(point, speed)| (point.speed_ms - speed).abs())
        .fold(0.0, f32::max);

    println!(
        "velocity_stats points={} speed_ms min={:.6} max={:.6} avg={:.6} accel_x_ms2 min={:.6} max={:.6} avg={:.6} corner_cap_unclipped_ms finite_min={:.6} finite_max={:.6} finite_avg={:.6} top_speed_limited_points={} closed_passes_run={} converged={} max_delta_ms={:.9} seam_adjacent_speed_delta_ms={:.6} seam_accel_x_ms2={:.6} max_speed_mismatch={:.9}",
        raceline.points.len(),
        speed.min,
        speed.max,
        speed.avg,
        accel.min,
        accel.max,
        accel.avg,
        corner_cap_stats.min,
        corner_cap_stats.max,
        corner_cap_stats.avg,
        top_speed_limited_points,
        profile.diagnostics.passes_run,
        profile.diagnostics.converged,
        profile.diagnostics.max_delta_ms,
        seam_adjacent_speed_delta_ms,
        seam_accel_x_ms2,
        max_speed_mismatch,
    );
    if let Some(index) = tightest_index {
        print_corner_cap_sample("tightest", raceline, &corner_caps, index);
    }
    if let Some(index) = straight_index {
        print_corner_cap_sample("straight", raceline, &corner_caps, index);
    }
    print_seam_sample(raceline, &corner_caps);
    Ok(())
}

#[derive(Clone, Copy, Debug)]
struct ScalarStats {
    min: f32,
    max: f32,
    avg: f32,
}

fn stats(values: impl Iterator<Item = f32>) -> ScalarStats {
    let mut min = f32::INFINITY;
    let mut max = f32::NEG_INFINITY;
    let mut sum = 0.0;
    let mut count = 0usize;
    for value in values {
        min = min.min(value);
        max = max.max(value);
        sum += value;
        count += 1;
    }
    ScalarStats {
        min,
        max,
        avg: if count == 0 { 0.0 } else { sum / count as f32 },
    }
}

fn corner_cap_unclipped(curvature_m_inv: f32, params: &RaceLineSolverParams) -> f32 {
    let curvature = curvature_m_inv.abs();
    if curvature <= params.velocity_curvature_epsilon_m_inv
        || params.velocity_lateral_accel_limit_ms2 <= 0.0
    {
        return f32::INFINITY;
    }
    (params.velocity_lateral_accel_limit_ms2 / curvature).sqrt()
}

fn print_corner_cap_sample(label: &str, raceline: &RaceLine, corner_caps: &[f32], index: usize) {
    let point = &raceline.points[index];
    println!(
        "corner_cap_sample kind={} index={} s_m={:.6} curvature_m_inv={:.6} corner_cap_unclipped_ms={:.6} speed_ms={:.6} accel_x_ms2={:.6}",
        label,
        index,
        point.s_m,
        point.curvature,
        corner_caps[index],
        point.speed_ms,
        point.accel_x_ms2,
    );
}

fn print_seam_sample(raceline: &RaceLine, corner_caps: &[f32]) {
    if !raceline.closed || raceline.points.len() < 2 {
        return;
    }
    let last_index = raceline.points.len() - 1;
    let first = &raceline.points[0];
    let last = &raceline.points[last_index];
    let segment_m = last.p.dist(&first.p);
    println!(
        "seam_sample last_index={} segment_m={:.6} speed_last_ms={:.6} speed_first_ms={:.6} speed_delta_ms={:.6} accel_last_to_first_ms2={:.6} curvature_last_m_inv={:.6} curvature_first_m_inv={:.6} corner_cap_last_ms={:.6} corner_cap_first_ms={:.6}",
        last_index,
        segment_m,
        last.speed_ms,
        first.speed_ms,
        first.speed_ms - last.speed_ms,
        last.accel_x_ms2,
        last.curvature,
        first.curvature,
        corner_caps[last_index],
        corner_caps[0],
    );
}

fn print_help() {
    eprintln!(
        "usage: car-raceline <world.json> --out <preview.mcap> [options]\n\n\
         options:\n\
           --out <path.mcap>              write Foxglove MCAP preview\n\
           --frame <start|world>          coordinate frame passed to the solver [default: start]\n\
           --bin-width <m>                legacy boundary fallback x-bin width\n\
           --v-max <m/s>                  velocity-profile top speed\n\
           --curvature-slowdown <k>       legacy fallback curvature slowdown\n\
           --smoothing-passes <n>         legacy boundary fallback smoothing passes\n\
           --stats                        print velocity-profile validation stats"
    );
}
