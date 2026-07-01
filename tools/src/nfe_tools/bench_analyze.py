from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Iterable


DEFAULT_KAPPA_SPEED_FLOOR_MS = 0.5
DEFAULT_WHEELBASE_M = 0.21
DEFAULT_CURRENT_YAW_RATE_GAIN_S = 0.08
DEFAULT_NOTE_KAPPA_GAIN_M = 0.08
DEFAULT_MOTOR_DEADBAND = 0.05
DEFAULT_MIN_FIT_SPEED_MS = 0.2
DEFAULT_MIN_COAST_DECEL_MS2 = 0.02
DEFAULT_COAST_SETTLE_S = 1.0


@dataclass(frozen=True)
class Sample:
    source: str
    script: str
    t_s: float
    phase: str
    steering_rad: float
    throttle: float
    speed_ms: float
    yaw_rate_rad_s: float
    ax_ms2: float
    ay_ms2: float
    kappa_est_m_inv: float
    front_obstacle_m: float
    abort_reason: str
    file: str


@dataclass
class SummaryInfo:
    file: str
    samples: int | None = None
    completed: bool | None = None
    aborted: bool | None = None
    max_speed_ms: float | None = None
    max_abs_yaw_rate_rad_s: float | None = None
    max_abs_ax_ms2: float | None = None
    final_speed_ms: float | None = None
    duration_s: float | None = None


@dataclass
class RunInfo:
    file: str
    source: str
    script: str
    samples: int
    valid_samples: int
    aborted: bool
    abort_reasons: list[str]
    incomplete: bool


@dataclass
class ScalarStats:
    count: int
    min: float | None = None
    max: float | None = None
    mean: float | None = None
    median: float | None = None


@dataclass
class CoastdownReport:
    samples: int = 0
    fit_samples: int = 0
    incomplete_run: bool = False
    rolling_accel_ms2: float | None = None
    drag_k: float | None = None
    r2: float | None = None
    speed_ms: ScalarStats | None = None
    decel_ms2: ScalarStats | None = None
    estimated_p_only_speed_error_ms: ScalarStats | None = None
    speed_feedback_gain_ms2_per_ms: float | None = None
    warnings: list[str] = field(default_factory=list)


@dataclass
class ThrottleReport:
    samples: int = 0
    fit_samples: int = 0
    incomplete_run: bool = False
    throttle: ScalarStats | None = None
    speed_ms: ScalarStats | None = None
    accel_ms2: ScalarStats | None = None
    motor_gain_ms2_estimate: ScalarStats | None = None
    steady_state_speed_ms: float | None = None
    steady_state_accel_ms2: float | None = None
    model_drag_k_used: float | None = None
    model_rolling_accel_used: float | None = None
    warnings: list[str] = field(default_factory=list)


@dataclass
class SpeedBinReport:
    speed_min_ms: float
    speed_max_ms: float
    samples: int
    speed_mean_ms: float
    kappa_residual_abs_mean_m_inv: float
    current_delta_correction_abs_mean_rad: float
    note_delta_correction_abs_mean_rad: float
    current_equiv_kappa_gain_mean_m: float


@dataclass
class SteeringReport:
    samples: int = 0
    used_samples: int = 0
    incomplete_run: bool = False
    excluded_low_speed_samples: int = 0
    kappa_speed_floor_ms: float = DEFAULT_KAPPA_SPEED_FLOOR_MS
    wheelbase_m: float = DEFAULT_WHEELBASE_M
    current_yaw_rate_gain_s: float = DEFAULT_CURRENT_YAW_RATE_GAIN_S
    note_kappa_gain_m: float = DEFAULT_NOTE_KAPPA_GAIN_M
    kappa_command_m_inv: ScalarStats | None = None
    kappa_est_m_inv: ScalarStats | None = None
    kappa_residual_m_inv: ScalarStats | None = None
    current_delta_rad: ScalarStats | None = None
    note_delta_rad: ScalarStats | None = None
    current_feedback_correction_rad: ScalarStats | None = None
    note_feedback_correction_rad: ScalarStats | None = None
    current_equivalent_kappa_gain_m: ScalarStats | None = None
    speed_bins: list[SpeedBinReport] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)


@dataclass
class BenchReport:
    runs: list[RunInfo]
    summaries: list[SummaryInfo]
    guardrails: dict[str, object]
    coastdown: CoastdownReport | None
    throttle_step: ThrottleReport | None
    steering_sweep: SteeringReport | None
    draft_toml: str
    inertia: dict[str, object]


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    samples = load_samples(args.csv)
    summaries = load_summaries(args.summary_json or [])
    runs = run_info(samples, summaries)
    aborted_scripts = {s.script for s in samples if s.abort_reason}
    valid = [s for s in samples if is_valid_sample(s)]

    coastdown = analyze_coastdown(valid, args, "coastdown" in aborted_scripts)
    throttle = analyze_throttle(valid, args, coastdown, "throttle_step" in aborted_scripts)
    steering = analyze_steering(valid, args, "steering_sweep" in aborted_scripts)
    draft_toml = build_draft_toml(coastdown, throttle)
    report = BenchReport(
        runs=runs,
        summaries=summaries,
        guardrails={
            "abort_rows_excluded": True,
            "incomplete_runs_flagged": True,
            "kappa_speed_floor_ms": args.kappa_speed_floor_ms,
            "low_speed_steering_samples_excluded": True,
            "inertia_empirical_fit_attempted": False,
        },
        coastdown=coastdown,
        throttle_step=throttle,
        steering_sweep=steering,
        draft_toml=draft_toml,
        inertia={
            "empirical_fit_attempted": False,
            "status": "blocked_without_tach_rpm_load_or_known_tractive_force",
            "note": "This utility does not fit rotational inertia from bench CSV samples.",
        },
    )

    print_report(report)
    if args.report:
        Path(args.report).write_text(json.dumps(to_jsonable(report), indent=2) + "\n")
    return 0


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze CSV/JSON outputs from car-bench open-loop dynamics tests."
    )
    parser.add_argument("--csv", nargs="+", required=True, help="One or more car-bench CSV files")
    parser.add_argument(
        "--summary-json",
        nargs="*",
        default=[],
        help="Optional car-bench JSON summary files. Used only to cross-check completion/abort metadata.",
    )
    parser.add_argument("--report", help="Write JSON report to this path")
    parser.add_argument(
        "--kappa-speed-floor-ms",
        type=float,
        default=DEFAULT_KAPPA_SPEED_FLOOR_MS,
        help=(
            "Speed floor/gate for steering curvature estimates. Samples below this speed are "
            "excluded from δ_note/δ_current comparison because yaw_rate/v is unreliable. "
            "Default matches the controller low-speed feedback activation threshold."
        ),
    )
    parser.add_argument("--wheelbase-m", type=float, default=DEFAULT_WHEELBASE_M)
    parser.add_argument(
        "--current-yaw-rate-gain-s",
        type=float,
        default=DEFAULT_CURRENT_YAW_RATE_GAIN_S,
        help="Current steering-controller yaw-rate feedback gain K_r [s].",
    )
    parser.add_argument(
        "--note-kappa-gain-m",
        type=float,
        default=DEFAULT_NOTE_KAPPA_GAIN_M,
        help=(
            "Candidate note-form curvature-error gain K_kappa [m]. Default equals the "
            "current yaw-rate gain evaluated at 1 m/s; pass an explicit value for decisions."
        ),
    )
    parser.add_argument("--motor-deadband", type=float, default=DEFAULT_MOTOR_DEADBAND)
    parser.add_argument(
        "--speed-feedback-gain-ms2-per-ms",
        type=float,
        default=4.0,
        help="Current longitudinal P gain used to estimate e_ss ~= disturbance_accel / gain.",
    )
    parser.add_argument(
        "--drag-k",
        type=float,
        default=None,
        help="Drag coefficient to use for throttle-step gain fitting. Defaults to coastdown fit if available.",
    )
    parser.add_argument(
        "--rolling-accel-ms2",
        type=float,
        default=None,
        help="Constant rolling/resistance accel to use for throttle-step gain fitting. Defaults to coastdown fit if available.",
    )
    parser.add_argument("--min-fit-speed-ms", type=float, default=DEFAULT_MIN_FIT_SPEED_MS)
    parser.add_argument(
        "--min-coast-decel-ms2",
        type=float,
        default=DEFAULT_MIN_COAST_DECEL_MS2,
        help="Minimum positive coastdown deceleration used in drag/rolling fit; filters motor-lag transients.",
    )
    parser.add_argument(
        "--coast-settle-s",
        type=float,
        default=DEFAULT_COAST_SETTLE_S,
        help="Seconds after the first coast sample to ignore before fitting drag/rolling decel.",
    )
    parser.add_argument(
        "--speed-bin-ms",
        type=float,
        default=0.5,
        help="Speed-bin width for steering comparison summaries.",
    )
    return parser.parse_args(argv)


def load_samples(paths: Iterable[str]) -> list[Sample]:
    samples: list[Sample] = []
    for path in paths:
        with Path(path).open(newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                samples.append(
                    Sample(
                        source=row.get("source", ""),
                        script=row.get("script", ""),
                        t_s=parse_float(row.get("t_s")),
                        phase=row.get("phase", ""),
                        steering_rad=parse_float(row.get("steering_rad")),
                        throttle=parse_float(row.get("throttle")),
                        speed_ms=parse_float(row.get("speed_ms")),
                        yaw_rate_rad_s=parse_float(row.get("yaw_rate_rad_s")),
                        ax_ms2=parse_float(row.get("ax_ms2")),
                        ay_ms2=parse_float(row.get("ay_ms2")),
                        kappa_est_m_inv=parse_float(row.get("kappa_est_m_inv")),
                        front_obstacle_m=parse_float(row.get("front_obstacle_m")),
                        abort_reason=row.get("abort_reason", "").strip(),
                        file=str(path),
                    )
                )
    if not samples:
        raise SystemExit("no samples loaded")
    return samples


def parse_float(value: str | None) -> float:
    if value is None or value == "":
        return math.nan
    if value == "inf":
        return math.inf
    if value == "-inf":
        return -math.inf
    return float(value)


def is_valid_sample(sample: Sample) -> bool:
    if sample.abort_reason:
        return False
    if sample.phase in {"abort", "done"}:
        return False
    return all(
        math.isfinite(value)
        for value in [
            sample.t_s,
            sample.steering_rad,
            sample.throttle,
            sample.speed_ms,
            sample.yaw_rate_rad_s,
            sample.ax_ms2,
            sample.ay_ms2,
        ]
    )


def run_info(samples: list[Sample]) -> list[RunInfo]:
    by_file: dict[str, list[Sample]] = {}
    for sample in samples:
        by_file.setdefault(sample.file, []).append(sample)
    runs: list[RunInfo] = []
    for file, rows in sorted(by_file.items()):
        abort_reasons = sorted({r.abort_reason for r in rows if r.abort_reason})
        phases = {r.phase for r in rows}
        scripts = {r.script for r in rows}
        sources = {r.source for r in rows}
        runs.append(
            RunInfo(
                file=file,
                source=next(iter(sources)) if len(sources) == 1 else "mixed",
                script=next(iter(scripts)) if len(scripts) == 1 else "mixed",
                samples=len(rows),
                valid_samples=sum(1 for r in rows if is_valid_sample(r)),
                aborted=bool(abort_reasons),
                abort_reasons=abort_reasons,
                incomplete=bool(abort_reasons) or "done" in phases or "abort" in phases,
            )
        )
    return runs


def analyze_coastdown(
    samples: list[Sample], args: argparse.Namespace, incomplete_run: bool
) -> CoastdownReport | None:
    coast_candidates = [s for s in samples if s.script == "coastdown" and s.phase == "coast"]
    coast_start_s = min((s.t_s for s in coast_candidates), default=0.0)
    rows = [
        s
        for s in coast_candidates
        if s.t_s >= coast_start_s + args.coast_settle_s
        and s.speed_ms >= args.min_fit_speed_ms
        and -s.ax_ms2 >= args.min_coast_decel_ms2
    ]
    if not rows:
        return None
    report = CoastdownReport(
        samples=len([s for s in samples if s.script == "coastdown"]),
        fit_samples=len(rows),
        incomplete_run=incomplete_run,
    )
    if incomplete_run:
        report.warnings.append("run contains abort rows; fit is diagnostic only and not config-ready")
    speeds = [s.speed_ms for s in rows]
    decels = [-s.ax_ms2 for s in rows]
    report.speed_ms = stats(speeds)
    report.decel_ms2 = stats(decels)
    report.speed_feedback_gain_ms2_per_ms = args.speed_feedback_gain_ms2_per_ms
    if args.speed_feedback_gain_ms2_per_ms > 1.0e-6:
        report.estimated_p_only_speed_error_ms = stats(
            decel / args.speed_feedback_gain_ms2_per_ms for decel in decels
        )
    fit = linear_fit([s.speed_ms * s.speed_ms for s in rows], [s.ax_ms2 for s in rows])
    if fit is None:
        report.warnings.append("not enough coastdown samples for ax = -rolling - drag_k*v^2 fit")
        return report
    intercept, slope, r2 = fit
    report.rolling_accel_ms2 = -intercept
    report.drag_k = -slope
    report.r2 = r2
    if report.drag_k is not None and report.drag_k < 0.0:
        report.warnings.append("fit produced negative drag_k; check IMU bias/sign or coast phase selection")
    if report.rolling_accel_ms2 is not None and report.rolling_accel_ms2 < -0.2:
        report.warnings.append("fit produced negative rolling resistance; check data quality")
    return report


def analyze_throttle(
    samples: list[Sample],
    args: argparse.Namespace,
    coastdown: CoastdownReport | None,
    incomplete_run: bool,
) -> ThrottleReport | None:
    rows = [s for s in samples if s.script == "throttle_step" and s.phase == "throttle_step"]
    if not rows:
        return None
    drag_k = args.drag_k
    rolling = args.rolling_accel_ms2
    if drag_k is None and coastdown and coastdown.drag_k is not None:
        drag_k = coastdown.drag_k
    if rolling is None and coastdown and coastdown.rolling_accel_ms2 is not None:
        rolling = max(0.0, coastdown.rolling_accel_ms2)
    if drag_k is None:
        drag_k = 0.0
    if rolling is None:
        rolling = 0.0

    gains = []
    fit_rows = []
    for s in rows:
        eff = effective_throttle(s.throttle, args.motor_deadband)
        if eff <= 1.0e-6:
            continue
        # a_measured ≈ motor_gain*eff - rolling - drag_k*v²
        gains.append((s.ax_ms2 + rolling + drag_k * s.speed_ms * s.speed_ms) / eff)
        fit_rows.append(s)

    tail = rows[int(len(rows) * 0.8) :] if rows else []
    report = ThrottleReport(
        samples=len(rows),
        fit_samples=len(fit_rows),
        incomplete_run=incomplete_run,
        throttle=stats([s.throttle for s in rows]),
        speed_ms=stats([s.speed_ms for s in rows]),
        accel_ms2=stats([s.ax_ms2 for s in rows]),
        motor_gain_ms2_estimate=stats(gains) if gains else None,
        steady_state_speed_ms=statistics.fmean(s.speed_ms for s in tail) if tail else None,
        steady_state_accel_ms2=statistics.fmean(s.ax_ms2 for s in tail) if tail else None,
        model_drag_k_used=drag_k,
        model_rolling_accel_used=rolling,
    )
    if incomplete_run:
        report.steady_state_speed_ms = None
        report.steady_state_accel_ms2 = None
        report.warnings.append("run contains abort rows; steady-state/config metrics are not valid")
    if not gains:
        report.warnings.append("no positive effective-throttle samples after deadband")
    return report


def analyze_steering(
    samples: list[Sample], args: argparse.Namespace, incomplete_run: bool
) -> SteeringReport | None:
    rows = [s for s in samples if s.script == "steering_sweep" and s.phase.startswith("steer_")]
    if not rows:
        return None
    used = [s for s in rows if abs(s.speed_ms) >= args.kappa_speed_floor_ms]
    report = SteeringReport(
        samples=len(rows),
        used_samples=len(used),
        incomplete_run=incomplete_run,
        excluded_low_speed_samples=len(rows) - len(used),
        kappa_speed_floor_ms=args.kappa_speed_floor_ms,
        wheelbase_m=args.wheelbase_m,
        current_yaw_rate_gain_s=args.current_yaw_rate_gain_s,
        note_kappa_gain_m=args.note_kappa_gain_m,
    )
    if incomplete_run:
        report.warnings.append("run contains abort rows; steering comparison is diagnostic only")
    if not used:
        report.warnings.append("no steering samples above kappa-speed floor")
        return report

    k_cmd = []
    k_est = []
    k_resid = []
    d_current = []
    d_note = []
    fb_current = []
    fb_note = []
    equiv_gain = []
    for s in used:
        kappa_t = math.tan(s.steering_rad) / args.wheelbase_m
        kappa = s.yaw_rate_rad_s / s.speed_ms
        residual = kappa_t - kappa
        current = math.atan(args.wheelbase_m * kappa_t) + args.current_yaw_rate_gain_s * (
            s.speed_ms * kappa_t - s.yaw_rate_rad_s
        )
        note = args.wheelbase_m * kappa_t + args.note_kappa_gain_m * residual
        k_cmd.append(kappa_t)
        k_est.append(kappa)
        k_resid.append(residual)
        d_current.append(current)
        d_note.append(note)
        fb_current.append(current - math.atan(args.wheelbase_m * kappa_t))
        fb_note.append(note - args.wheelbase_m * kappa_t)
        equiv_gain.append(args.current_yaw_rate_gain_s * s.speed_ms)

    report.kappa_command_m_inv = stats(k_cmd)
    report.kappa_est_m_inv = stats(k_est)
    report.kappa_residual_m_inv = stats(k_resid)
    report.current_delta_rad = stats(d_current)
    report.note_delta_rad = stats(d_note)
    report.current_feedback_correction_rad = stats_abs(fb_current)
    report.note_feedback_correction_rad = stats_abs(fb_note)
    report.current_equivalent_kappa_gain_m = stats(equiv_gain)
    report.speed_bins = steering_bins(used, args)
    if report.excluded_low_speed_samples:
        report.warnings.append(
            f"excluded {report.excluded_low_speed_samples} samples below {args.kappa_speed_floor_ms:.3f} m/s"
        )
    return report


def steering_bins(rows: list[Sample], args: argparse.Namespace) -> list[SpeedBinReport]:
    if not rows:
        return []
    width = max(args.speed_bin_ms, 1.0e-3)
    min_speed = min(s.speed_ms for s in rows)
    bins: dict[int, list[Sample]] = {}
    for row in rows:
        bins.setdefault(int((row.speed_ms - min_speed) / width), []).append(row)
    out = []
    for index, group in sorted(bins.items()):
        residuals = []
        current_corr = []
        note_corr = []
        equiv = []
        for s in group:
            kappa_t = math.tan(s.steering_rad) / args.wheelbase_m
            kappa = s.yaw_rate_rad_s / s.speed_ms
            residual = kappa_t - kappa
            residuals.append(abs(residual))
            current_corr.append(abs(args.current_yaw_rate_gain_s * (s.speed_ms * residual)))
            note_corr.append(abs(args.note_kappa_gain_m * residual))
            equiv.append(args.current_yaw_rate_gain_s * s.speed_ms)
        out.append(
            SpeedBinReport(
                speed_min_ms=min_speed + index * width,
                speed_max_ms=min_speed + (index + 1) * width,
                samples=len(group),
                speed_mean_ms=statistics.fmean(s.speed_ms for s in group),
                kappa_residual_abs_mean_m_inv=statistics.fmean(residuals),
                current_delta_correction_abs_mean_rad=statistics.fmean(current_corr),
                note_delta_correction_abs_mean_rad=statistics.fmean(note_corr),
                current_equiv_kappa_gain_mean_m=statistics.fmean(equiv),
            )
        )
    return out


def effective_throttle(throttle: float, deadband: float) -> float:
    if abs(throttle) <= deadband:
        return 0.0
    return math.copysign((abs(throttle) - deadband) / (1.0 - deadband), throttle)


def linear_fit(xs: list[float], ys: list[float]) -> tuple[float, float, float] | None:
    if len(xs) < 2 or len(xs) != len(ys):
        return None
    x_bar = statistics.fmean(xs)
    y_bar = statistics.fmean(ys)
    sxx = sum((x - x_bar) ** 2 for x in xs)
    if sxx <= 1.0e-12:
        return None
    sxy = sum((x - x_bar) * (y - y_bar) for x, y in zip(xs, ys, strict=True))
    slope = sxy / sxx
    intercept = y_bar - slope * x_bar
    ss_tot = sum((y - y_bar) ** 2 for y in ys)
    ss_res = sum((y - (intercept + slope * x)) ** 2 for x, y in zip(xs, ys, strict=True))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1.0e-12 else 1.0
    return intercept, slope, r2


def stats(values: Iterable[float]) -> ScalarStats:
    vals = [v for v in values if math.isfinite(v)]
    if not vals:
        return ScalarStats(count=0)
    return ScalarStats(
        count=len(vals),
        min=min(vals),
        max=max(vals),
        mean=statistics.fmean(vals),
        median=statistics.median(vals),
    )


def stats_abs(values: Iterable[float]) -> ScalarStats:
    return stats(abs(v) for v in values)


def build_draft_toml(coastdown: CoastdownReport | None, throttle: ThrottleReport | None) -> str:
    lines = [
        "# Draft only. Review against raw data before pasting into nfe.toml.",
        "# This tool never overwrites tuned constants.",
    ]
    if (
        coastdown
        and not coastdown.incomplete_run
        and coastdown.drag_k is not None
        and coastdown.drag_k >= 0.0
    ):
        lines += [
            "[algo.raceline_controller.longitudinal]",
            f"drag_k = {coastdown.drag_k:.6g}",
        ]
    if (
        throttle
        and not throttle.incomplete_run
        and throttle.motor_gain_ms2_estimate
        and throttle.motor_gain_ms2_estimate.median
    ):
        if "[algo.raceline_controller.longitudinal]" not in lines:
            lines.append("[algo.raceline_controller.longitudinal]")
        lines.append(f"motor_gain_ms2 = {throttle.motor_gain_ms2_estimate.median:.6g}")
    if len(lines) == 2:
        lines.append("# No draft values available from supplied samples.")
    return "\n".join(lines)


def print_report(report: BenchReport) -> None:
    print("bench analysis")
    print("==============")
    for run in report.runs:
        suffix = " ABORTED" if run.aborted else ""
        print(
            f"run {run.file}: {run.source}/{run.script} samples={run.samples} valid={run.valid_samples}{suffix}"
        )
        if run.abort_reasons:
            print(f"  abort_reasons={','.join(run.abort_reasons)}")
    if report.coastdown:
        c = report.coastdown
        print("\ncoastdown")
        print(f"  fit_samples={c.fit_samples}")
        print(f"  rolling_accel_ms2={fmt(c.rolling_accel_ms2)} drag_k={fmt(c.drag_k)} r2={fmt(c.r2)}")
        print_stats("  speed_ms", c.speed_ms)
        print_stats("  decel_ms2", c.decel_ms2)
        print_stats("  estimated_P_only_speed_error_ms", c.estimated_p_only_speed_error_ms)
        for warning in c.warnings:
            print(f"  warning: {warning}")
    if report.throttle_step:
        t = report.throttle_step
        print("\nthrottle_step")
        print(f"  fit_samples={t.fit_samples} drag_k_used={fmt(t.model_drag_k_used)} rolling_used={fmt(t.model_rolling_accel_used)}")
        print_stats("  throttle", t.throttle)
        print_stats("  speed_ms", t.speed_ms)
        print_stats("  accel_ms2", t.accel_ms2)
        print_stats("  motor_gain_ms2_estimate", t.motor_gain_ms2_estimate)
        print(f"  tail_speed_ms={fmt(t.steady_state_speed_ms)} tail_accel_ms2={fmt(t.steady_state_accel_ms2)}")
        for warning in t.warnings:
            print(f"  warning: {warning}")
    if report.steering_sweep:
        s = report.steering_sweep
        print("\nsteering_sweep")
        print(
            f"  used={s.used_samples}/{s.samples} kappa_speed_floor_ms={s.kappa_speed_floor_ms:.3f} "
            f"wheelbase_m={s.wheelbase_m:.3f} current_Kr_s={s.current_yaw_rate_gain_s:.3f} "
            f"note_Kkappa_m={s.note_kappa_gain_m:.3f}"
        )
        print_stats("  kappa_cmd_m_inv", s.kappa_command_m_inv)
        print_stats("  kappa_est_m_inv", s.kappa_est_m_inv)
        print_stats("  kappa_residual_m_inv", s.kappa_residual_m_inv)
        print_stats("  current_abs_feedback_rad", s.current_feedback_correction_rad)
        print_stats("  note_abs_feedback_rad", s.note_feedback_correction_rad)
        print_stats("  current_equiv_Kkappa_m", s.current_equivalent_kappa_gain_m)
        for b in s.speed_bins:
            print(
                "  bin "
                f"{b.speed_min_ms:.2f}-{b.speed_max_ms:.2f}m/s n={b.samples} "
                f"resid_abs={b.kappa_residual_abs_mean_m_inv:.4f} "
                f"current_corr={b.current_delta_correction_abs_mean_rad:.4f} "
                f"note_corr={b.note_delta_correction_abs_mean_rad:.4f} "
                f"current_equiv_K={b.current_equiv_kappa_gain_mean_m:.4f}"
            )
        for warning in s.warnings:
            print(f"  warning: {warning}")
    print("\ninertia")
    print("  empirical fit: not attempted (blocked without tach/RPM/load/known tractive force)")
    print("\ndraft TOML")
    print(report.draft_toml)


def print_stats(label: str, value: ScalarStats | None) -> None:
    if value is None or value.count == 0:
        print(f"{label}: n=0")
        return
    print(
        f"{label}: n={value.count} min={fmt(value.min)} max={fmt(value.max)} "
        f"mean={fmt(value.mean)} median={fmt(value.median)}"
    )


def fmt(value: float | None) -> str:
    if value is None:
        return "n/a"
    return f"{value:.6g}"


def to_jsonable(value: object) -> object:
    if hasattr(value, "__dataclass_fields__"):
        return {k: to_jsonable(v) for k, v in asdict(value).items()}
    if isinstance(value, list):
        return [to_jsonable(v) for v in value]
    if isinstance(value, dict):
        return {k: to_jsonable(v) for k, v in value.items()}
    return value


if __name__ == "__main__":
    raise SystemExit(main())
