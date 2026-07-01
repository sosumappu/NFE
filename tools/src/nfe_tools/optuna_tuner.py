from __future__ import annotations

import argparse
import json
import subprocess
import tempfile
from pathlib import Path
from typing import Any

import optuna
from rich.console import Console

console = Console()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Tune NFE runtime params with Optuna TPE")
    parser.add_argument("--car-tune", default="car-tune")
    parser.add_argument("--sim", required=True)
    parser.add_argument("--config")
    parser.add_argument("--out", required=True)
    parser.add_argument("--trials", type=int, default=500)
    parser.add_argument("--storage", default="sqlite:///nfe-optuna.db")
    parser.add_argument("--study", default="nfe-tpe")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--param-prefix", action="append")
    parser.add_argument("--episode-s", type=float, default=90.0)
    parser.add_argument("--target-laps", type=int, default=3)
    parser.add_argument("--target-speed", type=float, default=3.0)
    parser.add_argument("--min-avg-speed", type=float, default=1.0)
    parser.add_argument("--eval-seeds", type=int, default=3)
    parser.add_argument("--robustness-weight", type=float, default=0.0)
    parser.add_argument("--sim-seed", type=int)
    parser.add_argument("--model", default="kinematic")
    parser.add_argument("--model-params")
    parser.add_argument("--trial-dir", help="Persist per-trial candidate, score, stdout, and stderr files")
    args = parser.parse_args()
    if args.param_prefix is None:
        args.param_prefix = ["algo.apex"]
    return args


def base_car_tune_args(args: argparse.Namespace) -> list[str]:
    cmd = [
        "--sim",
        args.sim,
        "--episode-s",
        str(args.episode_s),
        "--target-laps",
        str(args.target_laps),
        "--target-speed",
        str(args.target_speed),
        "--min-avg-speed",
        str(args.min_avg_speed),
        "--eval-seeds",
        str(args.eval_seeds),
        "--robustness-weight",
        str(args.robustness_weight),
        "--model",
        args.model,
    ]
    if args.config:
        cmd += ["--config", args.config]
    if args.sim_seed is not None:
        cmd += ["--sim-seed", str(args.sim_seed)]
    if args.model_params:
        cmd += ["--model-params", args.model_params]
    return cmd


def run_checked(cmd: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, text=True, capture_output=True, check=True)


def load_search_space(args: argparse.Namespace) -> list[dict[str, Any]]:
    with tempfile.NamedTemporaryFile(suffix=".json") as tmp:
        cmd = [args.car_tune, "--dump-search-space", "--out", tmp.name]
        if args.config:
            cmd += ["--config", args.config]
        for prefix in args.param_prefix:
            cmd += ["--param-prefix", prefix]
        run_checked(cmd)
        return json.loads(Path(tmp.name).read_text())


def suggest_value(trial: optuna.Trial, entry: dict[str, Any]) -> float | int:
    name = str(entry["name"])
    low = float(entry["low"])
    high = float(entry["high"])
    log = entry.get("scale") == "log"
    if entry["kind"] == "int":
        return trial.suggest_int(name, int(low), int(high), log=log)
    return trial.suggest_float(name, low, high, log=log)


def write_candidate(path: Path, params: dict[str, float | int]) -> None:
    path.write_text(json.dumps({"params": params}, indent=2))


def evaluate_candidate(
    args: argparse.Namespace,
    params: dict[str, float | int],
    *,
    output_config: Path | None = None,
    trial_number: int | None = None,
) -> dict[str, Any]:
    if args.trial_dir and trial_number is not None:
        trial_dir = Path(args.trial_dir) / f"trial_{trial_number:06d}"
        trial_dir.mkdir(parents=True, exist_ok=True)
        return evaluate_candidate_in_dir(args, params, trial_dir, output_config=output_config)

    with tempfile.TemporaryDirectory() as tmpdir:
        return evaluate_candidate_in_dir(args, params, Path(tmpdir), output_config=output_config)


def evaluate_candidate_in_dir(
    args: argparse.Namespace,
    params: dict[str, float | int],
    work_dir: Path,
    *,
    output_config: Path | None = None,
) -> dict[str, Any]:
    candidate_path = work_dir / "candidate.json"
    score_path = work_dir / "score.json"
    stdout_path = work_dir / "stdout.log"
    stderr_path = work_dir / "stderr.log"
    write_candidate(candidate_path, params)
    cmd = [
        args.car_tune,
        "--candidate",
        str(candidate_path),
        "--score-out",
        str(score_path),
    ] + base_car_tune_args(args)
    if output_config is not None:
        output_config.parent.mkdir(parents=True, exist_ok=True)
        cmd += ["--out", str(output_config)]

    completed = subprocess.run(cmd, text=True, capture_output=True)
    stdout_path.write_text(completed.stdout)
    stderr_path.write_text(completed.stderr)
    if completed.returncode != 0:
        message = completed.stderr.strip() or completed.stdout.strip()
        console.print(f"[red]car-tune failed[/red]: {message}")
        raise optuna.TrialPruned(message or "car-tune candidate evaluation failed")
    return json.loads(score_path.read_text())


def main() -> None:
    args = parse_args()
    space = load_search_space(args)
    if not space:
        raise SystemExit("car-tune returned an empty search space")

    console.print(f"Loaded {len(space)} parameters from car-tune")
    sampler = optuna.samplers.TPESampler(
        seed=args.seed,
        n_startup_trials=min(30, max(1, args.trials // 10)),
    )
    study = optuna.create_study(
        direction="minimize",
        sampler=sampler,
        storage=args.storage,
        study_name=args.study,
        load_if_exists=True,
    )

    if not study.trials:
        baseline_params = {
            entry["name"]: entry.get("current", entry.get("default"))
            for entry in space
            if entry.get("current", entry.get("default")) is not None
        }
        study.enqueue_trial(baseline_params, user_attrs={"seeded_baseline": True})

    def objective(trial: optuna.Trial) -> float:
        params = {entry["name"]: suggest_value(trial, entry) for entry in space}
        if (
            "algo.apex.apex_switch_threshold_rad" in params
            and "algo.apex.apex_switch_hysteresis_factor" in params
        ):
            stability_product = (
                float(params["algo.apex.apex_switch_threshold_rad"])
                * float(params["algo.apex.apex_switch_hysteresis_factor"])
            )
            trial.set_user_attr("apex_stability_product", stability_product)
        score = evaluate_candidate(args, params, trial_number=trial.number)
        for key, value in score.items():
            trial.set_user_attr(key, value)
        console.print(
            "trial={trial} status={status} score={score:.3f} progress={progress:.1%} "
            "laps={laps} crashed={crashed} avg={avg:.2f}m/s".format(
                trial=trial.number,
                status=score.get("status", "unknown"),
                score=float(score["score"]),
                progress=float(score.get("progress_ratio", 0.0)),
                laps=score.get("completed_laps", 0),
                crashed=score.get("crashed", False),
                avg=float(score.get("avg_speed_ms", 0.0)),
            )
        )
        return float(score["score"])

    study.optimize(objective, n_trials=args.trials)
    completed = [trial for trial in study.trials if trial.state == optuna.trial.TrialState.COMPLETE]
    if not completed:
        raise SystemExit("No completed Optuna trials; check car-tune stderr for pruned failures")

    best_out = Path(args.out)
    evaluate_candidate(args, study.best_trial.params, output_config=best_out)
    console.print(f"Best score: {study.best_value:.6f}")
    console.print(f"Best config written to {best_out}")


if __name__ == "__main__":
    main()
