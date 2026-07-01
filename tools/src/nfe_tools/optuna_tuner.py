from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Any

import optuna
from rich.console import Console

console = Console()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Tune NFE runtime params with Optuna TPE"
    )
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
    parser.add_argument("--model", default="dynamic")
    parser.add_argument("--model-params")
    parser.add_argument(
        "--trial-dir",
        help=(
            "Persist per-trial candidate, score, stdout, stderr, "
            "and runtime config files"
        ),
    )
    parser.add_argument(
        "--recover-trial",
        type=int,
        help="Write --out from an existing Optuna trial number and exit",
    )
    args = parser.parse_args()
    if args.param_prefix is None:
        args.param_prefix = ["algo.apex", "algo.reactive"]
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


def trial_dir(args: argparse.Namespace, trial_number: int) -> Path | None:
    if not args.trial_dir:
        return None
    return Path(args.trial_dir) / f"trial_{trial_number:06d}"


def evaluate_candidate(
    args: argparse.Namespace,
    params: dict[str, float | int],
    *,
    output_config: Path | None = None,
    trial_number: int | None = None,
) -> dict[str, Any]:
    persisted_trial_dir = (
        trial_dir(args, trial_number) if trial_number is not None else None
    )
    if persisted_trial_dir is not None:
        persisted_trial_dir.mkdir(parents=True, exist_ok=True)
        return evaluate_candidate_in_dir(
            args, params, persisted_trial_dir, output_config=output_config
        )

    with tempfile.TemporaryDirectory() as tmpdir:
        return evaluate_candidate_in_dir(
            args, params, Path(tmpdir), output_config=output_config
        )


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


def complete_trials(study: optuna.Study) -> list[optuna.trial.FrozenTrial]:
    return [
        trial
        for trial in study.trials
        if trial.state == optuna.trial.TrialState.COMPLETE and trial.value is not None
    ]


def best_complete_trial(study: optuna.Study) -> optuna.trial.FrozenTrial:
    completed = complete_trials(study)
    if not completed:
        raise SystemExit(
            "No completed Optuna trials; check car-tune stderr for pruned failures"
        )
    return min(completed, key=lambda trial: float(trial.value))


def trial_runtime_config_path(args: argparse.Namespace, trial_number: int) -> Path | None:
    persisted_trial_dir = trial_dir(args, trial_number)
    if persisted_trial_dir is None:
        return None
    return persisted_trial_dir / "runtime_config.json"


def write_trial_config(
    args: argparse.Namespace,
    trial: optuna.trial.FrozenTrial,
    out: Path,
    *,
    score_out: Path | None = None,
) -> None:
    cached = trial_runtime_config_path(args, trial.number)
    if cached is not None and cached.exists():
        out.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(cached, out)
        if score_out is not None:
            cached_score = cached.parent / "score.json"
            if cached_score.exists():
                shutil.copyfile(cached_score, score_out)
        return

    with tempfile.TemporaryDirectory() as tmpdir:
        work_dir = Path(tmpdir)
        evaluate_candidate_in_dir(args, trial.params, work_dir, output_config=out)
        if score_out is not None:
            shutil.copyfile(work_dir / "score.json", score_out)


def write_best_config(args: argparse.Namespace, study: optuna.Study, out: Path) -> None:
    best = best_complete_trial(study)
    write_trial_config(args, best, out)
    console.print(f"Best trial: {best.number}")
    console.print(f"Best score: {float(best.value):.6f}")
    console.print(f"Best config written to {out}")


def main() -> None:
    args = parse_args()
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

    best_out = Path(args.out)
    if args.recover_trial is not None:
        matches = [trial for trial in study.trials if trial.number == args.recover_trial]
        if not matches:
            raise SystemExit(f"Trial {args.recover_trial} not found in study {args.study}")
        trial = matches[0]
        if trial.state != optuna.trial.TrialState.COMPLETE or trial.value is None:
            raise SystemExit(f"Trial {args.recover_trial} is not complete")
        score_out = None
        persisted_trial_dir = trial_dir(args, trial.number)
        if persisted_trial_dir is not None:
            persisted_trial_dir.mkdir(parents=True, exist_ok=True)
            score_out = persisted_trial_dir / "recovered_score.json"
        write_trial_config(args, trial, best_out, score_out=score_out)
        console.print(f"Recovered trial: {trial.number}")
        console.print(f"Recovered score: {float(trial.value):.6f}")
        console.print(f"Recovered config written to {best_out}")
        return

    space = load_search_space(args)
    if not space:
        raise SystemExit("car-tune returned an empty search space")

    console.print(f"Loaded {len(space)} parameters from car-tune")

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
            stability_product = float(
                params["algo.apex.apex_switch_threshold_rad"]
            ) * float(params["algo.apex.apex_switch_hysteresis_factor"])
            trial.set_user_attr("apex_stability_product", stability_product)
        output_config = trial_runtime_config_path(args, trial.number)
        score = evaluate_candidate(
            args,
            params,
            output_config=output_config,
            trial_number=trial.number,
        )
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

    if complete_trials(study):
        write_best_config(args, study, best_out)

    def update_best_callback(
        study: optuna.Study, trial: optuna.trial.FrozenTrial
    ) -> None:
        if trial.state != optuna.trial.TrialState.COMPLETE or trial.value is None:
            return
        best = best_complete_trial(study)
        if trial.number == best.number:
            write_best_config(args, study, best_out)

    study.optimize(objective, n_trials=args.trials, callbacks=[update_best_callback])
    write_best_config(args, study, best_out)


if __name__ == "__main__":
    main()
