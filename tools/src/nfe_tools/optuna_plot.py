from __future__ import annotations

import argparse
import warnings
from pathlib import Path
from typing import Callable

import optuna
import pandas as pd
import plotly.express as px
from optuna.exceptions import ExperimentalWarning
from optuna.importance import PedAnovaImportanceEvaluator
from optuna.visualization import (
    plot_optimization_history,
    plot_parallel_coordinate,
    plot_param_importances,
    plot_slice,
)
from plotly.graph_objects import Figure
from rich.console import Console

console = Console()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot NFE Optuna tuning runs")
    parser.add_argument("--storage", default="sqlite:///runs/tuning/nfe-optuna.db")
    parser.add_argument("--study", default="nfe-tpe")
    parser.add_argument("--out-dir", default="runs/tuning/plots")
    return parser.parse_args()


def write_plot(out_dir: Path, name: str, build: Callable[[], Figure]) -> None:
    path = out_dir / name
    try:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", ExperimentalWarning)
            figure = build()
        figure.write_html(path)
        console.print(f"wrote {path}")
    except (ImportError, RuntimeError, ValueError) as error:
        console.print(f"[yellow]skipped {name}: {error}[/yellow]")


def completed_trials(df: pd.DataFrame) -> pd.DataFrame:
    if "state" not in df.columns:
        return df.iloc[0:0]
    return df[df["state"].astype(str).str.endswith("COMPLETE")].copy()


def write_metric_plots(out_dir: Path, df: pd.DataFrame) -> None:
    complete = completed_trials(df)
    if complete.empty:
        console.print("[yellow]skipped metric plots: no completed trials[/yellow]")
        return

    metric_columns = [
        "user_attrs_progress_ratio",
        "user_attrs_avg_speed_ms",
        "user_attrs_max_speed_ms",
        "user_attrs_lateral_rms_m",
        "user_attrs_heading_rms_rad",
        "user_attrs_steering_rate_rms",
        "user_attrs_throttle_rate_rms",
        "user_attrs_unavailable_fraction",
    ]
    available = [column for column in metric_columns if column in complete.columns]
    if not available:
        console.print("[yellow]skipped metric plots: no rich score user_attrs found[/yellow]")
        return

    metric_df = complete[["number", *available]].copy()
    for column in available:
        metric_df[column] = pd.to_numeric(metric_df[column], errors="coerce")
    long = metric_df.melt(id_vars="number", value_vars=available, var_name="metric", value_name="value")
    long = long.dropna(subset=["value"])
    long["metric"] = long["metric"].str.removeprefix("user_attrs_")
    if long.empty:
        console.print("[yellow]skipped metric plots: metrics are empty[/yellow]")
        return

    figure = px.line(
        long,
        x="number",
        y="value",
        color="metric",
        markers=True,
        title="Candidate metrics over trials",
    )
    figure.write_html(out_dir / "candidate_metrics.html")
    console.print(f"wrote {out_dir / 'candidate_metrics.html'}")

    if "user_attrs_status" in complete.columns:
        counts = complete["user_attrs_status"].fillna("unknown").value_counts().reset_index()
        counts.columns = ["status", "count"]
        figure = px.bar(counts, x="status", y="count", title="Trial status counts")
        figure.write_html(out_dir / "status_counts.html")
        console.print(f"wrote {out_dir / 'status_counts.html'}")


def main() -> None:
    args = parse_args()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    study = optuna.load_study(study_name=args.study, storage=args.storage)
    df = study.trials_dataframe()
    csv_path = out_dir / "trials.csv"
    df.to_csv(csv_path, index=False)
    console.print(f"wrote {csv_path}")

    write_plot(out_dir, "optimization_history.html", lambda: plot_optimization_history(study))
    write_plot(
        out_dir,
        "param_importances.html",
        lambda: plot_param_importances(study, evaluator=PedAnovaImportanceEvaluator()),
    )
    write_plot(out_dir, "parallel_coordinate.html", lambda: plot_parallel_coordinate(study))
    write_plot(out_dir, "slice.html", lambda: plot_slice(study))
    write_metric_plots(out_dir, df)


if __name__ == "__main__":
    main()
