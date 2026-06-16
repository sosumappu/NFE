#!/usr/bin/env python3
"""
tools/identify.py — Vehicle parameter identification from MCAP recordings

Reads the metrics CSV written by `cargo run -- --csv-out metrics.csv` and
performs ordinary least-squares to recover the dynamic bicycle model constants
that best explain the observed lateral acceleration and yaw rate.

Usage
─────
  # 1. Record a session (several laps, varied speed + steering)
  cargo run --release -- --record session.mcap --csv-out metrics.csv

  # 2. Run identification
  python3 tools/identify.py metrics.csv --out model_params.json

  # 3. Use the identified model in simulation
  cargo run --release -- --sim track.json \
      --model identified --model-params model_params.json

What is identified
──────────────────
  motor_gain   throttle [−1,+1] → longitudinal acceleration [m/s²]
  drag_k       quadratic drag coefficient [m/s²/(m/s)²]
  cf           front cornering stiffness [N/rad]
  cr           rear  cornering stiffness [N/rad]

Fixed (measure physically):
  wheelbase    0.33 m  (front axle to rear axle — use calipers)
  lf           0.165 m (CoM to front axle — weigh front/rear separately)
  lr           0.165 m (CoM to rear axle)
  mass         1.5 kg  (weigh with battery)
  iz           0.04 kg·m² (estimated: 1/12 * mass * (l^2 + w^2))

Requirements
────────────
  pip install numpy scipy pandas
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import least_squares

# ── Physical constants (measure these on your car) ────────────────────────

WHEELBASE = 0.33  # m
LF = 0.165  # m, CoM to front axle
LR = 0.165  # m, CoM to rear axle
MASS = 1.5  # kg
IZ = 0.04  # kg·m²


# ── Load data ─────────────────────────────────────────────────────────────


def load_csv(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    required = {"steering_rad", "throttle", "current_speed_ms", "gz_rad_s", "loop_us"}
    missing = required - set(df.columns)
    if missing:
        print(f"ERROR: missing columns: {missing}", file=sys.stderr)
        sys.exit(1)

    # Derived columns
    df["dt"] = df["loop_us"] / 1e6
    df["vx"] = df["current_speed_ms"].clip(lower=0.1)  # avoid div/0
    df["delta"] = df["steering_rad"]  # front wheel angle

    # Finite-difference lateral acceleration from vy
    if "vy_ms" in df.columns:
        df["ay_meas"] = df["vy_ms"].diff() / df["dt"]
    else:
        # Approximate: ay ≈ vx * d(gz)/dt  (only valid for small slip)
        df["ay_meas"] = df["vx"] * df["gz_rad_s"].diff() / df["dt"]

    df["yr_meas"] = df["gz_rad_s"]

    # Drop transients (first/last 0.5 s) and ESTOP ticks
    hz = 1.0 / df["dt"].median()
    trim = int(0.5 * hz)
    df = df.iloc[trim:-trim].copy()
    if "estop" in df.columns:
        df = df[df["estop"] == 0].copy()
    df.dropna(subset=["ay_meas", "yr_meas"], inplace=True)
    print(f"identify: {len(df)} usable ticks after trimming")
    return df


# ── Residual function for OLS ──────────────────────────────────────────────


def residuals(params, df):
    cf_val, cr_val, motor_g, drag_k = params

    vx = df["vx"].values
    vy = df["vy_ms"].values if "vy_ms" in df.columns else np.zeros(len(df))
    yr = df["yr_meas"].values
    delta = df["delta"].values
    throt = df["throttle"].values

    # Linear tyre model slip angles
    alpha_f = delta - (vy + LF * yr) / vx
    alpha_r = -(vy - LR * yr) / vx

    fy_f = cf_val * alpha_f
    fy_r = cr_val * alpha_r

    ay_pred = (fy_f * np.cos(delta) + fy_r) / MASS - vx * yr
    yr_accel_pred = (LF * fy_f - LR * fy_r) / IZ

    # Longitudinal residual on vx derivative
    ax_pred = throt * motor_g - drag_k * vx**2
    ax_meas = np.gradient(vx, df["dt"].values)

    r_ay = (ay_pred - df["ay_meas"].values) / (df["ay_meas"].values.std() + 1e-6)
    r_yr = (yr_accel_pred - np.gradient(yr, df["dt"].values)) / (yr.std() + 1e-6)
    r_ax = (ax_pred - ax_meas) / (ax_meas.std() + 1e-6)

    return np.concatenate([r_ay, r_yr, r_ax])


# ── Main ───────────────────────────────────────────────────────────────────


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", type=Path, help="metrics CSV file")
    ap.add_argument("--out", type=Path, default=Path("model_params.json"))
    ap.add_argument("--wheelbase", type=float, default=WHEELBASE)
    ap.add_argument("--lf", type=float, default=LF)
    ap.add_argument("--lr", type=float, default=LR)
    ap.add_argument("--mass", type=float, default=MASS)
    ap.add_argument("--iz", type=float, default=IZ)
    args = ap.parse_args()

    df = load_csv(args.csv)

    # Initial guess: [Cf, Cr, motor_gain, drag_k]
    x0 = np.array([12.0, 10.0, 8.0, 0.5])
    bounds = ([0.1, 0.1, 0.1, 0.0], [200.0, 200.0, 50.0, 5.0])

    print("identify: running least-squares optimisation...")
    result = least_squares(
        residuals, x0, bounds=bounds, args=(df,), method="trf", verbose=1
    )

    cf_fit, cr_fit, motor_g_fit, drag_k_fit = result.x
    rmse_lateral = float(np.sqrt(np.mean(result.fun[: len(df)] ** 2)))
    rmse_yaw_rate = float(np.sqrt(np.mean(result.fun[len(df) : 2 * len(df)] ** 2)))

    params = {
        "wheelbase": args.wheelbase,
        "lf": args.lf,
        "lr": args.lr,
        "mass": args.mass,
        "iz": args.iz,
        "cf": float(cf_fit),
        "cr": float(cr_fit),
        "motor_gain": float(motor_g_fit),
        "drag_k": float(drag_k_fit),
        "accel_max": 20.0,
        "source_mcap": str(args.csv),
        "rmse_lateral": rmse_lateral,
        "rmse_yaw_rate": rmse_yaw_rate,
    }

    print("\n── Identified parameters ──")
    for k, v in params.items():
        if isinstance(v, float):
            print(f"  {k:20s} = {v:.4f}")
        else:
            print(f"  {k:20s} = {v}")

    args.out.write_text(json.dumps(params, indent=2))
    print(f"\nidentify: written to {args.out}")


if __name__ == "__main__":
    main()
