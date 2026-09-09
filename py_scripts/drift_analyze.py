#!/usr/bin/env python3
"""Analyze/plot a drift_log.py capture (one summary row per window, e.g.
15 min): orientation and gyro-bias drift over time, with a headline
deg/hour drift rate for each axis. Safe to run on a partial (still-running)
log file.

Usage:
    python drift_analyze.py drift_12h.csv [--no-plot]
"""
import argparse

import numpy as np

FIELDS = ["roll", "pitch", "yaw", "bias_x", "bias_y", "bias_z",
          "rate_x", "rate_y", "rate_z", "accel_x", "accel_y", "accel_z"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    with open(args.csv) as fh:
        header = fh.readline().strip().split(",")
    data = np.loadtxt(args.csv, delimiter=",", skiprows=1, ndmin=2)
    col = {name: i for i, name in enumerate(header)}

    t = data[:, col["t_s"]] / 3600.0  # hours
    duration_h = t[-1] - t[0] if len(t) > 1 else 0.0
    print(f"{len(data)} windows over {duration_h:.2f} hours "
          f"(total samples: {int(data[:, col['n']].sum())})")
    print()

    means = {f: data[:, col[f"{f}_mean"]] for f in FIELDS}

    for f in ["roll", "pitch", "yaw"]:
        series = means[f]
        total = series[-1] - series[0] if len(series) > 1 else 0.0
        fit_rate = np.polyfit(t, series, 1)[0] if len(t) > 1 else float("nan")
        within_window_std = data[:, col[f"{f}_std"]].mean()
        print(f"{f:6s}: total drift={total:+7.3f} deg over {duration_h:.2f}h  "
              f"(linear-fit rate={fit_rate:+.4f} deg/h)  "
              f"mean within-window std={within_window_std:.4f} deg  "
              f"range=[{series.min():.3f}, {series.max():.3f}]")

    print()
    for f in ["bias_x", "bias_y", "bias_z"]:
        series = means[f]
        total = series[-1] - series[0] if len(series) > 1 else 0.0
        fit_rate = np.polyfit(t, series, 1)[0] if len(t) > 1 else float("nan")
        within_window_std = data[:, col[f"{f}_std"]].mean()
        print(f"{f:6s}: total change={total:+7.4f} dps over {duration_h:.2f}h  "
              f"(linear-fit rate={fit_rate:+.5f} dps/h)  "
              f"mean within-window std={within_window_std:.4f} dps  "
              f"range=[{series.min():.4f}, {series.max():.4f}]")

    print()
    for f in ["rate_x", "rate_y", "rate_z"]:
        series = means[f]
        within_window_std = data[:, col[f"{f}_std"]].mean()
        print(f"{f:6s}: mean={series.mean():+7.4f} dps  "
              f"mean within-window std={within_window_std:.4f} dps  "
              f"range=[{series.min():.4f}, {series.max():.4f}]  "
              f"(debiased angular rate -- stationary, so this is pure noise; should hover near 0)")

    print()
    for f in ["accel_x", "accel_y", "accel_z"]:
        series = means[f]
        within_window_std = data[:, col[f"{f}_std"]].mean()
        print(f"{f:6s}: mean={series.mean():+7.5f} g  "
              f"mean within-window std={within_window_std:.5f} g  "
              f"range=[{series.min():.5f}, {series.max():.5f}]")

    if args.no_plot:
        return

    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(11, 12))
    for f in ["roll", "pitch", "yaw"]:
        axes[0].errorbar(t, means[f], yerr=data[:, col[f"{f}_std"]], label=f, capsize=2)
    axes[0].set_ylabel("orientation (deg)")
    axes[0].legend()
    axes[0].grid(alpha=0.3)

    for f in ["bias_x", "bias_y", "bias_z"]:
        axes[1].errorbar(t, means[f], yerr=data[:, col[f"{f}_std"]], label=f, capsize=2)
    axes[1].set_ylabel("gyro bias (deg/s)")
    axes[1].legend()
    axes[1].grid(alpha=0.3)

    for f in ["rate_x", "rate_y", "rate_z"]:
        axes[2].errorbar(t, means[f], yerr=data[:, col[f"{f}_std"]], label=f, capsize=2)
    axes[2].set_ylabel("debiased angular rate (deg/s)")
    axes[2].legend()
    axes[2].grid(alpha=0.3)

    for f in ["accel_x", "accel_y", "accel_z"]:
        axes[3].errorbar(t, means[f], yerr=data[:, col[f"{f}_std"]], label=f, capsize=2)
    axes[3].set_ylabel("accel (g)")
    axes[3].set_xlabel("time (hours)")
    axes[3].legend()
    axes[3].grid(alpha=0.3)

    fig.suptitle(f"Stationary drift over {duration_h:.2f}h, per-window mean +/- std ({args.csv})")
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
