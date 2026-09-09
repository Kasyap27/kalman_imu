#!/usr/bin/env python3
"""Long-duration (multi-hour) attitude EKF telemetry logger, for
characterizing stationary drift -- gyro bias convergence/wander, residual
yaw drift from magnetometer noise, etc. -- over many hours instead of the
short (10-20s) live tests used for the rest of this project's tuning.

Leave the board perfectly still (flat, undisturbed, away from anything
magnetic that might move nearby -- fans, phones, etc.) for the full run.

Rather than logging every raw sample (12h at even 1Hz is still 43200 rows
for no real analytical benefit -- drift is a slow phenomenon), this buffers
samples internally at the full telemetry rate and writes ONE summary row
per window (default 15 min): mean/std/min/max for roll/pitch/yaw and gyro
bias, plus the window's own drift (last-first). That keeps a 12h run to
~48 rows while still capturing both the drift trend AND the noise/wander
within each window. Written incrementally (flushed each window) so a
partial run is fully usable if interrupted.

Usage:
    python drift_log.py --port /dev/ttyUSB1 --hours 12 --out drift_12h.csv

Run this with nohup (or in tmux/screen) if you might close the terminal:
    nohup python drift_log.py --hours 12 --out drift_12h.csv > drift_12h.log 2>&1 &

Then analyze whenever (including mid-run, on the partial file) with:
    python drift_analyze.py drift_12h.csv
"""
import argparse
import math
import signal
import sys
import time

import numpy as np
import serial

from imu_live_plot import read_packets, quat_to_euler_deg

FIELDS = ["roll", "pitch", "yaw", "bias_x", "bias_y", "bias_z",
          "rate_x", "rate_y", "rate_z", "accel_x", "accel_y", "accel_z"]


def window_header():
    cols = ["t_s", "n"]
    for f in FIELDS:
        cols += [f"{f}_mean", f"{f}_std", f"{f}_min", f"{f}_max", f"{f}_drift"]
    return ",".join(cols)


def summarize_window(t_s, buf):
    """buf: dict of field -> list of samples for this window."""
    parts = [f"{t_s:.1f}", str(len(buf["roll"]))]
    for f in FIELDS:
        arr = np.array(buf[f])
        if f == "yaw":
            arr = np.degrees(np.unwrap(np.radians(arr)))  # avoid a +/-180 wrap corrupting mean/drift
        drift = arr[-1] - arr[0]
        parts += [f"{arr.mean():.5f}", f"{arr.std():.5f}", f"{arr.min():.5f}", f"{arr.max():.5f}", f"{drift:.5f}"]
    return ",".join(parts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB1")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--hours", type=float, default=12.0)
    ap.add_argument("--window-minutes", type=float, default=15.0,
                     help="aggregation window size in minutes (default 15)")
    ap.add_argument("--out", default="drift_log.csv")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    print(f"Opened {args.port} @ {args.baud}. Logging for {args.hours:.1f}h, one summary row per "
          f"{args.window_minutes:.0f} min, to {args.out}. Leave the board perfectly still.")

    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))

    window_s = args.window_minutes * 60.0
    t_run_end = time.monotonic() + args.hours * 3600.0
    t_wall_start = time.time()
    window_start_mono = time.monotonic()
    next_window_end = window_start_mono + window_s

    buf = {f: [] for f in FIELDS}
    n_windows = 0

    with open(args.out, "w") as f:
        f.write(window_header() + "\n")
        for pkt in read_packets(ser):
            now = time.monotonic()
            if now >= t_run_end or stop["flag"]:
                break

            roll, pitch, yaw = quat_to_euler_deg(pkt["q"])
            bx, by, bz = pkt["gyro_bias"]
            rx, ry, rz = pkt["angular_rate"]
            ax, ay, az = pkt["accel"]
            buf["roll"].append(roll)
            buf["pitch"].append(pitch)
            buf["yaw"].append(yaw)
            buf["bias_x"].append(math.degrees(bx))
            buf["bias_y"].append(math.degrees(by))
            buf["bias_z"].append(math.degrees(bz))
            buf["rate_x"].append(math.degrees(rx))
            buf["rate_y"].append(math.degrees(ry))
            buf["rate_z"].append(math.degrees(rz))
            buf["accel_x"].append(ax)
            buf["accel_y"].append(ay)
            buf["accel_z"].append(az)

            if now >= next_window_end:
                if buf["roll"]:
                    t_s = time.time() - t_wall_start
                    f.write(summarize_window(t_s, buf) + "\n")
                    f.flush()
                    n_windows += 1
                    hrs = t_s / 3600.0
                    print(f"[{hrs:5.2f}h] window {n_windows}: n={len(buf['roll'])}  "
                          f"roll={np.mean(buf['roll']):+7.3f}  pitch={np.mean(buf['pitch']):+7.3f}  "
                          f"yaw={np.mean(buf['yaw']):+7.3f}  "
                          f"bias=({np.mean(buf['bias_x']):+.4f},{np.mean(buf['bias_y']):+.4f},"
                          f"{np.mean(buf['bias_z']):+.4f}) dps")
                buf = {f: [] for f in FIELDS}
                next_window_end += window_s

        # Flush a final partial window on early stop/end of run.
        if buf["roll"]:
            t_s = time.time() - t_wall_start
            f.write(summarize_window(t_s, buf) + "\n")
            f.flush()
            n_windows += 1

    print(f"Done: {n_windows} window(s) logged to {args.out}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
