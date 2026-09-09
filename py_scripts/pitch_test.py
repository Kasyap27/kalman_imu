#!/usr/bin/env python3
"""Capture N seconds of attitude EKF telemetry and quantify how much of the
observed yaw-rate is explained by Euler kinematic coupling (p,q,r -> yaw-rate
via the ZYX Euler-rate equation) versus unexplained (residual) noise -- the
same "windowed instantaneous-rate correlation" methodology used throughout
the magnetometer-disturbance-gate tuning: measured yaw-rate is regressed
against kinematic-predicted yaw-rate, and the residual std is the headline
number (lower = less yaw noise not explained by legitimate motion, i.e. less
likely to be magnetometer disturbance/miscalibration leaking through).

Usage:
    python pitch_test.py [--port /dev/ttyUSB1] [--seconds 20] [--out capture.csv]

Move the board (e.g. a clean pitch-only sweep) during the capture window.
"""
import argparse
import math
import sys
import time

import numpy as np
import serial

from imu_live_plot import read_packets, quat_to_euler_deg


def capture(port, baud, seconds, out_path):
    ser = serial.Serial(port, baud, timeout=0.05)
    print(f"Opened {port} @ {baud}. Capturing for {seconds:.0f}s -- move the board now.")
    rows = []
    t_start = None
    with open(out_path, "w") as f:
        f.write("t,seq,roll,pitch,yaw,p,q,r\n")
        for pkt in read_packets(ser):
            if t_start is None:
                t_start = pkt["t"]
            roll, pitch, yaw = quat_to_euler_deg(pkt["q"])
            p, q, r = pkt["angular_rate"]
            rows.append((pkt["t"], pkt["seq"], roll, pitch, yaw, p, q, r))
            f.write(f"{pkt['t']:.6f},{pkt['seq']},{roll:.5f},{pitch:.5f},{yaw:.5f},"
                    f"{p:.6f},{q:.6f},{r:.6f}\n")
            if pkt["t"] - t_start >= seconds:
                break
    print(f"Captured {len(rows)} good packets to {out_path}")
    return np.array(rows)


def analyze(rows):
    t, seq, roll, pitch, yaw, p, q, r = rows.T
    roll_rad, pitch_rad = np.radians(roll), np.radians(pitch)

    # Kinematic yaw-rate prediction (ZYX Euler rates) from measured body
    # rates and the filter's own current roll/pitch estimate:
    #   psi_dot = [q*sin(phi) + r*cos(phi)] / cos(theta)
    cos_pitch = np.cos(pitch_rad)
    kinematic_yaw_rate = np.degrees(
        (q * np.sin(roll_rad) + r * np.cos(roll_rad)) / cos_pitch
    )

    dt = np.diff(t)
    measured_yaw_rate = np.diff(np.unwrap(np.radians(yaw))) / dt
    measured_yaw_rate = np.degrees(measured_yaw_rate)
    kin_mid = 0.5 * (kinematic_yaw_rate[:-1] + kinematic_yaw_rate[1:])

    # Guard against bad dt (duplicate/out-of-order timestamps).
    valid = dt > 1e-4
    measured_yaw_rate, kin_mid = measured_yaw_rate[valid], kin_mid[valid]

    meas_std = measured_yaw_rate.std()
    kin_std = kin_mid.std()
    corr = np.corrcoef(measured_yaw_rate, kin_mid)[0, 1]
    r2 = corr ** 2
    residual = measured_yaw_rate - kin_mid
    residual_std = residual.std()

    print()
    print(f"roll range: {roll.min():.2f} to {roll.max():.2f}")
    print(f"pitch range: {pitch.min():.2f} to {pitch.max():.2f}")
    print(f"yaw std: {yaw.std():.2f} deg, total swing: {yaw.max() - yaw.min():.2f} deg")
    print()
    print(f"measured yaw-rate std={meas_std:.2f} dps, kinematic std={kin_std:.2f} dps")
    print(f"R^2={100 * r2:.1f}%  residual std={residual_std:.2f} dps")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB1")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--out", default="pitch_test.csv")
    args = ap.parse_args()

    rows = capture(args.port, args.baud, args.seconds, args.out)
    if len(rows) < 10:
        print("Too few packets captured.", file=sys.stderr)
        sys.exit(1)
    analyze(rows)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
