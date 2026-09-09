#!/usr/bin/env python3
"""Capture raw magnetometer CSV stream from the board's mag-calibration mode
(APP_MODE_MAG_CALIBRATION) and save it to a file for offline calibration.

Board must already be flashed with APP_MODE_MAG_CALIBRATION set (idf.py
menuconfig -> Kalman IMU Application -> Application mode, or edit sdkconfig
directly and idf.py reconfigure && idf.py -p <PORT> flash).

Usage:
    python mag_capture.py /dev/ttyUSB0 [--baud 921600] [--seconds 60] [--out mag_raw.csv]

While it runs, wave the board through as many figure-8s / orientations as
possible to cover the full sphere of orientations -- the ellipsoid fit needs
good angular coverage to be well-conditioned.
"""
import argparse
import re
import sys
import time

import serial

LINE_RE = re.compile(rb"^(\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+)\s*$")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--out", default="mag_raw.csv")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=1.0)
    print(f"Opened {args.port} @ {args.baud}. Waiting for stream...")

    rows = []
    t_end = None
    with open(args.out, "w") as f:
        f.write("seq,mx_raw,my_raw,mz_raw,ax_raw,ay_raw,az_raw\n")
        while True:
            line = ser.readline()
            if not line:
                continue
            m = LINE_RE.match(line.strip())
            if not m:
                # startup banner / non-data lines
                text = line.decode(errors="replace").strip()
                if text:
                    print(text)
                continue
            if t_end is None:
                t_end = time.monotonic() + args.seconds
                print(f"Streaming started -- capturing for {args.seconds:.0f}s. "
                      f"Wave the board through many orientations now.")
            seq, mx, my, mz, ax, ay, az = (int(g) for g in m.groups())
            rows.append((seq, mx, my, mz, ax, ay, az))
            f.write(f"{seq},{mx},{my},{mz},{ax},{ay},{az}\n")
            if len(rows) % 200 == 0:
                print(f"  {len(rows)} samples captured...")
            if t_end is not None and time.monotonic() >= t_end:
                break

    print(f"Done: {len(rows)} samples written to {args.out}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
