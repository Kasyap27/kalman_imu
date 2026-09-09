#!/usr/bin/env python3
"""Short, full-rate raw-accel capture for characterizing the frequency
content of real linear-acceleration disturbance (hand pushes/shoves, etc.).

Unlike drift_log.py (windowed summaries over hours -- drift is slow), this
logs every sample at the full telemetry rate (~200Hz) for a short run, since
we need actual frequency content, not aggregate stats. Analyze the result
with motion_psd_analyze.py to pick attitude_ekf's accel_lpf_time_const from
where the disturbance's energy actually sits, instead of guessing.

Usage:
    python motion_capture.py --port /dev/ttyUSB1 --seconds 60 --out push_test.csv

Hold the board flat and, starting a couple seconds in (so there's a clean
still baseline at the start), do the motion you want characterized --
repeated pushes/shoves along one axis, shakes, whatever real disturbance
you're trying to reject -- for the rest of the run.
"""
import argparse
import signal
import sys
import time

import serial

from imu_live_plot import read_packets


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB1")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--out", default="motion_capture.csv")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    print(f"Opened {args.port} @ {args.baud}. Capturing full-rate samples for "
          f"{args.seconds:.0f}s to {args.out}. Start still, then move a couple "
          f"seconds in.")

    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))

    t_end = time.monotonic() + args.seconds
    n = 0
    with open(args.out, "w") as f:
        f.write("seq,t_s,accel_x,accel_y,accel_z,rate_x,rate_y,rate_z\n")
        for pkt in read_packets(ser):
            if time.monotonic() >= t_end or stop["flag"]:
                break
            ax, ay, az = pkt["accel"]
            rx, ry, rz = pkt["angular_rate"]
            f.write(f"{pkt['seq']},{pkt['t']:.6f},{ax:.6f},{ay:.6f},{az:.6f},"
                     f"{rx:.6f},{ry:.6f},{rz:.6f}\n")
            n += 1

    print(f"Done: {n} samples logged to {args.out}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
