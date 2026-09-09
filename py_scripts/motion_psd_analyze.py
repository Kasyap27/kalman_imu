#!/usr/bin/env python3
"""Analyze a motion_capture.py CSV to pick attitude_ekf's
accel_lpf_time_const from real data instead of a guessed constant.

Computes each accel axis's power spectral density (Welch's method) and
reports the frequency below which most of the *disturbance* energy (i.e.
excluding the DC/near-DC bin, which is gravity itself, not what we're
trying to filter out) is concentrated. Recommends a cutoff one decade below
that, since a single-pole low-pass only reaches ~-20dB attenuation a decade
past its corner (at the corner itself it's only -3dB -- see the fc=1Hz
discussion this script exists to replace with a derived number).

Usage:
    python motion_psd_analyze.py push_test.csv
    python motion_psd_analyze.py push_test.csv --no-plot
"""
import argparse

import numpy as np
from scipy.signal import welch


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--energy-fraction", type=float, default=0.9,
                     help="cumulative disturbance-energy fraction defining the "
                          "'disturbance band' edge (default 0.9)")
    args = ap.parse_args()

    data = np.genfromtxt(args.csv, delimiter=",", names=True)
    t = data["t_s"]
    dt = np.median(np.diff(t))
    fs = 1.0 / dt
    print(f"{len(t)} samples, median dt={dt*1000:.2f}ms -> fs={fs:.1f}Hz "
          f"(span {t[-1]-t[0]:.1f}s)")

    axes = {"x": data["accel_x"], "y": data["accel_y"], "z": data["accel_z"]}

    # nperseg: long enough for decent low-frequency resolution (~0.1Hz bins)
    # without exceeding the record length.
    nperseg = int(min(len(t), fs * 10))

    fig = None
    if not args.no_plot:
        import matplotlib.pyplot as plt
        fig, ax_plot = plt.subplots(figsize=(9, 5))

    edges = []
    for label, sig in axes.items():
        sig = sig - np.mean(sig)  # drop DC (gravity's static component, not disturbance)
        f, pxx = welch(sig, fs=fs, nperseg=nperseg)

        # Exclude the very first (near-DC) bin from the energy accounting --
        # any slow baseline wander/tilt-holding drift would otherwise dominate
        # the "disturbance" accounting despite not being the fast motion
        # artifact we're trying to characterize.
        cum = np.cumsum(pxx[1:])
        cum /= cum[-1]
        idx = np.searchsorted(cum, args.energy_fraction)
        f_edge = f[1:][min(idx, len(cum) - 1)]
        edges.append(f_edge)
        print(f"  accel_{label}: {args.energy_fraction*100:.0f}% of disturbance "
              f"energy below {f_edge:.3f} Hz")

        if fig is not None:
            ax_plot.semilogy(f, pxx, label=f"accel_{label}")
            ax_plot.axvline(f_edge, linestyle="--", alpha=0.4)

    f_disturbance = max(edges)
    fc = f_disturbance / 10.0
    tau = 1.0 / (2 * np.pi * fc)
    print(f"\nDisturbance band edge (max over axes): {f_disturbance:.3f} Hz")
    print(f"Recommended accel LPF cutoff (1 decade below): fc = {fc:.4f} Hz")
    print(f"=> accel_lpf_time_const = 1/(2*pi*fc) = {tau:.2f} s")

    if fig is not None:
        ax_plot.axvline(fc, color="k", linewidth=2, label=f"recommended fc={fc:.3f}Hz")
        ax_plot.set_xlabel("frequency (Hz)")
        ax_plot.set_ylabel("PSD (g^2/Hz)")
        ax_plot.set_xlim(0, min(fs / 2, 10))
        ax_plot.legend()
        ax_plot.set_title("Accel disturbance PSD (motion_capture.py)")
        import matplotlib.pyplot as plt
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
