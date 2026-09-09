#!/usr/bin/env python3
"""Full 9/10-parameter (ellipsoid) magnetometer calibration.

Fits the raw mag samples (captured via mag_capture.py, one CSV row per
sample: seq,mx_raw,my_raw,mz_raw) to a general ellipsoid

    (m - b)^T W^T W (m - b) = 1

instead of the simpler 6-parameter hard-iron-bias + per-axis-diagonal-scale
model. The extra 3 off-diagonal terms in W correct for soft-iron coupling
between axes and axis misalignment, which a diagonal scale cannot.

Usage:
    python mag_calibrate.py mag_raw.csv

Prints the bias vector b (3) and correction matrix W (3x3, symmetric) to
paste into main.c as MAG_BIAS and MAG_SOFT_IRON (replacing the old
MAG_BIAS_MG / MAG_SCALE diagonal-only constants), and reports fit quality
(residual norm-error before/after correction) so you can see whether the
9-param fit is actually better than the old 6-param one on the same data.
"""
import argparse
import sys

import numpy as np


def load_csv(path):
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    return data[:, 1:4]  # mx, my, mz raw


def fit_ellipsoid(m):
    """Fit (m-b)^T A (m-b) = 1 via the standard unconstrained algebraic
    least-squares method (assumes the quadric's constant term is nonzero,
    true for any ellipsoid not passing through the origin -- always true
    here since raw mag has a large hard-iron offset)."""
    x, y, z = m[:, 0], m[:, 1], m[:, 2]
    D = np.column_stack([x * x, y * y, z * z, 2 * x * y, 2 * x * z, 2 * y * z,
                          2 * x, 2 * y, 2 * z])
    rhs = np.ones(len(m))
    v, *_ = np.linalg.lstsq(D, rhs, rcond=None)
    a, b, c, d, e, f, g, h, i = v

    Q3 = np.array([[a, d, e],
                   [d, b, f],
                   [e, f, c]])
    u = np.array([g, h, i])

    center = -np.linalg.solve(Q3, u)
    k = 1.0 - u @ center  # see derivation: q^T Q3 q = k after translating to center
    A = Q3 / k

    # A must be symmetric positive-definite for a real ellipsoid fit.
    eigvals, eigvecs = np.linalg.eigh(A)
    if np.any(eigvals <= 0):
        raise ValueError(
            f"Fit is not a proper ellipsoid (eigenvalues {eigvals}); "
            "need better angular coverage in the capture (more/rounder figure-8s)."
        )
    # Symmetric PSD square root: W = V diag(sqrt(eigvals)) V^T, so W^T W = A.
    W = eigvecs @ np.diag(np.sqrt(eigvals)) @ eigvecs.T
    return center, W, eigvals


def diagonal_calibration(m):
    """Old 6-parameter method for comparison: bias = (max+min)/2 per axis,
    scale = avg_radius / axis_radius."""
    mins = m.min(axis=0)
    maxs = m.max(axis=0)
    bias = (mins + maxs) / 2.0
    radii = (maxs - mins) / 2.0
    avg_radius = radii.mean()
    scale = avg_radius / radii
    return bias, scale


def report_residual(label, norms):
    print(f"{label}: mean |m|={norms.mean():.4f}  std={norms.std():.4f}  "
          f"(std/mean={100 * norms.std() / norms.mean():.2f}%)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    args = ap.parse_args()

    m = load_csv(args.csv)
    print(f"Loaded {len(m)} samples from {args.csv}")

    raw_norms = np.linalg.norm(m, axis=1)
    report_residual("raw (uncalibrated)", raw_norms)

    # Old diagonal method, for a side-by-side comparison on the same data.
    diag_bias, diag_scale = diagonal_calibration(m)
    diag_corrected = (m - diag_bias) * diag_scale
    report_residual("old 6-param (diag hard/soft-iron)", np.linalg.norm(diag_corrected, axis=1))

    # New ellipsoid fit.
    center, W, eigvals = fit_ellipsoid(m)
    ell_corrected = (m - center) @ W.T
    report_residual("new 9-param (ellipsoid)", np.linalg.norm(ell_corrected, axis=1))

    print()
    print("Ellipsoid eigenvalues (axis-length sanity check, should be within ~10-20% "
          f"of each other for a well-conditioned capture): {eigvals}")
    print()
    print("--- paste into main.c ---")
    print("static const float MAG_BIAS_MG[3] = { "
          f"{center[0]:.4f}f, {center[1]:.4f}f, {center[2]:.4f}f }};")
    print("static const float MAG_SOFT_IRON[3][3] = {")
    for row in W:
        print("    { " + ", ".join(f"{v:.6f}f" for v in row) + " },")
    print("};")


if __name__ == "__main__":
    main()
