#!/usr/bin/env python3
"""Fit a small residual rotation between the (hard/soft-iron corrected)
magnetometer frame and the accel/gyro body frame -- a "boresight" alignment
calibration, on top of the existing ellipsoid calibration (mag_calibrate.py).

Physical basis: Earth's magnetic field has a fixed dip angle relative to
gravity at a given location. For a mag reading that's both well-calibrated
(magnitude-corrected) AND perfectly axis-aligned with the accel/gyro frame,
the angle between the (normalized) mag vector and the (normalized) gravity
vector -- both expressed in body frame -- should be CONSTANT no matter how
the board is tilted. If it varies with orientation, that variation is
evidence of, and lets us solve for, a small residual rotation between the
mag die and the accel/gyro die that the coarse 90/180-degree axis remap
(icm20948.c) doesn't capture (real PCB mounting is never exactly
orthogonal).

Method: linearized (small-angle) least squares. For unit vectors m_i
(calibrated mag, body frame) and a_i (normalized accel, body frame), and a
small correction rotation R = I + skew(theta):

    m_i . (R^T a_i) = m_i.a_i - theta . (a_i x m_i)   (first order)

Solving `theta . (a_i x m_i) + c = m_i.a_i` by linear least squares over all
samples gives the correction vector theta (and c, the fitted constant dip
cosine, discarded). Two passes re-linearize around the updated rotation for
samples with non-negligible correction.

Usage:
    python mag_align.py accel_mag_raw.csv

CSV format (from mag_calib_task with accel logging):
    seq,mx_raw,my_raw,mz_raw,ax_raw,ay_raw,az_raw
"""
import argparse

import numpy as np

from mag_calibrate import fit_ellipsoid


def rodrigues(theta):
    """Exact rotation matrix for rotation vector theta (exponential map)."""
    angle = np.linalg.norm(theta)
    if angle < 1e-12:
        return np.eye(3)
    axis = theta / angle
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


def fit_alignment(m, a, iterations=3):
    """m, a: (N,3) unit vectors (calibrated mag, normalized accel), body frame."""
    R_total = np.eye(3)
    for _ in range(iterations):
        m_rot = m @ R_total.T  # current best-aligned mag
        cross = np.cross(a, m_rot)  # (N,3), = a_i x m_i
        d = np.einsum("ij,ij->i", m_rot, a)  # m_i . a_i
        A = np.column_stack([cross, np.ones(len(d))])
        sol, *_ = np.linalg.lstsq(A, d, rcond=None)
        theta = sol[:3]
        R_total = rodrigues(theta) @ R_total
    return R_total


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    args = ap.parse_args()

    data = np.loadtxt(args.csv, delimiter=",", skiprows=1)
    mag_raw = data[:, 1:4]
    accel_raw = data[:, 4:7]
    print(f"Loaded {len(data)} samples from {args.csv}")

    # Re-fit the hard/soft-iron ellipsoid on this same dataset for consistency.
    center, W, eigvals = fit_ellipsoid(mag_raw)
    print(f"Ellipsoid eigenvalue spread: {eigvals} "
          f"(ratio {eigvals.max() / eigvals.min():.2f}, want < ~1.3 for a well-conditioned capture)")

    m_cal = (mag_raw - center) @ W.T
    m_unit = m_cal / np.linalg.norm(m_cal, axis=1, keepdims=True)
    a_unit = accel_raw / np.linalg.norm(accel_raw, axis=1, keepdims=True)

    dip_cos_before = np.einsum("ij,ij->i", m_unit, a_unit)
    print()
    print(f"dip angle before alignment: mean={np.degrees(np.arccos(np.clip(dip_cos_before.mean(),-1,1))):.2f} deg "
          f"std(cos)={dip_cos_before.std():.5f}")

    R = fit_alignment(m_unit, a_unit)
    angle_deg = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
    print(f"\nFitted alignment correction: {angle_deg:.3f} deg rotation")

    m_aligned = m_unit @ R.T
    dip_cos_after = np.einsum("ij,ij->i", m_aligned, a_unit)
    print(f"dip angle after alignment:  mean={np.degrees(np.arccos(np.clip(dip_cos_after.mean(),-1,1))):.2f} deg "
          f"std(cos)={dip_cos_after.std():.5f}")
    improvement = 100 * (1 - dip_cos_after.std() / dip_cos_before.std())
    print(f"dip-angle-consistency improvement: {improvement:.1f}%")

    # Combine with the ellipsoid's soft-iron matrix so main.c only needs one
    # 3x3 multiply after subtracting bias, same as before: R @ W.
    combined = R @ W

    print()
    print("--- paste into main.c ---")
    print("static const float MAG_BIAS_RAW[3] = { "
          f"{center[0]:.4f}f, {center[1]:.4f}f, {center[2]:.4f}f }};")
    print("static const float MAG_SOFT_IRON[3][3] = {")
    for row in combined:
        print("    { " + ", ".join(f"{v:.6f}f" for v in row) + " },")
    print("};")


if __name__ == "__main__":
    main()
