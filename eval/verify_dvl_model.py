#!/usr/bin/env python3
"""Verify that the DVL error model in a bag is the unit the seed specifies.

The per-axis gains that analyze_anchorfree prints cannot separate the scale
factor from the misalignment's cross-axis coupling: a single-axis regression
of v_out[i] on v_true[i] absorbs whatever the other axes leak in, so a unit
with large misalignment reads as having the wrong gain.  Recovering the whole
3x3 operator by least squares separates them, and it is an exact test rather
than an approximate one -- every element has a predicted value.

The operator maps BODY velocity to what the bridge publishes:

    E = R . S . dR . R

The trailing R is easy to miss.  The bridge applies S and dR in the DVL's own
frame, so the true body velocity is rotated into that frame first; leaving it
off flips the sign of the last two columns and makes a correct bag look wrong.

Usage:  verify_dvl_model.py <bag> <seed> [--scale-sigma F] [--misalign-deg F]
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analyze_anchorfree as A


def rpy(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("seed", type=int)
    ap.add_argument("--scale-sigma", type=float, default=0.0101)
    ap.add_argument("--misalign-deg", type=float, default=0.5)
    ap.add_argument("--mount-roll-deg", type=float, default=180.0)
    ap.add_argument("--tol", type=float, default=0.002,
                    help="max elementwise difference to call it a match")
    args = ap.parse_args()

    # Reproduce the node's draw.  dvl_bridge uses one default_rng(seed) and
    # takes the scales first, so the order here has to match it exactly.
    rng = np.random.default_rng(args.seed)
    s = 1.0 + rng.normal(0.0, args.scale_sigma, 3)
    dr, dp, dy = rng.normal(0.0, math.radians(args.misalign_deg), 3)
    R = rpy(math.radians(args.mount_roll_deg), 0.0, 0.0)
    E_pred = R @ np.diag(s) @ rpy(dr, dp, dy) @ R

    D = A.load(args.bag)
    if A.T_DVL not in D or A.T_TRUTH not in D:
        sys.exit("bag is missing /bluerov2/dvl_twist or ground truth")
    truth, dvl = D[A.T_TRUTH], D[A.T_DVL]
    tv = A.interp_to(truth[:, 0], truth[:, 5:8], dvl[:, 0])
    moving = np.hypot(tv[:, 0], tv[:, 1]) > 0.05
    X, Y = tv[moving], dvl[moving, 1:4]
    if len(X) < 100:
        sys.exit(f"only {len(X)} moving samples; not enough to fit")
    E_fit = np.linalg.lstsq(X, Y, rcond=None)[0].T

    print("=" * 72)
    print(f"DVL ERROR MODEL CHECK: {os.path.basename(args.bag)}  seed {args.seed}")
    print("=" * 72)
    print(f"  realised unit: scale=({s[0]:.5f}, {s[1]:.5f}, {s[2]:.5f})  "
          f"misalignment=({math.degrees(dr):+.3f}, {math.degrees(dp):+.3f}, "
          f"{math.degrees(dy):+.3f}) deg")
    print(f"  fitted on {moving.sum()} samples while moving\n")
    print("    predicted                    recovered                    |diff|")
    for i in range(3):
        print("    " + " ".join(f"{E_pred[i, j]:+.5f}" for j in range(3))
              + "    " + " ".join(f"{E_fit[i, j]:+.5f}" for j in range(3))
              + "    " + " ".join(f"{abs(E_pred[i, j] - E_fit[i, j]):.5f}"
                                  for j in range(3)))
    worst = np.abs(E_pred - E_fit).max()
    print(f"\n  max elementwise difference {worst:.5f}")
    if worst <= args.tol:
        print(f"  MATCH: this bag carries the seed {args.seed} unit.")
    else:
        print(f"  MISMATCH (tolerance {args.tol}). Either the seed is not the "
              f"one that was run, or\n  the error model did not reach the "
              f"filter. Check the dvl_bridge warning in the launch log.")
        sys.exit(1)
    print("=" * 72)


if __name__ == "__main__":
    main()
