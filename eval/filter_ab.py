#!/usr/bin/env python3
"""Does fusing SLAM beat not fusing it? The within-run A/B.

`bluerov2_sim.py dr_reference:=true` runs a second EKF that is identical to
the primary one except that it has no SLAM input. Both consume the same IMU,
the same DVL, the same depth, on the same flown trajectory, so the pair
isolates exactly one variable: whether the SLAM pose is fused. Comparing
instead against a DIFFERENT run's dead reckoning would confound the answer
with the trajectory, which the heading noise also changes.

Error is horizontal position against `/bluerov2/odometry`, matching
`analyze_anchorfree.py`; depth is pressure-aided in both filters and is not
in contention. Both estimates are resampled onto ground-truth timestamps so
the two RMSEs are averages over the same instants, not over each filter's
own publishing grid.

The run is split at the first SLAM pose the bridge passed to the filter:
before that instant the two filters are the same filter, and including that
stretch would dilute the comparison toward 1.0 by construction.

Usage:  filter_ab.py BAG [BAG ...] [--jump-thresh M]
"""
import os
import sys

try:
    import mcap_ros2  # noqa: F401
except ImportError:
    _alt = os.path.expanduser("~/venv-ardupilot/bin/python3")
    if not os.environ.get("_BAGREADER_REEXEC") and os.path.exists(_alt) \
            and os.path.abspath(_alt) != os.path.abspath(sys.executable):
        os.environ["_BAGREADER_REEXEC"] = "1"
        os.execv(_alt, [_alt] + sys.argv)
    sys.exit("mcap_ros2 not found in any available interpreter")

import argparse
import glob

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_anchorfree import (T_SLAM, T_TRUTH, _iter_indexed,
                                _iter_recovered, interp_to, recorded_topics)

T_FUSED = "/bluerov2/odometry/filtered"
T_DR = "/bluerov2/odometry/dr"


def load(bag):
    files = sorted(glob.glob(os.path.join(bag, "*.mcap")))
    if not files:
        sys.exit(f"no .mcap under {bag}")
    want = (T_TRUTH, T_FUSED, T_DR, T_SLAM)
    out = {k: [] for k in want}
    for f in files:
        mark = {k: len(v) for k, v in out.items()}
        try:
            it = _iter_indexed(f, want)
            for topic, t, r in it:
                _row(out, topic, t, r)
        except Exception as e:
            print(f"  indexed read failed ({type(e).__name__}); streaming",
                  file=sys.stderr)
            for k, n in mark.items():
                del out[k][n:]
            for topic, t, r in _iter_recovered(f, set(want)):
                _row(out, topic, t, r)
    return {k: np.array(v) for k, v in out.items() if v}


def _row(out, topic, t, r):
    if topic == T_SLAM:
        out[topic].append((t,))
    else:
        p = r.pose.pose.position
        out[topic].append((t, p.x, p.y))


def rmse(e):
    return float(np.sqrt((e ** 2).mean()))


def report(bag, jump_thresh):
    name = os.path.basename(os.path.normpath(bag))
    print("=" * 72)
    print(f"SLAM-AIDED vs SLAM-FREE: {name}")
    print("=" * 72)

    rec = recorded_topics(bag)
    if T_DR not in rec:
        print(f"  {T_DR} was never recorded -- this run had no SLAM-free")
        print("  reference. Relaunch with dr_reference:=true. No A/B possible.")
        return
    D = load(bag)
    for need in (T_TRUTH, T_FUSED, T_DR):
        if need not in D:
            print(f"  {need} recorded but empty; no A/B possible.")
            return

    truth = D[T_TRUTH]
    tt, tx, ty = truth[:, 0], truth[:, 1], truth[:, 2]
    t0 = tt[0]

    def err_of(arr):
        e = interp_to(arr[:, 0], arr[:, 1:], tt)
        return np.hypot(e[:, 0] - tx, e[:, 1] - ty)

    ef, ed = err_of(D[T_FUSED]), err_of(D[T_DR])

    # Confine every comparison to the span both filters actually covered;
    # np.interp holds the end value flat outside a series' own range, which
    # would silently compare a real estimate against a frozen one.
    lo = max(D[T_FUSED][0, 0], D[T_DR][0, 0], tt[0])
    hi = min(D[T_FUSED][-1, 0], D[T_DR][-1, 0], tt[-1])
    span = (tt >= lo) & (tt <= hi)

    t_slam = D[T_SLAM][0, 0] if T_SLAM in D and len(D[T_SLAM]) else None
    if t_slam is None:
        print("  no SLAM pose ever reached the filter: the two filters are")
        print("  the same filter and the comparison is empty.")
        return

    windows = [("before SLAM (filters identical)", span & (tt < t_slam)),
               ("WITH SLAM -- the comparison", span & (tt >= t_slam)),
               ("whole run", span)]

    print(f"  duration {tt[-1] - t0:.0f}s   "
          f"first SLAM pose to the filter at t = {t_slam - t0:.0f}s")
    print()
    print(f"  {'window':34s} {'SLAM-free':>10s} {'SLAM-aided':>11s} "
          f"{'ratio':>7s}")
    verdict = None
    for label, m in windows:
        if m.sum() < 2:
            continue
        a, b = rmse(ed[m]), rmse(ef[m])
        r = b / a if a > 0 else float("nan")
        print(f"  {label:34s} {a:9.2f}m {b:10.2f}m {r:7.2f}")
        if label.startswith("WITH SLAM"):
            verdict = (a, b, r)
    print()

    # Jumps are looked for in the fused filter only: the SLAM-free reference
    # has nothing that can jump, so a step there would be a bug, not an event.
    m = span & (tt >= t_slam)
    jumps = []
    if m.sum() > 2:
        ts, es = tt[m], ef[m]
        d = np.abs(np.diff(es))
        for i in np.flatnonzero(d > jump_thresh):
            jumps.append((ts[i + 1] - t0, es[i], es[i + 1]))
    if jumps:
        print(f"  position-error steps > {jump_thresh:.1f} m in the fused "
              f"filter:")
        for t, a, b in jumps:
            print(f"    t={t:6.0f}s   {a:5.2f} -> {b:5.2f} m")
    else:
        print(f"  no position-error step > {jump_thresh:.1f} m in the fused "
              f"filter.")
    print()

    if verdict:
        a, b, r = verdict
        c1 = a > 0.5
        print("  pre-registered criteria (docs/experiments/PREREG_heading_drift.md)")
        print(f"    1. dead reckoning degrades, SLAM-free RMSE > 0.5 m : "
              f"{a:.2f} m  {'PASS' if c1 else 'FAIL -> run is VOID'}")
        if c1:
            print(f"    2. SLAM-aided beats it, ratio < 1.0               : "
                  f"{r:.2f}    {'PASS' if r < 1.0 else 'FAIL'}")
            print()
            print("  SLAM is load-bearing." if r < 1.0 else
                  "  SLAM is NOT load-bearing: fusing it makes the estimate "
                  f"{r:.1f}x worse.")
        else:
            print()
            print("  No drift, no test. The criterion that decides the "
                  "question was not reached.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--jump-thresh", type=float, default=0.5,
                    help="metres of position-error step to flag (default 0.5)")
    a = ap.parse_args()
    for b in a.bags:
        report(b, a.jump_thresh)


if __name__ == "__main__":
    main()
