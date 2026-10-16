#!/usr/bin/env python3
"""Is there a fixed map->world transform for the bridge to find at all?

`slam_pose_bridge` assumes one exists: it fits a constant (theta, tx, ty) and
re-fits it when it goes stale. `lock_conditioning.py` showed the fits are not
ill-conditioned -- each window pins theta to under half a degree with a
near-perfect rigid fit -- and yet consecutive windows disagree by ~20 deg and
theta covers the circle. Sharp fits that sharply disagree mean the model is
wrong, not the trigger.

This tests the model directly, without the bridge in the loop:

  continuity   is the raw SLAM track continuous, or is it being re-founded?
               Re-founding would explain non-rigidity trivially and would
               call for a different fix.
  scale        the bridge fits rotation only, on the stated grounds that
               stereo SLAM's metric scale is correct. Similarity Procrustes
               recovers the scale a rotation-only fit is forced to absorb,
               locally and over the whole run.
  path length  scale-sensitive and needs no alignment at all, so it cannot be
               explained away by a bad fit.
  best case    the residual left by the best similarity transform over the
               whole run. This is the floor for ANY global rigid or scaled
               alignment: no trigger, gate or conditioning test can beat it.

Ground truth is used here only to characterise SLAM. Nothing in the
navigation pipeline sees it.

Usage:  slam_track_fidelity.py BAG [BAG ...]
"""
import glob
import math
import os
import sys

import numpy as np
from mcap_ros2.reader import read_ros2_messages

# The SLAM pose arrives in ENU; this workspace is NED. Without this the
# tool measures the unconverted track -- see eval/slamio.py.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from slamio import slam_to_ned_xy, NOTE

T_SLAM = "/bluerov2/robot_pose_slam"
T_REF = "/bluerov2/odometry/dr"
T_TRUTH = "/bluerov2/odometry"
WIN_S, STEP_S, MIN_BASELINE = 40.0, 10.0, 5.0


class _Ned:
    """A .x/.y view of a SLAM position, converted to NED."""
    __slots__ = ('x', 'y', 'z')

    def __init__(self, p):
        self.x, self.y = slam_to_ned_xy(p.x, p.y)
        self.z = -p.z


def grab(bag, topic, pose):
    rows = []
    for f in sorted(glob.glob(os.path.join(bag, "*.mcap"))):
        for m in read_ros2_messages(f, topics=[topic]):
            s = m.ros_msg.header.stamp
            p = pose(m.ros_msg)
            rows.append((s.sec + s.nanosec * 1e-9, p.x, p.y))
    return np.array(rows) if rows else np.empty((0, 3))


def sim_fit(S, E):
    """Similarity Procrustes: the scale a rotation-only fit has to absorb."""
    Sc, Ec = S - S.mean(0), E - E.mean(0)
    num = float((Sc[:, 0] * Ec[:, 1] - Sc[:, 1] * Ec[:, 0]).sum())
    den = float((Sc[:, 0] * Ec[:, 0] + Sc[:, 1] * Ec[:, 1]).sum())
    ss = float((Sc ** 2).sum())
    if ss <= 0:
        return float("nan"), float("nan"), float("nan"), float("nan")
    k = math.hypot(num, den) / ss
    th = math.atan2(num, den)
    c, s_ = math.cos(th), math.sin(th)
    R = np.array([[c, -s_], [s_, c]])
    rot = math.sqrt(float(((Ec - Sc @ R.T) ** 2).sum()) / len(Sc))
    sim = math.sqrt(float(((Ec - k * (Sc @ R.T)) ** 2).sum()) / len(Sc))
    return k, math.degrees(th), rot, sim


def path_len(a):
    return float(np.hypot(np.diff(a[:, 0]), np.diff(a[:, 1])).sum())


def main():
    bags = sys.argv[1:]
    if not bags:
        sys.exit(__doc__)

    for bag in bags:
        # Converted on read: a rotation fitted to the published (ENU)
        # track can never succeed, which is what every residual in
        # this file reported before 2026-10-16.
        raw = grab(bag, T_SLAM, lambda m: _Ned(m.pose.position))
        ref = grab(bag, T_REF, lambda m: m.pose.pose.position)
        tru = grab(bag, T_TRUTH, lambda m: m.pose.pose.position)
        if len(raw) < 100 or len(tru) < 100:
            print(f"{os.path.basename(bag)}: not enough data")
            continue

        at = lambda src: np.column_stack(
            [np.interp(raw[:, 0], src[:, 0], src[:, 1]),
             np.interp(raw[:, 0], src[:, 0], src[:, 2])])
        T = at(tru)
        E = at(ref) if len(ref) > 100 else T

        print(f"\n{'=' * 74}\n{os.path.basename(bag)}\n{'=' * 74}")
        print(f"  {NOTE}")

        d = np.hypot(np.diff(raw[:, 1]), np.diff(raw[:, 2]))
        print(f"  continuity: {len(raw)} poses, step median "
              f"{np.median(d) * 100:.1f} cm, max {d.max():.2f} m, "
              f"{(d > 2.0).sum()} steps over 2 m")
        print("    -> the track is continuous; it is not being re-founded"
              if (d > 2.0).sum() == 0 else
              "    -> the track has large discontinuities; it IS being re-founded")

        pr, pt = path_len(raw[:, 1:3]), path_len(T)
        print(f"  path length: SLAM {pr:.0f} m vs truth {pt:.0f} m "
              f"(ratio {pr / pt:.2f})")

        ks = []
        t = raw[0, 0]
        while t + WIN_S <= raw[-1, 0]:
            w = (raw[:, 0] >= t) & (raw[:, 0] < t + WIN_S)
            if w.sum() >= 30:
                Sc = raw[w, 1:3] - raw[w, 1:3].mean(0)
                if math.hypot(np.ptp(Sc[:, 0]), np.ptp(Sc[:, 1])) >= MIN_BASELINE:
                    ks.append(sim_fit(raw[w, 1:3], E[w])[0])
            t += STEP_S
        ks = np.array(ks)
        if len(ks):
            print(f"  local scale over {WIN_S:.0f} s windows ({len(ks)}): "
                  f"median {np.median(ks):.2f}  "
                  f"[p10 {np.percentile(ks, 10):.2f}, "
                  f"p90 {np.percentile(ks, 90):.2f}]")

        for label, X in (("SLAM-free reference", E), ("ground truth", T)):
            k, th, rot, sim = sim_fit(raw[:, 1:3], X)
            print(f"  best GLOBAL fit to {label}: scale {k:.2f}, "
                  f"theta {th:+.0f} deg")
            print(f"    residual: rotation-only {rot:.1f} m, "
                  f"with scale {sim:.1f} m")
        print(f"  for scale, the true track spans "
              f"{np.ptp(T[:, 0]):.0f} x {np.ptp(T[:, 1]):.0f} m")

    print(f"\n{'=' * 74}")
    print("  A global residual comparable to the size of the trajectory means")
    print("  no fixed transform exists. The bridge's constant-(theta,t) model")
    print("  is then the wrong model, and no change to WHEN it fits can help.")


if __name__ == "__main__":
    main()
