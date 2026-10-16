#!/usr/bin/env python3
"""Classify every ORB-SLAM3 loop closure in a run.

A FACE CONFUSION on a four-fold symmetric structure is a rotation of the
estimate about the structure's axis by 90 or 180 deg. It has two
signatures and both must hold (see docs/LOOP_CLOSURE_FACE_CONFUSION.md):

    radius from the axis preserved     |dr| / r <= RADIUS_TOL
    bearing about the axis turns by    |turn| within TURN_TOL of 90 or 180

How an event is located, with nothing hand-picked:

  1. the console log's `*Loop detected` line, bracketed by the nearest
     timestamped lines either side, and the `Local Mapping RELEASE` that
     follows it, give a wall-clock interval [t_lo, t_hi];
  2. the largest consecutive step of the raw SLAM track inside
     [t_lo - 3 s, t_hi + 30 s] is the correction (the map is re-posed when
     the fusion thread finishes, which is after RELEASE, hence the margin);
  3. a similarity SLAM->world is fitted on the preceding PRE_FIT_S seconds
     of poses ONLY -- clipped at any earlier event -- so the "after" point
     is placed by a transform that knew nothing about the jump.

Each event ends in exactly one class:

    benign          chord < BENIGN_CHORD m: a small drift correction
    face-aliased    large, radius kept, turn ~ 90/180
    other-large     large but not a symmetry operation
    unscorable      the pre-event fit is too short, too narrow or too poor
                    to place the before/after points; COUNTED, never
                    dropped, because dropping them would bias the aliased
                    fraction in whichever direction the fit happens to fail

Ground truth enters only to score. Nothing here feeds navigation.

Usage:  closure_events.py BAG [LOG]   (LOG defaults to the console log whose
                                        start time is nearest the bag's)
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

import glob
import math
import re

import numpy as np
from mcap_ros2.reader import read_ros2_messages

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from slamio import slam_to_ned_xy

# --- the rule. Registered in docs/experiments/PREREG_closure_events.md;
# change these only by amending that document first.
TURBINE = (20.0, 0.0)
BENIGN_CHORD = 3.0     # m   below this a closure is a drift correction
RADIUS_TOL = 0.10      # fraction of r
TURN_TOL = 15.0        # deg about 90 or 180
PRE_FIT_S = 150.0      # s of poses used to place the before/after points
MIN_FIT_POSES = 100
MIN_FIT_SPREAD = 5.0   # m
MAX_FIT_RESID = 1.5    # m
MARGIN_LO, MARGIN_HI = 3.0, 30.0

_TS = re.compile(r"\[(\d{10}\.\d+)\]")


def log_events(path):
    """`Loop detected` events with the interval they must lie in."""
    lines = open(path, errors="replace").read().splitlines()
    ts = [None] * len(lines)
    for i, ln in enumerate(lines):
        m = _TS.search(ln)
        if m:
            ts[i] = float(m.group(1))

    def prev_ts(i):
        for k in range(i, -1, -1):
            if ts[k] is not None:
                return ts[k]

    def next_ts(i):
        for k in range(i, len(lines)):
            if ts[k] is not None:
                return ts[k]

    out = []
    for i, ln in enumerate(lines):
        if "Loop detected" not in ln:
            continue
        rel = next((k for k in range(i, min(i + 200, len(lines)))
                    if "Local Mapping RELEASE" in lines[k]), i)
        out.append((prev_ts(i), next_ts(rel + 1) or next_ts(rel)))
    return out


def pick_log(bag):
    m = re.search(r"(\d{8})_(\d{6})", os.path.basename(os.path.normpath(bag)))
    if not m:
        sys.exit("cannot infer the bag's start time from its name")
    key = int(m.group(1)) * 1_000_000 + int(m.group(2))
    best = None
    for p in glob.glob(os.path.join(os.path.dirname(__file__), "..",
                                    "slam_atlas", "orb_slam3_console_*.log")):
        n = re.search(r"(\d{8})_(\d{6})", p)
        k = int(n.group(1)) * 1_000_000 + int(n.group(2))
        # the console log opens a few seconds BEFORE the recorder does
        if 0 <= key - k <= 300 and (best is None or key - k < best[0]):
            best = (key - k, p)
    return best[1] if best else None


def track(bag, topic, slam):
    r = []
    for f in sorted(glob.glob(os.path.join(bag, "*.mcap"))):
        for m in read_ros2_messages(f, topics=[topic]):
            t = (m.log_time.timestamp() if hasattr(m.log_time, "timestamp")
                 else m.log_time / 1e9)
            ps = m.ros_msg.pose
            if slam:
                x, y = slam_to_ned_xy(ps.position.x, ps.position.y)
            else:
                x, y = ps.pose.position.x, ps.pose.position.y
            r.append((t, x, y))
    return np.array(r)


def fit_similarity(S, E):
    Sm, Em = S.mean(0), E.mean(0)
    s0, e0 = S - Sm, E - Em
    num = (s0[:, 0] * e0[:, 1] - s0[:, 1] * e0[:, 0]).sum()
    den = (s0[:, 0] * e0[:, 0] + s0[:, 1] * e0[:, 1]).sum()
    th = math.atan2(num, den)
    k = math.hypot(num, den) / (s0 ** 2).sum()
    c, s = math.cos(th), math.sin(th)
    R = np.array([[c, -s], [s, c]])
    resid = float(np.sqrt((((k * (s0 @ R.T)) - e0) ** 2).sum(1).mean()))
    return (lambda p: k * (R @ (np.asarray(p) - Sm)) + Em), th, k, resid


def polar(p):
    d = np.asarray(p) - np.array(TURBINE)
    return math.hypot(*d), math.degrees(math.atan2(d[1], d[0]))


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def classify(bag, log):
    sl, tr = track(bag, "/bluerov2/robot_pose_slam", True), \
        track(bag, "/bluerov2/odometry", False)
    t0 = tr[0, 0]
    events = log_events(log)
    rows, prev_step_t = [], -1e18
    for (lo, hi) in events:
        w = (sl[:-1, 0] >= lo - MARGIN_LO) & (sl[:-1, 0] <= hi + MARGIN_HI)
        if not w.any():
            rows.append(dict(t=lo - t0, cls="unscorable",
                             why="no SLAM samples in the event interval"))
            continue
        step = np.hypot(np.diff(sl[:, 1]), np.diff(sl[:, 2]))
        idx = np.flatnonzero(w)
        j = int(idx[np.argmax(step[idx])])
        tj = sl[j, 0]
        pre = (sl[:, 0] < tj) & (sl[:, 0] >= max(tj - PRE_FIT_S, prev_step_t))
        row = dict(t=tj - t0, raw_step=float(step[j]))
        S = sl[pre, 1:3]
        if pre.sum() < MIN_FIT_POSES:
            row.update(cls="unscorable", why=f"only {pre.sum()} pre-event poses")
        elif max(np.ptp(S[:, 0]), np.ptp(S[:, 1])) < MIN_FIT_SPREAD:
            row.update(cls="unscorable", why="pre-event window too narrow")
        else:
            E = np.column_stack([np.interp(sl[pre, 0], tr[:, 0], tr[:, 1]),
                                 np.interp(sl[pre, 0], tr[:, 0], tr[:, 2])])
            to_w, th, k, resid = fit_similarity(S, E)
            row.update(resid=resid, scale=k)
            if resid > MAX_FIT_RESID:
                row.update(cls="unscorable", why=f"pre-event fit residual {resid:.2f} m")
            else:
                b, a = to_w(sl[j, 1:3]), to_w(sl[j + 1, 1:3])
                chord = float(np.hypot(*(a - b)))
                rb, bb = polar(b)
                ra, ba = polar(a)
                turn = wrap_deg(ba - bb)
                dr = (ra - rb) / rb
                row.update(chord=chord, r_before=rb, dr=dr, turn=turn,
                           bearing_before=bb, bearing_after=ba)
                if chord < BENIGN_CHORD:
                    row["cls"] = "benign"
                else:
                    m = min((90.0, 180.0), key=lambda v: abs(abs(turn) - v))
                    off = abs(abs(turn) - m)
                    if abs(dr) <= RADIUS_TOL and off <= TURN_TOL:
                        row.update(cls="face-aliased", mult=m, off=off)
                    else:
                        row["cls"] = "other-large"
        prev_step_t = tj + 1.0
        rows.append(row)
    return rows, len(events)


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    bag = sys.argv[1]
    log = sys.argv[2] if len(sys.argv) > 2 else pick_log(bag)
    print("=" * 74)
    print(f"LOOP CLOSURES: {os.path.basename(os.path.normpath(bag))}")
    print(f"  console log : {os.path.basename(log) if log else 'NONE FOUND'}")
    if not log:
        sys.exit("  no console log matches this bag; cannot locate events")
    rows, n = classify(bag, log)
    print(f"  `Loop detected` lines: {n}")
    for r in rows:
        print()
        print(f"  t = {r['t']:7.0f}s   raw step {r.get('raw_step', float('nan')):6.2f}   "
              f"-> {r['cls'].upper()}")
        if r["cls"] == "unscorable":
            print(f"      why: {r['why']}")
            continue
        print(f"      pre-fit: scale {r['scale']:.3f}  residual {r['resid']:.2f} m")
        print(f"      chord {r['chord']:6.2f} m   radius {r['r_before']:.1f} m "
              f"({100 * r['dr']:+.1f} %)   bearing turn {r['turn']:+7.1f} deg"
              f"   (about the axis, from {r['bearing_before']:+.0f})")
        if r["cls"] == "face-aliased":
            print(f"      nearest symmetry op {r['mult']:.0f} deg, off by {r['off']:.1f}")
    print("=" * 74)


if __name__ == "__main__":
    main()
