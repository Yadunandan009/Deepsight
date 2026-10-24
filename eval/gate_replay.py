#!/usr/bin/env python3
"""What would a narrower or adaptive consistency gate have rejected? Offline replay.

Reads eval/figures/gate_plateau_series.npz (written by eval/gate_plateau.py): for every
pose the bridge published, its distance from the SLAM-free reference filter (bdr: the
quantity the bridge's gate tests) and from the true position (btr). A published pose is
BAD if it is more than 1 m from the truth, GOOD otherwise. The path length the
reference travelled is read from /bluerov2/odometry/dr in each bag (cached in
eval/figures/gate_replay_path.npz).

Each candidate gate (stonefish_bluerov2/consistency_gate.py) is replayed on the published
stream, open loop. Probation is approximated: a silence of more than 2 s in the
published stream is taken as a re-lock, after which the floor width applies until 25
consecutive poses have passed (the spread condition is assumed met, which is optimistic).

PASS LINE, fixed before running: at least 95 % of BAD poses rejected and at most 0.5 % of
GOOD poses wrongly rejected, over the eleven flights.

LIMITS: (1) the published poses were already accepted by the 5 m gate, so poses it
rejected are invisible; (2) not closed loop: rejection would change the filter, the
path and later SLAM output; (3) on one lap the reference is good to under 0.7 m, so a
narrow gate looks free; the bridge refits its alignment to the reference every 5 s over
a 75 s window, so slow dead-reckoning drift is absorbed with a lag of tens of seconds
and a narrow gate does not by itself forbid following it; (4) 1 m for BAD is a choice.
Exploratory, not registered.

Run: python3 eval/gate_replay.py [--refresh]
"""
import csv
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "src", "stonefish_bluerov2"))
from stonefish_bluerov2.consistency_gate import ConsistencyGate  # noqa: E402

S = np.load(os.path.join(HERE, "figures", "gate_plateau_series.npz"), allow_pickle=True)
FL = ["slamref_s0", "slamref_s1", "slamref_s2", "L4_s0", "L4_s1", "L0_s0", "L5_s0",
      "L0_s1", "L5_s1", "L0_s2", "L5_s2"]
T_BAD = 1.0
PATH_CACHE = os.path.join(HERE, "figures", "gate_replay_path.npz")


def bag_of(fid):
    with open(os.path.join(ROOT, "docs", "experiments", "flights_manifest.csv")) as fh:
        for r in csv.DictReader(fh):
            if r["id"] == fid:
                return os.path.join(ROOT, r["bag"])
    raise KeyError(fid)


def reference_path(fid):
    """(t, s): seconds from the first truth sample, cumulative reference path (m)."""
    from mcap_ros2.reader import read_ros2_messages
    bag = bag_of(fid)
    t0 = None
    for f in sorted(glob.glob(os.path.join(bag, "*.mcap"))):
        for m in read_ros2_messages(f, topics=["/bluerov2/odometry"]):
            t0 = m.log_time.timestamp() if hasattr(m.log_time, "timestamp") else m.log_time / 1e9
            break
        break
    ts, xs, ys = [], [], []
    for f in sorted(glob.glob(os.path.join(bag, "*.mcap"))):
        for m in read_ros2_messages(f, topics=["/bluerov2/odometry/dr"]):
            t = m.log_time.timestamp() if hasattr(m.log_time, "timestamp") else m.log_time / 1e9
            ts.append(t - t0)
            xs.append(m.ros_msg.pose.pose.position.x)
            ys.append(m.ros_msg.pose.pose.position.y)
    ts, xs, ys = map(np.array, (ts, xs, ys))
    step = np.hypot(np.diff(xs), np.diff(ys))
    step[step >= 20.0] = 0.0
    return ts, np.concatenate([[0.0], np.cumsum(step)])


def paths(refresh):
    if os.path.exists(PATH_CACHE) and not refresh:
        c = np.load(PATH_CACHE, allow_pickle=True)
        return {k: (c[k + "_t"], c[k + "_s"]) for k in FL}
    out, save = {}, {}
    for k in FL:
        print("reading reference path", k, flush=True)
        out[k] = reference_path(k)
        save[k + "_t"], save[k + "_s"] = out[k]
    np.savez(PATH_CACHE, **save)
    return out


def replay(gate_kw, probation, P):
    """Return (bad, bad_rejected, good, good_rejected) over all flights."""
    nb = nbr = ng = ngr = 0
    for k in FL:
        tb, bdr, btr = S[k + "_tb"], S[k + "_bdr"], S[k + "_btr"]
        tp, sp = P[k]
        s = np.interp(tb, tp, sp)
        bad = btr > T_BAD
        g = ConsistencyGate(**gate_kw)
        rej = np.zeros(len(tb), bool)
        last = None
        passed = 0
        young = False
        for i in range(len(tb)):
            if last is not None and tb[i] - last > 2.0:     # silence: a re-lock
                young, passed = True, 0
            last = tb[i]
            width = min(g.cap, g.floor + g.k_drift * s[i])
            if probation and young:
                width = g.floor
            ok = bdr[i] <= width
            if probation and young:
                passed = passed + 1 if ok else 0
                if passed >= 25:
                    young = False
            rej[i] = not ok
        nb += int(bad.sum()); nbr += int((bad & rej).sum())
        ng += int((~bad).sum()); ngr += int(((~bad) & rej).sum())
    return nb, nbr, ng, ngr


CANDIDATES = [
    ("current: fixed 5 m", dict(floor=5.0, k_drift=0.0, cap=5.0), False),
    ("fixed 1.5 m", dict(floor=1.5, k_drift=0.0), False),
    ("fixed 1.2 m", dict(floor=1.2, k_drift=0.0), False),
    ("fixed 1.0 m", dict(floor=1.0, k_drift=0.0), False),
    ("1.0 m + 0.0015/m", dict(floor=1.0, k_drift=0.0015), False),
    ("1.0 m + 0.003/m", dict(floor=1.0, k_drift=0.003), False),
    ("1.0 m + 0.003/m, probation", dict(floor=1.0, k_drift=0.003), True),
    ("1.0 m, probation", dict(floor=1.0, k_drift=0.0), True),
]

if __name__ == "__main__":
    P = paths("--refresh" in sys.argv)
    res = []
    print(f"{'gate':32s} {'bad':>6s} {'rejected':>16s} {'good':>7s} {'wrongly rejected':>20s}  pass")
    for name, kw, prob in CANDIDATES:
        nb, nbr, ng, ngr = replay(kw, prob, P)
        a, b = 100 * nbr / nb, 100 * ngr / ng
        ok = a >= 95.0 and b <= 0.5
        res.append(dict(gate=name, params=kw, probation=prob, bad=nb, bad_rejected=nbr,
                        good=ng, good_rejected=ngr, passes=bool(ok)))
        print(f"{name:32s} {nb:6d} {nbr:7d} ({a:5.1f}%) {ng:7d} {ngr:7d} ({b:5.2f}%)  {'PASS' if ok else 'fail'}")
    json.dump(res, open(os.path.join(HERE, "figures", "gate_replay.json"), "w"), indent=1)
