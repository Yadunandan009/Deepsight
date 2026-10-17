#!/usr/bin/env python3
"""How do loop-closure checks hold up as dead reckoning drifts?

Three checks, scored on the 11 closures that are cleanly classified
(5 wrong-face, 6 benign; the one 'other-large' event is excluded because it
follows an earlier error in its own mission). A check FLAGS a closure as
suspect. A true positive is a wrong-face closure flagged; a false positive is a
benign closure flagged.

    size     flag if the correction is larger than BASE + 3 sigma_dr(t):
             too big to be drift. Needs dead reckoning's uncertainty.
    revisit  flag if dead reckoning says the vehicle has NOT been near this
             place before: nearest earlier dead-reckoned position is farther
             than BASE + 3 sigma_dr(t). Needs dead reckoning's track.
    chord    flag if the correction's length equals the chord of a symmetry
             operation of the orbit, 2 R sin(pi k / n). Needs NO dead
             reckoning at all: R and n come from the mission and the CAD.

WHY THE SWEEP. On a single lap dead reckoning is good to 0.2 m, so any check
has an enormous margin and they all look alike. The checks differ in what
happens when dead reckoning is NOT good. This injects drift into the true
track and asks where each check stops working.

DRIFT MODEL (an assumption, stated): the dead-reckoned position is the true
position plus D(t) = X * (s(t) / s_total) * u, growing linearly with distance
travelled to X metres at the end of the mission, in a random fixed direction
u (20 draws per mission). sigma_dr(t) = X * s(t) / s_total. A benign closure
then corrects the drift accumulated between the two visits, so its size is
sqrt(c0^2 + |D(t_c) - D(t_s)|^2), c0 being the recorded benign size. A
wrong-face closure's size is a property of SLAM's own frame and does not
depend on dead reckoning, so it is left as recorded.

FIXED BEFORE LOOKING AT RESULTS: BASE = 3 m (alignment and sonar tolerance),
k = 3, tau = 120 s, R = 17 m (the mission standoff), chord tolerance 10 %
(from the spread of SLAM scale, +-5 %, and of orbit radius, +-5 %).
CAVEAT: the chords of the five wrong-face closures had already been seen when
this was written, so the chord check is in-sample here, not a blind test.

Run:  python3 eval/gate_sweep.py [--refresh]
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
import json
import math

import numpy as np
from mcap_ros2.reader import read_ros2_messages

HERE = os.path.dirname(os.path.abspath(__file__))
EVENTS = os.path.join(HERE, "figures", "closures_data.json")
TRACKS = os.path.join(HERE, "figures", "gate_tracks.npz")
OUT = os.path.join(HERE, "figures", "gate_results.json")
BAGS = os.path.join(HERE, "..", "bags")

BASE, K_SIGMA, TAU = 3.0, 3.0, 120.0
R_ORBIT, N_FOLD, CHORD_TOL = 17.0, 4, 0.10
DRAWS = 20
SWEEP = [0, 0.3, 1, 2, 3, 5, 7, 10, 14, 18, 24, 30, 40]


def events():
    ev = json.load(open(EVENTS))["events"]
    return [e for e in ev if e["cls"] in ("face-aliased", "benign")]


def extract():
    names = sorted({e["bag"] for e in events()})
    out = {}
    for b in names:
        trk = {"odometry": [], "odometry/dr": []}
        for f in sorted(glob.glob(os.path.join(BAGS, b, "*.mcap"))):
            for m in read_ros2_messages(f, topics=["/bluerov2/odometry",
                                                   "/bluerov2/odometry/dr"]):
                t = (m.log_time.timestamp() if hasattr(m.log_time, "timestamp")
                     else m.log_time / 1e9)
                key = m.channel.topic.replace("/bluerov2/", "")
                p = m.ros_msg.pose.pose.position
                trk[key].append((t, p.x, p.y))
        for key, rows in trk.items():
            if not rows:
                continue
            a = np.array(rows)
            a[:, 0] -= trk["odometry"][0][0]
            keep, nxt = [], -1e9
            for i, t in enumerate(a[:, 0]):             # 1 Hz is plenty
                if t >= nxt:
                    keep.append(i)
                    nxt = t + 1.0
            out[f"{b}|{key}"] = a[keep]
        print("read", b, flush=True)
    np.savez(TRACKS, **out)


def nearest_earlier(track_xy, t, tc):
    """Distance from the position at tc to the nearest position >= TAU earlier."""
    i = int(np.argmin(np.abs(t - tc)))
    old = t <= tc - TAU
    if not old.any():
        return float("nan")
    return float(np.min(np.hypot(*(track_xy[old] - track_xy[i]).T)))


def chord_flag(c):
    chords = [2 * R_ORBIT * math.sin(math.pi * k / N_FOLD)
              for k in range(1, N_FOLD // 2 + 1)]
    return any(abs(c - x) <= CHORD_TOL * x for x in chords)


def run():
    tr = np.load(TRACKS)
    ev = events()
    rng = np.random.default_rng(0)
    prep = []
    for e in ev:
        a = tr[f"{e['bag']}|odometry"]
        t, xy = a[:, 0], a[:, 1:3]
        s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))])
        i = int(np.argmin(np.abs(t - e["t"])))
        old = np.flatnonzero(t <= e["t"] - TAU)
        j = old[int(np.argmin(np.hypot(*(xy[old] - xy[i]).T)))]
        prep.append(dict(e=e, t=t, xy=xy, s=s, i=i, j=j, tot=s[-1],
                         aliased=e["cls"] == "face-aliased",
                         d_truth=nearest_earlier(xy, t, e["t"])))
    # ---- the real dead-reckoning filter, where it was recorded
    real = []
    for p in prep:
        key = f"{p['e']['bag']}|odometry/dr"
        if key in tr.files:
            a = tr[key]
            err = float(np.hypot(*(a[np.argmin(np.abs(a[:, 0] - p["e"]["t"])), 1:3]
                                   - p["xy"][p["i"]])))
            real.append(dict(bag=p["e"]["bag"], cls=p["e"]["cls"], t=p["e"]["t"],
                             d_revisit_truth=p["d_truth"],
                             d_revisit_dr=nearest_earlier(a[:, 1:3], a[:, 0], p["e"]["t"]),
                             dr_error_at_closure=err, chord=p["e"]["chord"]))
    # ---- the sweep
    table = []
    for X in SWEEP:
        cnt = {g: {"tp": 0, "fp": 0, "na": 0, "nb": 0}
               for g in ("size", "revisit", "chord")}
        for p in prep:
            for _ in range(DRAWS):
                u = rng.normal(size=2)
                u /= np.linalg.norm(u)
                D = (X * p["s"] / p["tot"])[:, None] * u
                pos = p["xy"] + D
                sig = X * p["s"][p["i"]] / p["tot"]
                thr = BASE + K_SIGMA * sig
                c0 = p["e"]["chord"]
                size = c0 if p["aliased"] else math.hypot(
                    c0, float(np.hypot(*(D[p["i"]] - D[p["j"]]))))
                old = p["t"] <= p["e"]["t"] - TAU
                d_rev = float(np.min(np.hypot(*(pos[old] - pos[p["i"]]).T)))
                flags = {"size": size > thr, "revisit": d_rev > thr,
                         "chord": chord_flag(size)}
                for g, f in flags.items():
                    c = cnt[g]
                    if p["aliased"]:
                        c["na"] += 1
                        c["tp"] += int(f)
                    else:
                        c["nb"] += 1
                        c["fp"] += int(f)
        table.append(dict(X=X, **{g: dict(tpr=c["tp"] / c["na"], fpr=c["fp"] / c["nb"])
                                  for g, c in cnt.items()}))
    res = dict(params=dict(BASE=BASE, K=K_SIGMA, TAU=TAU, R=R_ORBIT, N=N_FOLD,
                           TOL=CHORD_TOL, DRAWS=DRAWS),
               n_aliased=sum(p["aliased"] for p in prep),
               n_benign=sum(not p["aliased"] for p in prep),
               truth=[dict(bag=p["e"]["bag"], cls=p["e"]["cls"],
                           d_revisit_truth=p["d_truth"], chord=p["e"]["chord"])
                      for p in prep],
               real_dr=real, sweep=table)
    json.dump(res, open(OUT, "w"), indent=1)
    return res


def show(res):
    print(f"closures scored: {res['n_aliased']} wrong-face, {res['n_benign']} benign\n")
    print("1. nearest earlier visit, from the TRUE track (reproduces the earlier check):")
    for t in res["truth"]:
        print(f"   {t['bag'][-26:]:26s} {t['cls']:13s} {t['d_revisit_truth']:6.1f} m   correction {t['chord']:5.1f} m")
    if res["real_dr"]:
        print("\n2. the same, from the recorded SLAM-free dead-reckoning filter:")
        for r in res["real_dr"]:
            print(f"   {r['bag'][-26:]:26s} {r['cls']:13s} {r['d_revisit_dr']:6.1f} m  "
                  f"(truth {r['d_revisit_truth']:.1f} m; dead-reckoning error there {r['dr_error_at_closure']:.2f} m)")
    print("\n3. sweep: dead-reckoning drift at the end of the mission  X  (m)")
    print(f"   {'X':>4s} | {'size':^15s} | {'revisit':^15s} | {'chord':^15s}")
    print(f"   {'':>4s} | {'caught  false':^15s} | {'caught  false':^15s} | {'caught  false':^15s}")
    for r in res["sweep"]:
        print(f"   {r['X']:4g} | " + " | ".join(
            f"{100 * r[g]['tpr']:4.0f}%  {100 * r[g]['fpr']:4.0f}%  " for g in ("size", "revisit", "chord")))


if __name__ == "__main__":
    if "--refresh" in sys.argv or not os.path.exists(TRACKS):
        extract()
    res = run()
    show(res)
