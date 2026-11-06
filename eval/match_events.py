#!/usr/bin/env python3
"""Classify ORB-SLAM3 re-localisations and map merges the way closure_events
classifies loop closures.

The census of wrong-face closures (closure_events.py) reads only `Loop
detected` lines. ORB-SLAM3 can also put the estimate on another face without
a loop closure:

    reloc   `Relocalized!!`   after tracking is lost, the tracker matches the
                              current frame to a keyframe of the same map
    merge   `Merge finished!` the map started after a loss is welded into an
                              earlier map

eval/heading_check.py found several of these with SLAM-minus-IMU heading
steps near 90 deg. This scores each one by position against ground truth.

Method, per event, with closure_events' constants and fit:

  1. reference alignment: a similarity SLAM->world fitted on the poses of the
     map the event lands in, before that map was left. For a re-localisation
     that is the PRE_FIT_S seconds before the event, clipped at the previous
     logged event; for a merge it is the PRE_FIT_S seconds before the newer
     map was created, i.e. the last stretch of the map it is merged into
     (assumed to be the map active just before the newer one);
  2. the first POST_S seconds of poses after the event are put into the world
     by that alignment and compared with the true position: the error, the
     change of radius about the structure axis and the turn of bearing about
     it;
  3. the same rule: error < BENIGN_CHORD -> benign; radius kept and turn
     within TURN_TOL of 90 or 180 -> face-aliased; otherwise other-large; a
     reference fit that is too short, narrow or poor -> unscorable;
  4. the heading test: the SLAM-minus-IMU heading after the event against its
     value at the end of the reference stretch.

Loop closures are run through the same code as a check: they must come out
in the class closure_events gives them.

Ground truth enters only to score.

Usage:  python3 eval/match_events.py [BAG ...]
        (default: every bag under the external drive's bags/ holding the raw
         SLAM pose and the IMU; writes eval/figures/match_events.json)
"""
import glob
import json
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import closure_events as ce  # noqa: E402
import heading_check as hc   # noqa: E402

POST_S = 5.0     # s of poses after the event that are scored
HEAD_S = 2.0     # s at the end of the reference stretch for the heading

MARKS = (("loop", "Loop detected"), ("reloc", "Relocalized"),
         ("merge", "Merge finished"), ("new map", "Creation of new map with id"))


def log_marks(path):
    out, ts = [], None
    for ln in open(path, errors="replace"):
        m = ce._TS.search(ln)
        if m:
            ts = float(m.group(1))
        for kind, key in MARKS:
            if key in ln and ts is not None:
                if kind == "new map" and re.search(r"with id: 0\b", ln):
                    continue
                out.append((ts, kind))
    return out


def score(sl, tr, hd, ref_lo, ref_hi, post_lo, post_hi):
    """Reference fit on sl in [ref_lo, ref_hi]; score poses in (post_lo, post_hi]."""
    pre = (sl[:, 0] >= ref_lo) & (sl[:, 0] <= ref_hi)
    row = {}
    S = sl[pre, 1:3]
    if pre.sum() < ce.MIN_FIT_POSES:
        return dict(cls="unscorable", why=f"only {pre.sum()} reference poses")
    if max(np.ptp(S[:, 0]), np.ptp(S[:, 1])) < ce.MIN_FIT_SPREAD:
        return dict(cls="unscorable", why="reference stretch too narrow")
    E = np.column_stack([np.interp(sl[pre, 0], tr[:, 0], tr[:, 1]),
                         np.interp(sl[pre, 0], tr[:, 0], tr[:, 2])])
    to_w, th, k, resid = ce.fit_similarity(S, E)
    row.update(resid=round(resid, 2), scale=round(k, 3))
    if resid > ce.MAX_FIT_RESID:
        row.update(cls="unscorable", why=f"reference fit residual {resid:.2f} m")
        return row
    post = (sl[:, 0] > post_lo) & (sl[:, 0] <= post_hi)
    if not post.any():
        row.update(cls="unscorable", why="no poses after the event")
        return row
    P = np.array([to_w(p) for p in sl[post, 1:3]])
    T = np.column_stack([np.interp(sl[post, 0], tr[:, 0], tr[:, 1]),
                         np.interp(sl[post, 0], tr[:, 0], tr[:, 2])])
    est, true = np.median(P, 0), np.median(T, 0)
    err = float(np.hypot(*(est - true)))
    re_, be = ce.polar(est)
    rt, bt = ce.polar(true)
    turn = ce.wrap_deg(be - bt)
    dr = (re_ - rt) / rt
    th_, d = hd
    h_ref = d[(th_ > ref_hi - HEAD_S) & (th_ <= ref_hi)]
    h_post = d[(th_ > post_lo) & (th_ <= post_hi)]
    hstep = (round(ce.wrap_deg(float(np.median(h_post) - np.median(h_ref))), 1)
             if len(h_ref) and len(h_post) else None)
    row.update(err=round(err, 2), dr=round(dr, 3), turn=round(turn, 1), heading_step=hstep,
               n_post=int(post.sum()))
    if err < ce.BENIGN_CHORD:
        row["cls"] = "benign"
    else:
        m = min((90.0, 180.0), key=lambda v: abs(abs(turn) - v))
        if abs(dr) <= ce.RADIUS_TOL and abs(abs(turn) - m) <= ce.TURN_TOL:
            row.update(cls="face-aliased", mult=m)
        else:
            row["cls"] = "other-large"
    return row


def check(bag):
    log = ce.pick_log(bag)
    if not log:
        return None
    sl = ce.track(bag, "/bluerov2/robot_pose_slam", True)
    tr = ce.track(bag, "/bluerov2/odometry", False)
    hd = hc.load(bag)
    t0 = tr[0, 0]
    marks = log_marks(log)
    step = np.hypot(np.diff(sl[:, 1]), np.diff(sl[:, 2]))
    rows = []
    for i, (te, kind) in enumerate(marks):
        if kind == "new map":
            continue
        prev = [m for m in marks[:i] if m[0] < te]
        clip = prev[-1][0] + 1.0 if prev else -1e18
        if kind == "loop":
            # where closure_events puts it: the largest raw step after the line
            w = np.flatnonzero((sl[:-1, 0] >= te - ce.MARGIN_LO) & (sl[:-1, 0] <= te + ce.MARGIN_HI))
            if not len(w):
                rows.append(dict(t=round(te - t0, 1), kind=kind, cls="unscorable", why="no poses"))
                continue
            j = int(w[np.argmax(step[w])])
            tj = sl[j, 0]
            r = score(sl, tr, hd, max(tj - ce.PRE_FIT_S, clip), tj, tj, tj + POST_S)
        elif kind == "reloc":
            # the map was left at the last pose before the event; the poses
            # after the event are in that same map
            before = sl[sl[:, 0] < te - 0.05, 0]
            if not len(before):
                continue
            last = before[-1]
            r = score(sl, tr, hd, max(last - ce.PRE_FIT_S, clip), last, te - 0.05, te + POST_S)
        else:   # merge: into the map that was active before the newest map
            news = [m[0] for m in marks[:i] if m[1] == "new map" and m[0] < te]
            if not news:
                rows.append(dict(t=round(te - t0, 1), kind=kind, cls="unscorable",
                                 why="no newer map to merge"))
                continue
            left = news[-1]
            earlier = [m[0] for m in marks if m[0] < left - 1.0]
            clip2 = earlier[-1] + 1.0 if earlier else -1e18
            r = score(sl, tr, hd, max(left - ce.PRE_FIT_S, clip2), left, te, te + POST_S)
        r.update(t=round(te - t0, 1), kind=kind)
        rows.append(r)
    return dict(bag=os.path.basename(bag.rstrip("/")), log=os.path.basename(log), events=rows)


def main():
    bags = sys.argv[1:] or [b for b in sorted(glob.glob(os.path.join(hc.BAG_ROOT, "*")))
                            if os.path.exists(os.path.join(b, "metadata.yaml"))
                            and all(f"name: {k}\n" in open(os.path.join(b, "metadata.yaml")).read()
                                    for k in ("/bluerov2/robot_pose_slam", "/bluerov2/imu"))]
    res = []
    for bag in bags:
        r = check(bag)
        if r:
            res.append(r)
            print(json.dumps(r))
    json.dump(res, open(os.path.join(HERE, "figures", "match_events.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
