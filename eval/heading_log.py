#!/usr/bin/env python3
"""Read the in-loop heading check's decisions from a flight and set them
against closure_events' classes.

ORB-SLAM3 with HeadingPrior (Heading.Mode 1 or 2) prints one line per loop
closure, map merge and relocalization it is about to accept:

    [HEADING] kind=loop t_a=... t_m=... kf_a=... kf_m=... vis=... imu=...
              resid=... thresh=45.0 decision=accept|would_veto|veto|no_imu

This lists them, and for each loop closure closure_events classifies (from
the same bag and console log) finds the decision whose t_a falls in
[jump - 10 s, jump + 2 s] -- the matched keyframe is the one the loop is
accepted on, a little before the pose jump reaches the published pose.

Pass criterion for the log-only flight (step 3): every benign closure
accepted, every face-aliased one would_veto, and the matched keyframe ages
(t_a - t_m) like those match_gap.py estimated.

Usage:  python3 eval/heading_log.py BAG
"""
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import closure_events as ce  # noqa: E402

LINE = re.compile(r"\[HEADING\] kind=(\w+) t_a=([\d.]+) t_m=([\d.]+) kf_a=(\d+) kf_m=(\d+) "
                  r"vis=(-?[\d.]+) imu=(-?[\d.]+) resid=(-?[\d.]+) thresh=([\d.]+) decision=(\w+)")
PAIR = (-10.0, 2.0)


def decisions(log):
    out = []
    for ln in open(log, errors="replace"):
        m = LINE.search(ln)
        if m:
            k, ta, tm, ka, km, vis, imu, res, th, dec = m.groups()
            out.append(dict(kind=k, t_a=float(ta), t_m=float(tm), kf_a=int(ka), kf_m=int(km),
                            vis=float(vis), imu=float(imu), resid=float(res), decision=dec))
    return out


def main():
    bag = sys.argv[1]
    log = ce.pick_log(bag)
    if not log:
        sys.exit(f"no console log found for {bag}")
    dec = decisions(log)
    t0 = ce.track(bag, "/bluerov2/odometry", False)[0, 0]
    print(f"log: {os.path.basename(log)}   {len(dec)} [HEADING] decisions")
    for d in dec:
        print(f"  {d['kind']:5s} t={d['t_a'] - t0:7.1f}  age={d['t_a'] - d['t_m']:6.1f} s  "
              f"vis={d['vis']:7.1f} imu={d['imu']:7.1f} resid={d['resid']:7.1f}  {d['decision']}")
    rows, _ = ce.classify(bag, log)
    print("closure_events classes vs decisions:")
    out = []
    for r in rows:
        tj = t0 + r["t"]
        hit = [d for d in dec if d["kind"] == "loop" and PAIR[0] <= d["t_a"] - tj <= PAIR[1]]
        row = dict(t=round(r["t"], 1), cls=r["cls"],
                   decision=[(h["decision"], h["resid"], round(h["t_a"] - h["t_m"], 1)) for h in hit])
        out.append(row)
        print(f"  {row['t']:7.1f}  {row['cls']:12s}  {row['decision'] or 'NO DECISION LOGGED'}")
    json.dump(dict(bag=os.path.basename(bag.rstrip("/")), decisions=dec, classes=out),
              open(os.path.join(HERE, "figures", "heading_log_" + os.path.basename(bag.rstrip("/")) + ".json"), "w"),
              indent=1)


if __name__ == "__main__":
    main()
