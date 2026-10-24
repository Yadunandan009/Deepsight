#!/usr/bin/env python3
"""When does the fused error become large, and what happened just before?

For every flight in the manifest: the time of the first face-aliased closure, the
times at which ORB-SLAM3's map collapsed (the point count of /bluerov2/map_points
fell below half of its recent maximum, the bridge's own map-collapse criterion,
MAP_CRASH_RATIO 0.5 with a 2000-point floor), and the first times the primary
filter's error against truth exceeded 1 m and 2 m. Reads flights_summary.json and
flights_series.npz (flights_summary.py) and writes figures/jump_attribution.json.

The question it answers: is the large fused error at the end of a mission caused by
the wrong-face closure, or by a later failure of the SLAM map?
"""
import csv
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from mcap_ros2.reader import read_ros2_messages  # noqa: E402

OUT = os.path.join(HERE, "figures", "jump_attribution.json")


def map_sizes(bag, t0):
    out = []
    for f in sorted(glob.glob(os.path.join(bag, "*.mcap"))):
        for m in read_ros2_messages(f, topics=["/bluerov2/map_points"]):
            t = m.log_time.timestamp() if hasattr(m.log_time, "timestamp") else m.log_time / 1e9
            r = m.ros_msg
            out.append((t - t0, int(r.width) * int(r.height)))
    return np.array(out)


def collapses(ms, ratio=0.5, floor=2000, window=30.0):
    ev = []
    for i in range(1, len(ms)):
        recent = ms[(ms[:, 0] >= ms[i, 0] - window) & (ms[:, 0] < ms[i, 0])]
        if len(recent) and recent[:, 1].max() >= floor and ms[i, 1] < ratio * recent[:, 1].max():
            if not ev or ms[i, 0] - ev[-1] > window:
                ev.append(float(ms[i, 0]))
    return ev


import re
_TS = re.compile(r"\[(\d{10}\.\d+)\]")


def slam_events(log, t0):
    """Times (s into the mission) of events in the ORB-SLAM3 console log. Event lines
    carry no timestamp of their own, so each takes the time of the nearest stamped
    line before it. 'optimise' = a Local Mapping STOP, which ORB-SLAM3 issues for a
    loop correction and for a map merge alike; 'loop' = a `Loop detected` line;
    'lost' = the first line of each run of `Tracking LOST`; 'newmap' = `New Map created`."""
    lines = open(log, errors="replace").read().splitlines()
    ts = [None] * len(lines)
    last = None
    for i, ln in enumerate(lines):
        m = _TS.search(ln)
        if m:
            last = float(m.group(1))
        ts[i] = last
    ev = {"loop": [], "optimise": [], "lost": [], "newmap": []}
    in_lost = False
    for i, ln in enumerate(lines):
        t = ts[i]
        if t is None:
            continue
        if "Loop detected" in ln:
            ev["loop"].append(t - t0)
        elif "Local Mapping STOP" in ln:
            ev["optimise"].append(t - t0)
        elif "New Map created" in ln:
            ev["newmap"].append(t - t0)
        if "Tracking LOST" in ln:
            if not in_lost:
                ev["lost"].append(t - t0)
            in_lost = True
        elif "ORB-SLAM failed" not in ln and "Fail to track" not in ln:
            in_lost = False
    return {k: [round(x, 1) for x in v] for k, v in ev.items()}


def first_exceed(g, e, thr):
    i = np.flatnonzero(e > thr)
    return float(g[i[0]]) if len(i) else None


def main():
    S = np.load(os.path.join(HERE, "figures", "flights_series.npz"))
    F = {r["id"]: r for r in json.load(open(os.path.join(HERE, "figures", "flights_summary.json")))}
    flights = list(csv.DictReader(open(os.path.join(ROOT, "docs", "experiments", "flights_manifest.csv"))))
    res = json.load(open(OUT)) if os.path.exists(OUT) and "--refresh" not in sys.argv else {}
    from mcap.reader import make_reader
    for fl in flights:
        k = fl["id"]
        bag = os.path.join(ROOT, fl["bag"])
        if k in res or k not in F or not os.path.isdir(bag):
            continue
        print("reading map sizes", k, flush=True)
        # truth start time, to put the map series on the same clock as the 1 Hz series
        t0 = None
        for f in sorted(glob.glob(os.path.join(bag, "*.mcap"))):
            for m in read_ros2_messages(f, topics=["/bluerov2/odometry"]):
                t0 = m.log_time.timestamp() if hasattr(m.log_time, "timestamp") else m.log_time / 1e9
                break
            if t0:
                break
        ms = map_sizes(bag, t0)
        g, e = S[k + "_grid"], S[k + "_filt"]
        first_al = F[k]["first_aliased_t"]
        res[k] = dict(first_aliased_t=first_al, map_collapses=collapses(ms),
                      map_max=int(ms[:, 1].max()),
                      filt_over_1m=first_exceed(g, e, 1.0), filt_over_2m=first_exceed(g, e, 2.0),
                      duration=F[k]["duration_s"])
        json.dump(res, open(OUT, "w"), indent=1)
    import closure_events as CE
    for fl in flights:
        k = fl["id"]
        if k in res and "slam_events" not in res[k]:
            bag = os.path.join(ROOT, fl["bag"])
            t0 = None
            for f in sorted(glob.glob(os.path.join(bag, "*.mcap"))):
                for m in read_ros2_messages(f, topics=["/bluerov2/odometry"]):
                    t0 = m.log_time.timestamp() if hasattr(m.log_time, "timestamp") else m.log_time / 1e9
                    break
                if t0:
                    break
            lg = CE.pick_log(bag)
            res[k]["slam_events"] = slam_events(lg, t0) if lg else {}
            json.dump(res, open(OUT, "w"), indent=1)
    for k, v in res.items():
        print("%-11s closure %-6s  map collapses %s  err>1m %s  err>2m %s  of %.0f s" % (
            k, "%.0f" % v["first_aliased_t"] if v["first_aliased_t"] else "-",
            ["%.0f" % c for c in v["map_collapses"]],
            "%.0f" % v["filt_over_1m"] if v["filt_over_1m"] else "-",
            "%.0f" % v["filt_over_2m"] if v["filt_over_2m"] else "-", v["duration"]))
        e = v.get("slam_events", {})
        print("            loop %s  optimise %s  lost %s  newmap %s" % (
            e.get("loop"), e.get("optimise"), e.get("lost"), e.get("newmap")))


if __name__ == "__main__":
    main()
