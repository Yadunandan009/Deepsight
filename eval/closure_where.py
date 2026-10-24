#!/usr/bin/env python3
"""
Where in the mission does each classified loop closure fall, and what was the
vehicle doing?  Descriptive only: ground truth scores, nothing feeds navigation.

For every flight in the manifest (plus the earlier recordings in
closures_data.json) the closures are re-classified with eval/closure_events.py
and each event is given the vehicle's TRUE state at that instant:

  t           s since the first odometry sample
  phase       scan_down | scan_up | ring (low depth rate, bearing rate above
              0.3 deg/s; not necessarily the 9.5 m transit orbit) | still, from the
              10 s smoothed depth rate and bearing rate
  z, vz       depth (m) and depth rate (m/s)
  bearing     true bearing about the structure's axis (deg), r its radius
  face_off    degrees from the nearest face centre (faces 0, 90, 180, -90 deg)
  face_idx    which face centre that is, as a bearing
  visits      for the place the closure points to (the bearing AFTER the
              closure, same depth within 3 m): seconds since the vehicle was
              last within 10 deg of it at that depth, and how many separate
              earlier passes there were (None if never)
  nearest     3D distance (m) from the vehicle to its own earlier track,
              ignoring the last 60 s

Output: eval/figures/closure_where.json and a printed table.

Run:  python3 eval/closure_where.py            (use cache)
      python3 eval/closure_where.py --refresh  (re-read every bag)
"""
import csv
import glob
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import closure_events as ce  # noqa: E402
from mcap_ros2.reader import read_ros2_messages  # noqa: E402

ROOT = os.path.dirname(HERE)
OUT = os.path.join(HERE, "figures", "closure_where.json")


def odom_xyz(bag):
    r = []
    for f in sorted(glob.glob(os.path.join(bag, "*.mcap"))):
        for m in read_ros2_messages(f, topics=["/bluerov2/odometry"]):
            t = (m.log_time.timestamp() if hasattr(m.log_time, "timestamp")
                 else m.log_time / 1e9)
            p = m.ros_msg.pose.pose.position
            r.append((t, p.x, p.y, abs(p.z)))
    return np.array(r)


def smooth_rate(t, v, w=10.0):
    out = np.zeros_like(v)
    for i in range(len(t)):
        lo = np.searchsorted(t, t[i] - w / 2)
        hi = min(np.searchsorted(t, t[i] + w / 2), len(t) - 1)
        dt = t[hi] - t[lo]
        out[i] = 0.0 if dt <= 0 else (v[hi] - v[lo]) / dt
    return out


def bearings(x, y):
    return np.degrees(np.arctan2(y - ce.TURBINE[1], x - ce.TURBINE[0]))


def unwrap_deg(a):
    return np.degrees(np.unwrap(np.radians(a)))


def features(od, row):
    t0 = od[0, 0]
    t, x, y, z = od[:, 0] - t0, od[:, 1], od[:, 2], od[:, 3]
    b = bearings(x, y)
    bu = unwrap_deg(b)
    vz = smooth_rate(t, z)
    db = smooth_rate(t, bu)
    k = int(np.searchsorted(t, row["t"]))
    k = min(k, len(t) - 1)
    if abs(vz[k]) > 0.25:
        phase = "scan_down" if vz[k] > 0 else "scan_up"
    elif abs(db[k]) > 0.3:
        phase = "ring"
    else:
        phase = "still"
    cardinals = np.array([180.0, 90.0, 0.0, -90.0])
    off = np.array([abs(ce.wrap_deg(b[k] - c)) for c in cardinals])
    fi = int(np.argmin(off))
    out = dict(phase=phase, z=float(z[k]), vz=float(vz[k]),
               bearing=float(b[k]), r=float(math.hypot(x[k] - ce.TURBINE[0],
                                                       y[k] - ce.TURBINE[1])),
               face_off=float(off[fi]), face_idx=float(cardinals[fi]),
               bearing_rate=float(db[k]))
    past = t < t[k] - 60.0
    if past.any():
        d = np.sqrt((x[past] - x[k]) ** 2 + (y[past] - y[k]) ** 2
                    + (z[past] - z[k]) ** 2)
        out["nearest"] = float(d.min())
    else:
        out["nearest"] = None
    if "bearing_after" in row:
        ba = row["bearing_after"]
        at = (np.abs([ce.wrap_deg(v - ba) for v in b]) < 10.0) \
            & (np.abs(z - z[k]) < 3.0) & (t < t[k] - 5.0)
        if at.any():
            idx = np.flatnonzero(at)
            out["since_visit"] = float(t[k] - t[idx[-1]])
            out["passes"] = int(1 + np.sum(np.diff(idx) > 20))
        else:
            out["since_visit"], out["passes"] = None, 0
    return out


def flights():
    seen, bags = set(), []
    d = json.load(open(os.path.join(HERE, "figures", "closures_data.json")))
    for e in d["events"]:
        if e["bag"] not in seen:
            seen.add(e["bag"])
            bags.append((e["bag"], "closures_data"))
    with open(os.path.join(ROOT, "docs", "experiments", "flights_manifest.csv")) as fh:
        for r in csv.DictReader(fh):
            nm = os.path.basename(r["bag"])
            if nm not in seen:
                seen.add(nm)
                bags.append((nm, r["arm"]))
    return bags


def main():
    refresh = "--refresh" in sys.argv
    cache = json.load(open(OUT)) if os.path.exists(OUT) and not refresh else {}
    for name, arm in flights():
        if name in cache:
            continue
        bag = os.path.join(ROOT, "bags", name)
        if not os.path.isdir(bag):
            print("missing bag", name)
            continue
        log = ce.pick_log(bag)
        if not log:
            print("no console log", name)
            cache[name] = dict(arm=arm, events=[], note="no console log")
            continue
        print("reading", name, flush=True)
        rows, _ = ce.classify(bag, log)
        od = odom_xyz(bag)
        ev = []
        for r in rows:
            e = dict(r)
            if r["cls"] != "unscorable":
                e.update(features(od, r))
            ev.append(e)
        cache[name] = dict(arm=arm, duration=float(od[-1, 0] - od[0, 0]), events=ev)
        json.dump(cache, open(OUT, "w"), indent=1)
    json.dump(cache, open(OUT, "w"), indent=1)
    print()
    hdr = f"{'bag':44s} {'arm':10s} {'t':>6s} {'class':>12s} {'phase':>9s} {'z':>5s} {'vz':>5s} {'brg':>6s} {'face':>5s} {'off':>4s} {'sinceV':>6s} {'pass':>4s} {'nearest':>7s}"
    print(hdr)
    for name, v in cache.items():
        for e in v["events"]:
            if e["cls"] == "unscorable":
                print(f"{name[:44]:44s} {v['arm'][:10]:10s} {e['t']:6.0f} {'unscorable':>12s}")
                continue
            sv = e.get("since_visit")
            print(f"{name[:44]:44s} {v['arm'][:10]:10s} {e['t']:6.0f} {e['cls']:>12s} "
                  f"{e['phase']:>9s} {e['z']:5.1f} {e['vz']:5.2f} {e['bearing']:6.0f} "
                  f"{e['face_idx']:5.0f} {e['face_off']:4.0f} "
                  f"{('%6.0f' % sv) if sv is not None else '  none':>6s} "
                  f"{e.get('passes', ''):>4} "
                  f"{('%7.1f' % e['nearest']) if e['nearest'] is not None else '   none':>7s}")


if __name__ == "__main__":
    main()
