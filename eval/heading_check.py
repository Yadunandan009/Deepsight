#!/usr/bin/env python3
"""Does the IMU heading see a wrong-face match that the cameras cannot?

A wrong-face match on a four-fold symmetric structure re-poses the estimate
by a rotation about the structure's axis, so the camera's heading in the
SLAM map turns by the same 90 or 180 deg while the vehicle itself did not
turn. The IMU heading does not take part in the match. So

    d(t) = heading of the raw SLAM pose  -  heading from the IMU

is constant for a given map (the map's arbitrary heading) and should step by
the symmetry angle at a wrong-face match and by about nothing at a benign
one. This measures that, offline, on recorded bags, for every ORB-SLAM3
event that can re-pose the estimate:

    loop        `Loop detected`         (also classified by closure_events)
    merge       `Merge detected`        an Atlas map merge
    reloc       `Relocalized`           re-localisation after tracking loss
    new map     `Creation of new map`   a fresh map; its heading is
                                        arbitrary, so a step here is expected

and lists every heading step that no logged event explains.

The SLAM pose is published east-north-up; its heading is negated to compare
with the north-east-down IMU (docs/SLAM_FRAME_ENU_NED.md). The simulated IMU
heading is currently perfect, so this measures the size of the steps, not
how well a real heading sensor would separate them; that needs a heading
error model and comes next. Ground truth enters only through
closure_events' class labels.

Usage:  python3 eval/heading_check.py [BAG ...]
        (default: every bag under the external drive's bags/ holding the raw
         SLAM pose and the IMU; writes eval/figures/heading_check.json)
"""
import glob
import json
import math
import os
import re
import sys

import numpy as np
from mcap_ros2.reader import read_ros2_messages

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import closure_events as ce  # noqa: E402

BAG_ROOT = "/media/yadunandan/more space/bags"
STEP_MIN = 5.0          # deg   a step between consecutive SLAM poses this big is listed
WINDOW = (-3.0, 30.0)   # s     where a logged event's step is looked for, as closure_events
SYM = 90.0              # deg   the structure's symmetry angle
PAIR_S = 2.0            # s     either side of a classified closure's position jump

KINDS = (("loop", "Loop detected"), ("merge", "Merge detected"),
         ("reloc", "Relocalized"), ("new map", "Creation of new map with id"))


def yaw(o):
    return math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))


def load(bag):
    S, I = [], []
    for f in sorted(glob.glob(os.path.join(bag, "*.mcap"))):
        for m in read_ros2_messages(f, topics=["/bluerov2/robot_pose_slam", "/bluerov2/imu"]):
            t = m.log_time.timestamp()
            if m.channel.topic == "/bluerov2/imu":
                I.append((t, yaw(m.ros_msg.orientation)))
            else:
                S.append((t, -yaw(m.ros_msg.pose.orientation)))    # ENU -> NED
    S, I = np.array(S), np.array(I)
    imu = np.interp(S[:, 0], I[:, 0], np.unwrap(I[:, 1]))
    d = np.degrees(np.unwrap(S[:, 1] - imu))
    return S[:, 0], d


def log_events(path):
    out = []
    ts = None
    for ln in open(path, errors="replace"):
        m = ce._TS.search(ln)
        if m:
            ts = float(m.group(1))
        for kind, key in KINDS:
            if key in ln and ts is not None:
                if kind == "new map" and re.search(r"with id: 0\b", ln):
                    continue        # the first map of the run
                out.append((ts, kind))
    return out


def sym_off(step):
    """Distance of a heading step from the nearest non-zero multiple of SYM."""
    a = abs(step) % 360.0
    return min(abs(a - k * SYM) for k in range(1, int(360 / SYM)))


def check(bag):
    log = ce.pick_log(bag)
    t, d = load(bag)
    step = np.diff(d)
    t0 = t[0]
    rows, used = [], set()
    for te, kind in (log_events(log) if log else []):
        w = np.flatnonzero((t[1:] >= te + WINDOW[0]) & (t[1:] <= te + WINDOW[1]))
        if not len(w):
            rows.append(dict(t=round(te - t0, 1), kind=kind, step=None))
            continue
        j = int(w[np.argmax(np.abs(step[w]))])
        used.add(j)
        rows.append(dict(t=round(te - t0, 1), kind=kind, step=round(float(step[j]), 1),
                         step_t=round(float(t[j + 1] - t0), 1),
                         off_sym=round(sym_off(step[j]), 1)))
    big = np.flatnonzero(np.abs(step) >= STEP_MIN)
    unexplained = [dict(t=round(float(t[j + 1] - t0), 1), step=round(float(step[j]), 1),
                        off_sym=round(sym_off(step[j]), 1)) for j in big if j not in used]
    # closure_events' class for each loop, with the heading step measured at the
    # very pose where it places the position jump: median of d over the
    # PAIR_S seconds before against the PAIR_S seconds after. A wider window
    # picks up a new map's arbitrary heading a few seconds later.
    classes = []
    if log:
        try:
            crow, _ = ce.classify(bag, log)
            tr0 = ce.track(bag, "/bluerov2/odometry", False)[0, 0]
            for r in crow:
                row = dict(t=round(r["t"], 1), cls=r["cls"], chord=r.get("chord"),
                           turn=r.get("turn"), heading_step=None)
                if "raw_step" in r:
                    tj = tr0 + r["t"]
                    pre = d[(t > tj - PAIR_S) & (t <= tj)]
                    post = d[(t > tj) & (t <= tj + PAIR_S)]
                    if len(pre) and len(post):
                        row["heading_step"] = round(float(np.median(post) - np.median(pre)), 1)
                classes.append(row)
        except Exception as e:                      # noqa: BLE001
            classes = [dict(error=str(e))]
    quiet = np.abs(step[np.abs(step) < STEP_MIN])
    return dict(bag=os.path.basename(bag.rstrip("/")), log=os.path.basename(log) if log else None,
                events=rows, unexplained=unexplained, loop_classes=classes,
                quiet_step_p999=round(float(np.percentile(quiet, 99.9)), 2) if len(quiet) else None)


def main():
    bags = sys.argv[1:] or [b for b in sorted(glob.glob(os.path.join(BAG_ROOT, "*")))
                            if os.path.exists(os.path.join(b, "metadata.yaml"))
                            and all(f"name: {k}\n" in open(os.path.join(b, "metadata.yaml")).read()
                                    for k in ("/bluerov2/robot_pose_slam", "/bluerov2/imu"))]
    res = []
    for bag in bags:
        r = check(bag)
        res.append(r)
        print(json.dumps(r))
    json.dump(res, open(os.path.join(HERE, "figures", "heading_check.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
