#!/usr/bin/env python3
"""Replay recorded bags through the real slam_pose_bridge, offline.

Feeds each bag's raw SLAM pose, map-point counts and the bridge's reference
odometry into SlamPoseBridge in log-time order, with time.time() pinned to
the message time and the 5 s refit timer driven from the same clock, and
captures what it would have published. Run twice per bag:

  relock      the behaviour of every run before 2026-10-25
  quarantine  the jump_policy fix (see slam_pose_bridge.py)

and reports, per bag:
  fidelity    relock replay against the bridge output actually recorded
              (median distance between them and the pose-count ratio). Exact
              agreement is only expected for bags recorded with
              dr_reference:=true: in the others the reference is the primary
              filter, which the bridge itself feeds, and the replay uses the
              filter as it was recorded, i.e. open-loop.
  identical   whether the two policies publish exactly the same poses. Must
              be true for every bag with no quarantine.
  quarantine  when it started and what relock published afterwards (count,
              median and max error against ground truth), and what the
              SLAM-free reference did over the same period.

Ground truth enters only to score.

Usage:  source ROS, then  python3 eval/replay_bridge.py [BAG ...]
        (default: every bag under the external drive's bags/ that holds the
         raw SLAM pose; writes eval/figures/replay_bridge.json)
"""
import glob
import importlib.util
import json
import os
import sys
from types import SimpleNamespace

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from mcap_ros2.reader import read_ros2_messages
from nav_msgs.msg import Odometry

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)
BAG_ROOT = "/media/yadunandan/more space/bags"
BRIDGE = os.path.join(WS, "src", "stonefish_bluerov2", "scripts", "slam_pose_bridge.py")
# The SLAM pose was taken as NED, unconverted, until this bag's evening
# (docs/SLAM_FRAME_ENU_NED.md); slamned_seed0_20261016_160033 is the last
# bag flown that way.
ENU_FROM = "20261016_19"

spec = importlib.util.spec_from_file_location("slam_pose_bridge", BRIDGE)
spb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(spb)


class Clock:
    t = 0.0


spb.time = SimpleNamespace(time=lambda: Clock.t)


def load(bag):
    meta = open(os.path.join(bag, "metadata.yaml")).read()
    ref = "/bluerov2/odometry/dr" if "/bluerov2/odometry/dr" in meta else "/bluerov2/odometry/filtered"
    topics = ["/bluerov2/robot_pose_slam", "/bluerov2/map_points", ref,
              "/bluerov2/odometry", "/bluerov2/robot_pose_slam_ekf"]
    ev, truth, rec = [], [], []
    for f in sorted(glob.glob(os.path.join(bag, "*.mcap"))):
        for m in read_ros2_messages(f, topics=topics):
            t = m.log_time.timestamp()
            k, r = m.channel.topic, m.ros_msg
            if k == "/bluerov2/robot_pose_slam":
                p, q = r.pose.position, r.pose.orientation
                ev.append((t, 0, (p.x, p.y, p.z, q.x, q.y, q.z, q.w)))
            elif k == "/bluerov2/map_points":
                ev.append((t, 1, r.width * r.height))
            elif k == "/bluerov2/robot_pose_slam_ekf":
                p = r.pose.pose.position
                rec.append((t, p.x, p.y))
            else:
                p = r.pose.pose.position
                if k == "/bluerov2/odometry":
                    truth.append((t, p.x, p.y))
                if k == ref:
                    ev.append((t, 2, (p.x, p.y, p.z)))
    ev.sort(key=lambda e: (e[0], e[1]))
    return ref, ev, np.array(truth), np.array(rec)


def replay(ev, frame, policy):
    rclpy.init(args=["--ros-args", "-p", f"slam_frame:={frame}", "-p", f"jump_policy:={policy}",
                     "--log-level", "slam_pose_bridge:=error"])
    node = spb.SlamPoseBridge()
    out = []
    node.pub = SimpleNamespace(publish=lambda m: out.append(
        (Clock.t, m.pose.pose.position.x, m.pose.pose.position.y)))
    q_on = []
    next_refit = None
    for t, kind, v in ev:
        Clock.t = t
        if next_refit is None:
            next_refit = t + node.REFIT_INTERVAL
        while t >= next_refit:                      # the 5 s timer, on bag time
            Clock.t = next_refit
            node._periodic_refit()
            next_refit += node.REFIT_INTERVAL
        Clock.t = t
        if kind == 0:
            m = PoseStamped()
            m.pose.position.x, m.pose.position.y, m.pose.position.z = v[:3]
            (m.pose.orientation.x, m.pose.orientation.y,
             m.pose.orientation.z, m.pose.orientation.w) = v[3:]
            was = node.quarantined
            node.slam_cb(m)
            if node.quarantined and not was:
                q_on.append(t)
        elif kind == 1:
            node.map_cb(SimpleNamespace(width=v, height=1))
        else:
            m = Odometry()
            m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = v
            node.ekf_cb(m)
    node.destroy_node()
    rclpy.shutdown()
    return np.array(out).reshape(-1, 3), q_on


def err(a, g):
    if not len(a):
        return np.array([])
    return np.hypot(a[:, 1] - np.interp(a[:, 0], g[:, 0], g[:, 1]),
                    a[:, 2] - np.interp(a[:, 0], g[:, 0], g[:, 2]))


def main():
    bags = sys.argv[1:] or [b for b in sorted(glob.glob(os.path.join(BAG_ROOT, "*")))
                            if os.path.exists(os.path.join(b, "metadata.yaml")) and
                            "name: /bluerov2/robot_pose_slam\n" in open(os.path.join(b, "metadata.yaml")).read()]
    res = []
    for bag in bags:
        name = os.path.basename(bag.rstrip("/"))
        ref, ev, g, rec = load(bag)
        if not len(g) or not any(e[1] == 0 for e in ev):
            continue
        stamp = "_".join(name.split("_")[-2:])     # YYYYMMDD_HHMMSS
        frame = "enu" if stamp >= ENU_FROM else "ned"
        rl, _ = replay(ev, frame, "relock")
        qu, q_on = replay(ev, frame, "quarantine")
        t0 = g[0, 0]
        r = dict(bag=name, reference=ref, slam_frame=frame,
                 published_relock=len(rl), published_quarantine=len(qu),
                 identical=bool(len(rl) == len(qu) and np.array_equal(rl, qu)))
        if len(rec):
            d = np.hypot(np.interp(rec[:, 0], rl[:, 0], rl[:, 1]) - rec[:, 1],
                         np.interp(rec[:, 0], rl[:, 0], rl[:, 2]) - rec[:, 2]) if len(rl) else np.array([np.nan])
            r.update(recorded=len(rec), fidelity_median_m=float(np.median(d)),
                     fidelity_count_ratio=len(rl) / len(rec))
        r["quarantines_s"] = [round(t - t0, 1) for t in q_on]
        if q_on:
            tq = q_on[0]
            after = rl[rl[:, 0] > tq]
            e = err(after, g)
            r.update(relock_after_n=len(after),
                     relock_after_first_s=round(after[0, 0] - t0, 1) if len(after) else None,
                     relock_after_err_median=float(np.median(e)) if len(e) else None,
                     relock_after_err_max=float(e.max()) if len(e) else None,
                     quarantine_after_n=int((qu[:, 0] > tq).sum()) if len(qu) else 0)
            qa = qu[qu[:, 0] > tq] if len(qu) else qu
            eq = err(qa, g)
            r.update(quarantine_after_first_s=round(qa[0, 0] - t0, 1) if len(qa) else None,
                     quarantine_after_err_median=float(np.median(eq)) if len(eq) else None,
                     quarantine_after_err_max=float(eq.max()) if len(eq) else None)
        res.append(r)
        print(json.dumps(r))
    json.dump(res, open(os.path.join(HERE, "figures", "replay_bridge.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
