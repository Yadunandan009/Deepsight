#!/usr/bin/env python3
"""Is SLAM's own HEADING drifting, independent of its position?

The alignment rotation theta wanders ~20 deg per 40 s window. If that is
SLAM's yaw estimate drifting against the world, it is directly visible by
comparing the yaw in /bluerov2/robot_pose_slam against ground truth --
no Procrustes, no alignment, no fitting.
"""
import glob, math, os, sys
import numpy as np
from mcap_ros2.reader import read_ros2_messages

def yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))

for bag in sys.argv[1:]:
    s, t = [], []
    for f in sorted(glob.glob(os.path.join(bag, "*.mcap"))):
        for m in read_ros2_messages(f, topics=["/bluerov2/robot_pose_slam",
                                               "/bluerov2/odometry"]):
            ts = (m.log_time.timestamp() if hasattr(m.log_time, "timestamp")
                  else m.log_time / 1e9)
            if m.channel.topic.endswith("robot_pose_slam"):
                s.append((ts, yaw(m.ros_msg.pose.orientation)))
            else:
                t.append((ts, yaw(m.ros_msg.pose.pose.orientation)))
    if not s or not t:
        print(f"{os.path.basename(bag)}: missing a topic"); continue
    S, T = np.array(s), np.array(t)
    ty = np.interp(S[:, 0], T[:, 0], np.unwrap(T[:, 1]))
    sy = np.unwrap(S[:, 1])
    d = sy - ty
    d = d - d[0]                      # SLAM's frame starts arbitrary
    rel = S[:, 0] - S[0, 0]
    dd = np.degrees(d)
    # least-squares drift rate
    k = np.polyfit(rel, dd, 1)[0]
    print(f"{os.path.basename(bag)[:38]:40s}")
    print(f"   SLAM yaw minus truth yaw, relative to t=0:")
    print(f"     final {dd[-1]:+8.1f} deg   max |err| {np.abs(dd).max():7.1f} deg"
          f"   drift rate {k*60:+6.2f} deg/min")
    for q in (25, 50, 75, 95):
        print(f"     p{q:<2d} |err| {np.percentile(np.abs(dd), q):7.1f} deg")
