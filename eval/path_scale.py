#!/usr/bin/env python3
"""Is the 1.8-2.8x path inflation real motion, or per-sample noise?

Summing |dp| over consecutive samples is biased upward by noise: with
independent per-sample error sigma and true motion d per step,
E|dp| ~ sqrt(d^2 + 2 sigma^2). The bias is fixed while d grows with the
sampling interval, so the two hypotheses separate under decimation:

    noise          ratio -> 1.0 as the interval grows
    real motion    ratio stays put at every scale

Truth is decimated identically, so this is not an artifact of comparing
different sampling rates.
"""
import glob, math, os, sys
import numpy as np
from mcap_ros2.reader import read_ros2_messages
sys.path.insert(0, os.path.expanduser("~/ros2_ws/eval"))
from slamio import slam_to_ned_xy


def track(bag, topic, is_slam):
    r = []
    for f in sorted(glob.glob(os.path.join(bag, "*.mcap"))):
        for m in read_ros2_messages(f, topics=[topic]):
            t = (m.log_time.timestamp() if hasattr(m.log_time, "timestamp")
                 else m.log_time / 1e9)
            ps = m.ros_msg.pose
            if is_slam:
                x, y = slam_to_ned_xy(ps.position.x, ps.position.y)
            else:
                x, y = ps.pose.position.x, ps.pose.position.y
            r.append((t, x, y))
    return np.array(r)


def plen(a):
    return float(np.hypot(np.diff(a[:, 0]), np.diff(a[:, 1])).sum())


for bag in sys.argv[1:]:
    sl = track(bag, "/bluerov2/robot_pose_slam", True)
    tr = track(bag, "/bluerov2/odometry", False)
    T = np.column_stack([np.interp(sl[:, 0], tr[:, 0], tr[:, 1]),
                         np.interp(sl[:, 0], tr[:, 0], tr[:, 2])])
    S = sl[:, 1:3]
    dt = np.median(np.diff(sl[:, 0]))
    print(f"\n{os.path.basename(bag)[:44]}   {len(sl)} poses at {1/dt:.1f} Hz")
    print(f"  {'interval':>9} {'SLAM path':>10} {'truth path':>11} {'ratio':>7}")
    for k in (1, 2, 4, 8, 16, 32, 64, 128):
        s, t = S[::k], T[::k]
        if len(s) < 30:
            break
        ps, pt = plen(s), plen(t)
        print(f"  {k*dt:7.2f} s {ps:9.1f} m {pt:10.1f} m {ps/pt:7.2f}")
    # implied per-sample noise from the k=1 excess, given the k=128 truth
    d = plen(T) / (len(T) - 1)
    e = plen(S) / (len(S) - 1)
    sig = math.sqrt(max(e*e - d*d, 0.0) / 2.0)
    print(f"  implied per-sample SLAM noise: {100*sig:.1f} cm "
          f"(true motion {100*d:.1f} cm/sample)")
