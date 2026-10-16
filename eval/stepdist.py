#!/usr/bin/env python3
"""Discrete face confusion, or continuous rotational drift?

The two mechanisms make opposite predictions about how the SLAM->world
rotation moves between adjacent sliding windows.

  Perceptual aliasing at loop closure: theta sits in a basin and
  occasionally JUMPS by a multiple of 90 deg. Most adjacent steps are
  ~0; a minority are ~90/180.

  Drift / map fragmentation: theta wanders. Steps are spread, with no
  mass at 0 and no mass at 90.
"""
import glob, math, os, sys
import numpy as np
from mcap_ros2.reader import read_ros2_messages

sys.path.insert(0, os.path.expanduser("~/ros2_ws/eval"))
import analyze_anchorfree as A
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from slamio import slam_to_ned_xy

WIN, HOP = 40.0, 10.0


def track(bag, topic):
    rows = []
    for f in sorted(glob.glob(os.path.join(bag, "*.mcap"))):
        for m in read_ros2_messages(f, topics=[topic]):
            t = (m.log_time.timestamp() if hasattr(m.log_time, "timestamp")
                 else m.log_time / 1e9)
            # robot_pose_slam is PoseStamped, odometry/dr is Odometry
            ps = m.ros_msg.pose
            if hasattr(ps, "pose"):            # Odometry: already NED
                p = ps.pose.position
                rows.append((t, p.x, p.y))
            else:                              # the SLAM pose: ENU -> NED
                rows.append((t,) + slam_to_ned_xy(ps.position.x,
                                                  ps.position.y))
    return np.array(rows) if rows else None


def theta(s, e):
    """Rotation-only Procrustes, SO(2), on mean-removed pairs."""
    s = s - s.mean(0)
    e = e - e.mean(0)
    num = (s[:, 0] * e[:, 1] - s[:, 1] * e[:, 0]).sum()
    den = (s[:, 0] * e[:, 0] + s[:, 1] * e[:, 1]).sum()
    return math.atan2(num, den)


steps = []
for bag in sys.argv[1:]:
    sl = track(bag, "/bluerov2/robot_pose_slam")
    rf = track(bag, "/bluerov2/odometry/dr")
    if sl is None or rf is None:
        print(f"  {os.path.basename(bag)}: missing a track")
        continue
    t0, t1 = max(sl[0, 0], rf[0, 0]), min(sl[-1, 0], rf[-1, 0])
    ths, c = [], t0
    while c + WIN <= t1:
        m = (sl[:, 0] >= c) & (sl[:, 0] < c + WIN)
        if m.sum() >= 20:
            ts = sl[m, 0]
            ths.append(theta(sl[m, 1:3], A.interp_to(rf[:, 0], rf[:, 1:3], ts)))
        c += HOP
    d = [abs(math.degrees(math.atan2(math.sin(b - a), math.cos(b - a))))
         for a, b in zip(ths, ths[1:])]
    steps += d
    print(f"  {os.path.basename(bag)[:34]:36s} {len(d):3d} steps  "
          f"median {np.median(d):5.1f} deg")

d = np.array(steps)
n = len(d)
print(f"\n  pooled: {n} adjacent-window steps over {len(sys.argv)-1} runs")
print(f"  median {np.median(d):.1f} deg   mean {d.mean():.1f} deg\n")
print("                                 observed    aliasing predicts")
near0 = (d < 15).mean()
near90 = ((d > 75) & (d < 105)).mean()
near180 = (d > 165).mean()
mid = ((d >= 15) & (d <= 75)).mean()
print(f"    step < 15 deg  (same basin)   {100*near0:5.1f} %      the majority")
print(f"    step 75-105    (one face)     {100*near90:5.1f} %      the rest")
print(f"    step > 165     (two faces)    {100*near180:5.1f} %      a little")
print(f"    step 15-75     (neither)      {100*mid:5.1f} %      ~0, forbidden")
