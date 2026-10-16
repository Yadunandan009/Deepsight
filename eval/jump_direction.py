#!/usr/bin/env python3
"""Was the loop-closure jump a rotation about the turbine axis?

Magnitude alone is weak evidence: a chord of the right length can come
from anywhere. A FACE CONFUSION is a specific thing -- a rotation of the
estimate about the structure's axis by a multiple of 360/4 deg -- and a
rotation has two signatures, not one:

    the radius from the axis is PRESERVED
    the bearing about the axis turns by ~90 deg

Both must hold. Mapping SLAM into the world uses the alignment fitted on
pre-jump data only, so the "after" point is placed by a transform that
knew nothing about the jump.
"""
import glob, math, os, sys
import numpy as np
from mcap_ros2.reader import read_ros2_messages
sys.path.insert(0, os.path.expanduser("~/ros2_ws/eval"))
from slamio import slam_to_ned_xy

BAG = "/media/yadunandan/more space/bags/a50_slamned_seed0_20261016_160033"
TX, TY = 20.0, 0.0


def track(topic, is_slam):
    r = []
    for f in sorted(glob.glob(os.path.join(BAG, "*.mcap"))):
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


sl = track("/bluerov2/robot_pose_slam", True)
tr = track("/bluerov2/odometry", False)

# the jump: the largest consecutive step in the SLAM track
step = np.hypot(np.diff(sl[:, 1]), np.diff(sl[:, 2]))
j = int(np.argmax(step))
print(f"  largest SLAM step: {step[j]:.2f} m at t = {sl[j+1,0]-sl[0,0]:.0f}s "
      f"into the run")

# similarity SLAM->world fitted on PRE-JUMP data only
pre = sl[:, 0] < sl[j, 0]
S = sl[pre, 1:3]
E = np.column_stack([np.interp(sl[pre, 0], tr[:, 0], tr[:, 1]),
                     np.interp(sl[pre, 0], tr[:, 0], tr[:, 2])])
Sm, Em = S.mean(0), E.mean(0)
s0, e0 = S - Sm, E - Em
num = (s0[:, 0]*e0[:, 1] - s0[:, 1]*e0[:, 0]).sum()
den = (s0[:, 0]*e0[:, 0] + s0[:, 1]*e0[:, 1]).sum()
th = math.atan2(num, den)
k = math.hypot(num, den) / (s0**2).sum()
c, s_ = math.cos(th), math.sin(th)
R = np.array([[c, -s_], [s_, c]])
to_world = lambda p: k * (R @ (p - Sm)) + Em
res = np.sqrt((((k*(s0 @ R.T)) - e0)**2).sum(1).mean())
print(f"  pre-jump alignment: yaw {math.degrees(th):+.0f} deg  scale {k:.3f}"
      f"  residual {res:.2f} m   (fitted on {pre.sum()} poses)\n")

before, after = to_world(sl[j, 1:3]), to_world(sl[j+1, 1:3])
truth = np.array([np.interp(sl[j, 0], tr[:, 0], tr[:, 1]),
                  np.interp(sl[j, 0], tr[:, 0], tr[:, 2])])

def polar(p):
    d = p - np.array([TX, TY])
    return math.hypot(*d), math.degrees(math.atan2(d[1], d[0]))

for lab, p in (("truth", truth), ("SLAM before", before), ("SLAM after", after)):
    r, b = polar(p)
    print(f"  {lab:12s} ({p[0]:+7.2f},{p[1]:+7.2f})   r = {r:6.2f} m   "
          f"bearing = {b:+7.1f} deg")

rb, bb = polar(before); ra, ba = polar(after)
turn = math.degrees(math.atan2(math.sin(math.radians(ba-bb)),
                               math.cos(math.radians(ba-bb))))
print(f"\n  radius change : {ra-rb:+.2f} m  ({100*(ra-rb)/rb:+.1f} %)"
      f"   -- a rotation preserves it")
print(f"  bearing turn  : {turn:+.1f} deg"
      f"                -- a face confusion is a multiple of 90")
best = min((abs(abs(turn) - m), m) for m in (45, 90, 135, 180))
print(f"  nearest multiple: {best[1]} deg, off by {best[0]:.1f} deg")
