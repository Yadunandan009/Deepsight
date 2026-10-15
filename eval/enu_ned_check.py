#!/usr/bin/env python3
"""Does ENU->NED on the SLAM pose recover the trajectory in 3D?

The horizontal track is mirrored about x and the vertical axis is
inverted. Together that is (x, y, z) -> (x, -y, -z), whose determinant
is +1: a 180 deg rotation about x, which is exactly the ENU-to-NED
relationship. It only LOOKS like a reflection because the bridge fits in
the horizontal plane and throws z away.

If that is the defect, applying it and then fitting what the bridge
already fits -- a yaw and a scale -- should recover the track in 3D.
"""
import glob, math, os, sys
import numpy as np
from mcap_ros2.reader import read_ros2_messages
sys.path.insert(0, os.path.expanduser("~/ros2_ws/eval"))
import analyze_anchorfree as A


def track(bag, topic):
    r = []
    for f in sorted(glob.glob(os.path.join(bag, "*.mcap"))):
        for m in read_ros2_messages(f, topics=[topic]):
            t = (m.log_time.timestamp() if hasattr(m.log_time, "timestamp")
                 else m.log_time / 1e9)
            ps = m.ros_msg.pose
            p = ps.pose.position if hasattr(ps, "pose") else ps.position
            r.append((t, p.x, p.y, p.z))
    return np.array(r) if r else None


def yaw_scale_fit(S, E):
    """Best yaw + uniform scale, 3D, z left alone -- the bridge's model."""
    s, e = S - S.mean(0), E - E.mean(0)
    num = (s[:, 0] * e[:, 1] - s[:, 1] * e[:, 0]).sum()
    den = (s[:, 0] * e[:, 0] + s[:, 1] * e[:, 1]).sum()
    th = math.atan2(num, den)
    c, sn = math.cos(th), math.sin(th)
    R = np.array([[c, -sn, 0.0], [sn, c, 0.0], [0.0, 0.0, 1.0]])
    k = (R @ s.T).T
    sc = (k * e).sum() / (k * k).sum()
    res = np.sqrt(((sc * k - e) ** 2).sum(1).mean())
    return math.degrees(th), sc, res


print(f"  {'run':34s} {'pre-transform':>16} {'yaw':>7} {'scale':>7} {'3D resid':>9}")
for bag in sys.argv[1:]:
    sl = track(bag, "/bluerov2/robot_pose_slam")
    tr = track(bag, "/bluerov2/odometry")
    if sl is None or tr is None:
        continue
    E = np.column_stack([A.interp_to(tr[:, 0], tr[:, 1:3], sl[:, 0]),
                         np.interp(sl[:, 0], tr[:, 0], tr[:, 3])])
    name = os.path.basename(bag)[:34]
    for lab, M in (("none (as published)", np.diag([1.0, 1.0, 1.0])),
                   ("ENU->NED", np.diag([1.0, -1.0, -1.0]))):
        S = (M @ sl[:, 1:4].T).T
        th, sc, res = yaw_scale_fit(S, E)
        print(f"  {name if lab.startswith('none') else '':34s} {lab:>16} "
              f"{th:+6.0f}d {sc:7.3f} {res:8.2f} m")
    print()
