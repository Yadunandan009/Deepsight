#!/usr/bin/env python3
"""Which axis is mirrored, and is the mirror the same every run?

Frame relabelling and a swapped stereo pair both mirror a track. They
differ in where the mirror sits:

  frame relabelling -- a FIXED axis of the published frame, identical in
                       every run, and the Z axis is implicated too
  stereo swap       -- breaks triangulation; no clean fixed mirror, and
                       no reason for the vertical axis to invert

Z is the discriminator: depth is measured independently by the pressure
sensor (0.005 m error), so truth Z is beyond dispute.
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


for bag in sys.argv[1:]:
    sl = track(bag, "/bluerov2/robot_pose_slam")
    tr = track(bag, "/bluerov2/odometry")
    if sl is None or tr is None:
        print(f"{os.path.basename(bag)}: missing a track"); continue
    name = os.path.basename(bag)[:34]
    tz = np.interp(sl[:, 0], tr[:, 0], tr[:, 3])
    sz = sl[:, 3]
    # how does SLAM's published Z track the independently-measured depth?
    sz_c, tz_c = sz - sz.mean(), tz - tz.mean()
    g = (sz_c * tz_c).sum() / (tz_c * tz_c).sum()
    rho = np.corrcoef(sz_c, tz_c)[0, 1]
    # mirror line: mirror-about-x then rotate by th  ==  mirror about th/2
    E = A.interp_to(tr[:, 0], tr[:, 1:3], sl[:, 0])
    s = sl[:, 1:3].copy(); s[:, 1] *= -1.0
    s = s - s.mean(0); e = E - E.mean(0)
    U, D, Vt = np.linalg.svd(s.T @ e)
    d = np.sign(np.linalg.det(U @ Vt))
    R = (U @ np.diag([1.0, d]) @ Vt).T
    th = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    print(f"{name:36s}")
    print(f"   vertical: SLAM z vs truth z   gain {g:+.3f}   corr {rho:+.3f}"
          f"   {'INVERTED' if g < 0 else 'same sign'}")
    print(f"   horizontal mirror line at {th/2:+.1f} deg from the world x axis")
