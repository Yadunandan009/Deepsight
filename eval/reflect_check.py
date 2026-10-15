#!/usr/bin/env python3
"""Is the SLAM track a REFLECTION of the true track?

The bridge fits a rotation. If the published SLAM track is mirrored --
a left-handed frame reaching a right-handed consumer -- then no rotation
can ever fit it, the best-fit angle wanders with the vehicle, and the
residual stays at trajectory scale. Every symptom measured so far.

Fit both and compare: det=+1 (rotation, what the bridge assumes) against
det=-1 (reflection). If the reflected fit collapses the residual, the
defect is a frame convention, not the sensor and not the scene.
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
            r.append((t, p.x, p.y))
    return np.array(r) if r else None


def fit(S, E, allow_reflection):
    """Best similarity S->E. Returns (scale, deg, residual rms)."""
    s = S - S.mean(0)
    e = E - E.mean(0)
    U, D, Vt = np.linalg.svd(s.T @ e)
    d = np.sign(np.linalg.det(U @ Vt))
    corr = np.diag([1.0, d if not allow_reflection else 1.0])
    R = (U @ corr @ Vt).T
    if not allow_reflection and np.linalg.det(R) < 0:
        R = (U @ np.diag([1.0, -1.0]) @ Vt).T
    k = (R @ s.T).T
    sc = (k * e).sum() / (k * k).sum()
    res = np.sqrt(((sc * k - e) ** 2).sum(1).mean())
    return sc, math.degrees(math.atan2(R[1, 0], R[0, 0])), res, np.linalg.det(R)


print(f"  {'run':34s} {'det':>5} {'scale':>7} {'theta':>8} {'residual':>9}")
for bag in sys.argv[1:]:
    sl = track(bag, "/bluerov2/robot_pose_slam")
    rf = track(bag, "/bluerov2/odometry/dr")
    if sl is None or rf is None:
        print(f"  {os.path.basename(bag)[:34]}: missing a track"); continue
    E = A.interp_to(rf[:, 0], rf[:, 1:3], sl[:, 0])
    name = os.path.basename(bag)[:34]
    for lab, mirror in (("rotation", False), ("reflection", True)):
        s = sl[:, 1:3].copy()
        if mirror:
            s[:, 1] *= -1.0          # mirror the SLAM track about its x axis
        sc, th, res, det = fit(s, E, False)
        tag = f"{name if lab=='rotation' else '':34s}"
        print(f"  {tag} {'+1' if not mirror else '-1':>5} {sc:7.3f} "
              f"{th:+7.0f}d {res:8.2f} m   ({lab})")
    print()
