#!/usr/bin/env python3
"""
Task A re-extraction, for re-running the ICP loop-closure analysis against
a bag recorded under the CORRECTED stereo calibration (c34769e: fx 457.1 ->
417.03, a 9.6% error that scaled every triangulated map point).

Why this exists rather than reusing task_a_extract.py:

  PARAMETERISED BAG. The original hardcodes the single August mission. The
  whole point here is to run the same analysis on a different bag, so the
  bag and output path are arguments.

  TWO PASSES INSTEAD OF ONE. The original decodes every /map_points
  snapshot. These bags carry 263-346 snapshots of up to ~340k points, and
  the original's per-point struct.unpack_from loop is ~1e8 Python-level
  iterations. The analysis only ever indexes four snapshots (the one at
  each face window's start and end), so pass 1 reads the cheap topics and
  records only each snapshot's timestamp and size, the windows are solved
  from that, and pass 2 decodes just the snapshots those windows name.
  Non-decoded slots are written as empty arrays, which keeps mapc_pts
  index-aligned with mapc_t -- task_a_icp.py locates snapshots by
  searchsorted on mapc_t, so the indices must not shift.

  VECTORISED DECODE. numpy.frombuffer over a structured dtype instead of
  per-point unpacking, for the snapshots that are decoded.

  MONOTONICITY CHECK. The original's "new points in this window" step is a
  set difference that assumes the cumulative map only grows. Under the
  Atlas fragmentation measured in these runs the map can collapse (e.g.
  339731 -> 938 points when SLAM re-initialises), and across such a drop
  the difference is meaningless rather than merely noisy. This script
  reports any drop inside or spanning a face window so the analysis is
  not run blind over one.

Usage: python3 eval/task_a_reextract.py <bag_dir> <out.npz>
"""
import glob
import math
import os
import sys

import numpy as np
from mcap_ros2.reader import read_ros2_messages

# Mission geometry constants -- duplicated from task_a_icp.py deliberately:
# the window solve has to happen here, in pass 1, to know which snapshots
# pass 2 needs. Kept byte-identical in value so the two agree.
TURBINE_X, TURBINE_Y = 20.0, 0.0
SCAN_DIST = 17.0
FACE_BEARINGS = [180.0, 90.0, 0.0, 270.0]

TOPIC_ODOM = "/bluerov2/odometry"
TOPIC_SLAM = "/bluerov2/robot_pose_slam"
TOPIC_MAP = "/bluerov2/map_points"


def ts(msg):
    return msg.log_time.timestamp() if hasattr(msg.log_time, "timestamp") \
        else msg.log_time / 1e9


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def decode_pointcloud2(m):
    """PointCloud2 -> Nx3 float64 xyz, vectorised.

    Reads x/y/z at their declared offsets out of one structured view of
    the raw buffer. Equivalent to the original per-point unpack loop but
    without the 1e8 Python iterations; asserts float32 so a datatype
    change can't be silently misread as garbage.
    """
    fields = {f.name: f for f in m.fields}
    if not all(k in fields for k in ("x", "y", "z")):
        return np.zeros((0, 3))
    for k in ("x", "y", "z"):
        if fields[k].datatype != 7:          # 7 == FLOAT32
            raise ValueError(f"field {k} is datatype {fields[k].datatype}, expected FLOAT32")
    n = m.width * m.height
    buf = bytes(m.data)
    step = m.point_step
    if len(buf) < n * step:
        n = len(buf) // step
    raw = np.frombuffer(buf, dtype=np.uint8, count=n * step).reshape(n, step)
    out = np.empty((n, 3))
    for j, k in enumerate(("x", "y", "z")):
        o = fields[k].offset
        out[:, j] = raw[:, o:o + 4].copy().view(np.float32).ravel()
    return out


def face_window(odom_t, odom_xyz, face_idx):
    """Identical predicate to task_a_icp.py's face_window: within 2.5 m of
    the 17 m standoff AND within 20 deg of that face's assigned bearing."""
    dx = odom_xyz[:, 0] - TURBINE_X
    dy = odom_xyz[:, 1] - TURBINE_Y
    r = np.hypot(dx, dy)
    bearing = np.degrees(np.arctan2(dy, dx)) % 360
    ang = np.abs((bearing - FACE_BEARINGS[face_idx] + 180) % 360 - 180)
    idxs = np.nonzero((np.abs(r - SCAN_DIST) < 2.5) & (ang < 20))[0]
    if len(idxs) == 0:
        return None
    return odom_t[idxs[0]], odom_t[idxs[-1]]


def main(bag, out):
    mcaps = sorted(glob.glob(os.path.join(bag, "*.mcap")))
    if not mcaps:
        sys.exit(f"no .mcap files under {bag}")
    print(f"bag: {bag}  ({len(mcaps)} file(s))")

    # ---- pass 1: cheap topics + snapshot index only -----------------------
    odom_t, odom_xyz, odom_yaw = [], [], []
    slam_t, slam_xyz, slam_yaw = [], [], []
    mapc_t, mapc_n = [], []
    for f in mcaps:
        for msg in read_ros2_messages(f, topics=[TOPIC_ODOM, TOPIC_SLAM, TOPIC_MAP]):
            m, t, topic = msg.ros_msg, ts(msg), msg.channel.topic
            if topic == TOPIC_ODOM:
                p = m.pose.pose.position
                odom_t.append(t); odom_xyz.append((p.x, p.y, p.z))
                odom_yaw.append(yaw_from_quat(m.pose.pose.orientation))
            elif topic == TOPIC_SLAM:
                p = m.pose.position
                slam_t.append(t); slam_xyz.append((p.x, p.y, p.z))
                slam_yaw.append(yaw_from_quat(m.pose.orientation))
            elif topic == TOPIC_MAP:
                mapc_t.append(t); mapc_n.append(m.width * m.height)

    odom_t = np.array(odom_t); odom_xyz = np.array(odom_xyz); odom_yaw = np.array(odom_yaw)
    slam_t = np.array(slam_t); slam_xyz = np.array(slam_xyz); slam_yaw = np.array(slam_yaw)
    mapc_t = np.array(mapc_t); mapc_n = np.array(mapc_n)
    o = np.argsort(odom_t); odom_t, odom_xyz, odom_yaw = odom_t[o], odom_xyz[o], odom_yaw[o]
    o = np.argsort(slam_t); slam_t, slam_xyz, slam_yaw = slam_t[o], slam_xyz[o], slam_yaw[o]
    o = np.argsort(mapc_t); mapc_t, mapc_n = mapc_t[o], mapc_n[o]
    print(f"pass 1: odometry={len(odom_t)}  robot_pose_slam={len(slam_t)}  "
          f"map_points snapshots={len(mapc_t)}")
    print(f"        snapshot sizes: min={mapc_n.min()} max={mapc_n.max()} last={mapc_n[-1]}")

    # Timestamps are rebased to mission-relative seconds, matching the
    # original extract's downstream use (face windows were reported as
    # t=[311.4, 447.7] etc., not as epoch seconds).
    t0 = min(odom_t[0], slam_t[0], mapc_t[0] if len(mapc_t) else odom_t[0])
    odom_t -= t0; slam_t -= t0; mapc_t -= t0

    # ---- windows, and which snapshots they name ---------------------------
    wins = {}
    for fi in (1, 2):
        w = face_window(odom_t, odom_xyz, fi)
        if w is None:
            sys.exit(f"face {fi} never entered its SCAN envelope in this bag -- "
                     f"the mission did not produce a comparable dwell")
        wins[fi] = w
        print(f"        face{fi}: t=[{w[0]:.1f}, {w[1]:.1f}]  dwell={w[1]-w[0]:.1f}s")

    need = set()
    for fi, (s, e) in wins.items():
        i0 = max(0, int(np.searchsorted(mapc_t, s)) - 1)
        i1 = min(len(mapc_t) - 1, int(np.searchsorted(mapc_t, e)))
        need |= {i0, i1}
        seg = mapc_n[i0:i1 + 1]
        drops = np.nonzero(np.diff(seg.astype(np.int64)) < 0)[0]
        print(f"        face{fi}: snapshots [{i0}..{i1}]  "
              f"{mapc_n[i0]} -> {mapc_n[i1]} pts")
        if len(drops):
            worst = min(seg[d + 1] - seg[d] for d in drops)
            print(f"          !! {len(drops)} non-monotonic drop(s) inside this "
                  f"window, largest {worst} pts -- the cumulative-map set "
                  f"difference is NOT valid across a map reset")
        if mapc_n[i1] < mapc_n[i0]:
            print(f"          !! end snapshot is SMALLER than start "
                  f"({mapc_n[i1]} < {mapc_n[i0]}): map was reset across this "
                  f"window, 'new points' is undefined here")

    # ---- pass 2: decode only those snapshots ------------------------------
    need = sorted(need)
    print(f"pass 2: decoding {len(need)} of {len(mapc_t)} snapshots: {need}")
    decoded = {}
    seen = -1
    for f in mcaps:
        for msg in read_ros2_messages(f, topics=[TOPIC_MAP]):
            seen += 1
            if seen in need:
                decoded[seen] = decode_pointcloud2(msg.ros_msg)
                print(f"        snapshot {seen}: {len(decoded[seen])} pts")
    missing = [i for i in need if i not in decoded]
    if missing:
        sys.exit(f"failed to decode snapshots {missing}")

    mapc_pts = np.empty(len(mapc_t), dtype=object)
    for i in range(len(mapc_t)):
        mapc_pts[i] = decoded.get(i, np.zeros((0, 3)))

    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    np.savez(out,
             odom_t=odom_t, odom_xyz=odom_xyz, odom_yaw=odom_yaw,
             slam_t=slam_t, slam_xyz=slam_xyz, slam_yaw=slam_yaw,
             ekf_t=np.zeros(0), ekf_xy=np.zeros((0, 2)),
             mapc_t=mapc_t, mapc_pts=mapc_pts)
    print(f"saved {out}  ({os.path.getsize(out)/1e6:.1f} MB)")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__.strip().splitlines()[-1])
    main(sys.argv[1], sys.argv[2])
