#!/usr/bin/env python3
"""
Task A step 2 — extract the raw topics needed for ICP loop-closure
verification from the mission bag: odometry (world-frame ground truth,
for finding face time-windows), robot_pose_slam (raw SLAM-frame pose),
robot_pose_slam_ekf (world-frame pose, output of slam_pose_bridge's
rotation-only transform -- used to recover that transform per-face),
and map_points (ORB-SLAM3's accumulated map, PointCloud2 snapshots).

Saves everything to a single .npz so the analysis step doesn't need to
re-read the 13GB bag.
"""
import glob, math, struct, sys
import numpy as np
from mcap_ros2.reader import read_ros2_messages

BAG = "/media/yadunandan/yadu1/ros2_ws_bags/DeepSight_v5_20260810_004128"
OUT = "/home/yadunandan/ros2_ws/plots/task_a_topics.npz"

mcap_files = sorted(glob.glob(BAG + "/*.mcap"))
print(f"Bag files: {len(mcap_files)}")


def ts(msg):
    return msg.log_time.timestamp() if hasattr(msg.log_time, "timestamp") \
        else msg.log_time / 1e9


def decode_pointcloud2(m):
    """Minimal PointCloud2 -> Nx3 xyz decoder (assumes float32 x,y,z present)."""
    field_offsets = {f.name: f.offset for f in m.fields}
    if not all(k in field_offsets for k in ("x", "y", "z")):
        return np.zeros((0, 3), dtype=np.float32)
    ox, oy, oz = field_offsets["x"], field_offsets["y"], field_offsets["z"]
    step = m.point_step
    n = m.width * m.height
    data = bytes(m.data)
    pts = np.zeros((n, 3), dtype=np.float32)
    for i in range(n):
        base = i * step
        pts[i, 0] = struct.unpack_from("<f", data, base + ox)[0]
        pts[i, 1] = struct.unpack_from("<f", data, base + oy)[0]
        pts[i, 2] = struct.unpack_from("<f", data, base + oz)[0]
    return pts


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


odom_t, odom_xyz, odom_yaw = [], [], []
slam_t, slam_xyz, slam_yaw = [], [], []
ekf_t, ekf_xy = [], []
mapc_t, mapc_pts = [], []

topics = ["/bluerov2/odometry", "/bluerov2/robot_pose_slam",
          "/bluerov2/robot_pose_slam_ekf", "/bluerov2/map_points"]

n_seen = 0
for f in mcap_files:
    for msg in read_ros2_messages(f, topics=topics):
        n_seen += 1
        if n_seen % 20000 == 0:
            print(f"  ...{n_seen} messages processed")
        m = msg.ros_msg
        t = ts(msg)
        if msg.channel.topic == "/bluerov2/odometry":
            p = m.pose.pose.position
            odom_t.append(t)
            odom_xyz.append((p.x, p.y, p.z))
            odom_yaw.append(yaw_from_quat(m.pose.pose.orientation))
        elif msg.channel.topic == "/bluerov2/robot_pose_slam":
            p = m.pose.position
            slam_t.append(t)
            slam_xyz.append((p.x, p.y, p.z))
            slam_yaw.append(yaw_from_quat(m.pose.orientation))
        elif msg.channel.topic == "/bluerov2/robot_pose_slam_ekf":
            p = m.pose.pose.position
            ekf_t.append(t)
            ekf_xy.append((p.x, p.y))
        elif msg.channel.topic == "/bluerov2/map_points":
            pts = decode_pointcloud2(m)
            mapc_t.append(t)
            mapc_pts.append(pts)

print(f"odometry: {len(odom_t)}  slam: {len(slam_t)}  "
      f"slam_ekf: {len(ekf_t)}  map_points snapshots: {len(mapc_t)}")
print("map_points sizes (first 10):", [len(p) for p in mapc_pts[:10]])
print("map_points sizes (last 10):", [len(p) for p in mapc_pts[-10:]])

np.savez(
    OUT,
    odom_t=np.array(odom_t), odom_xyz=np.array(odom_xyz), odom_yaw=np.array(odom_yaw),
    slam_t=np.array(slam_t), slam_xyz=np.array(slam_xyz), slam_yaw=np.array(slam_yaw),
    ekf_t=np.array(ekf_t), ekf_xy=np.array(ekf_xy),
    mapc_t=np.array(mapc_t),
    mapc_pts=np.array(mapc_pts, dtype=object),
)
print(f"saved {OUT}")
