#!/usr/bin/env python3
"""
DeepSight — Camera splat data extraction.

Decodes a subsampled set of left-camera frames from the mission bag and
computes each frame's EXACT world camera pose from the ground-truth
odometry (the same source the ATE metrics are validated against) plus the
fixed camera mount transform from the scenario file. Writes frames +
transforms.json (nerfstudio / instant-ngp convention) so no COLMAP
structure-from-motion pose estimation is needed.

Output goes to the external drive (root filesystem is near-full).
"""
import os, sys, glob, json, math
import numpy as np

try:
    from mcap_ros2.reader import read_ros2_messages
except ImportError:
    print("pip3 install mcap-ros2-support --break-system-packages"); sys.exit(1)
try:
    import cv2
except ImportError:
    print("cv2 missing"); sys.exit(1)

# ── Config ─────────────────────────────────────────────────────────
BAG      = os.path.expanduser('~/ros2_ws/bags/DeepSight_v5_20260810_004128')
OUT_DIR  = '/media/yadunandan/more space/splat/camera'
N_TARGET = 1200          # ~target number of views to keep (subsampled)
IMG_TOPIC = '/bluerov2/left/image_color'
ODOM_TOPIC = '/bluerov2/odometry'

# Camera intrinsics from scenario: 640x480, HFOV 75deg, no distortion
W, H = 640, 480
HFOV = math.radians(75.0)
fx = (W / 2.0) / math.tan(HFOV / 2.0)
fy = fx                       # square pixels
cx, cy = W / 2.0, H / 2.0

# Camera mount (base_link -> camera), scenario: xyz + rpy(roll,pitch,yaw)
CAM_XYZ = np.array([0.16, -0.0725, 0.15])
CAM_RPY = np.array([1.571, 0.0, 1.571])

os.makedirs(os.path.join(OUT_DIR, 'images'), exist_ok=True)


def rpy_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy_, sy_ = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy_, -sy_, 0], [sy_, cy_, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def quat_to_R(x, y, z, w):
    n = math.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ])


def ts(msg):
    return msg.log_time.timestamp() if hasattr(msg.log_time, 'timestamp') \
           else msg.log_time / 1e9


# T_base_cam (constant): mount translation + rotation, then convert the
# robot-body camera axes to the OpenGL/nerfstudio camera convention
# (x right, y up, z back i.e. looking down -z). Stonefish/ROS optical
# cameras look down +z of the sensor frame after the rpy mount; the
# body->world uses x-fwd y-left(here NED so y-right)/z conventions, so
# this fixed correction is verified by checking rendered poses point at
# the known turbine. Kept explicit so it's easy to flip if a test
# render shows the cloud mirrored/inverted.
R_bc = rpy_to_R(*CAM_RPY)
T_bc = np.eye(4)
T_bc[:3, :3] = R_bc
T_bc[:3, 3] = CAM_XYZ

# ROS optical (x right, y down, z fwd) -> OpenGL (x right, y up, z back)
OPTICAL_TO_GL = np.diag([1.0, -1.0, -1.0])

mcap_files = sorted(glob.glob(os.path.join(BAG, '*.mcap')))
print(f"Bag files: {len(mcap_files)}")

# Pass 1: collect odometry (t, position, quaternion)
odom_t, odom_pose = [], []
for f in mcap_files:
    for msg in read_ros2_messages(f, topics=[ODOM_TOPIC]):
        m = msg.ros_msg
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        odom_t.append(ts(msg))
        odom_pose.append((p.x, p.y, p.z, q.x, q.y, q.z, q.w))
odom_t = np.array(odom_t)
order = np.argsort(odom_t)
odom_t = odom_t[order]
odom_pose = [odom_pose[i] for i in order]
print(f"Odometry samples: {len(odom_t)}")
if len(odom_t) == 0:
    print("No odometry in bag — cannot compute poses."); sys.exit(1)

# Pass 2: count images, decide subsample stride
n_img = 0
for f in mcap_files:
    for _ in read_ros2_messages(f, topics=[IMG_TOPIC]):
        n_img += 1
stride = max(1, n_img // N_TARGET)
print(f"Images: {n_img} -> keeping every {stride} (~{n_img//stride})")

frames = []
idx = 0
kept = 0
for f in mcap_files:
    for msg in read_ros2_messages(f, topics=[IMG_TOPIC]):
        if idx % stride != 0:
            idx += 1
            continue
        idx += 1
        m = msg.ros_msg
        t = ts(msg)
        # nearest odometry pose
        j = int(np.searchsorted(odom_t, t))
        j = min(max(j, 0), len(odom_t) - 1)
        if j > 0 and abs(odom_t[j-1] - t) < abs(odom_t[j] - t):
            j -= 1
        px, py, pz, qx, qy, qz, qw = odom_pose[j]

        # decode image (rgb8/bgr8 -> BGR for cv2 write)
        buf = np.frombuffer(bytes(m.data), dtype=np.uint8)
        img = buf.reshape(m.height, m.width, 3)
        if m.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        # world<-base
        T_wb = np.eye(4)
        T_wb[:3, :3] = quat_to_R(qx, qy, qz, qw)
        T_wb[:3, 3] = [px, py, pz]
        # world<-camera(optical), then optical->GL for nerfstudio
        T_wc = T_wb @ T_bc
        T_wc[:3, :3] = T_wc[:3, :3] @ OPTICAL_TO_GL

        name = f'images/frame_{kept:05d}.png'
        cv2.imwrite(os.path.join(OUT_DIR, name), img)
        frames.append({
            'file_path': name,
            'transform_matrix': T_wc.tolist(),
        })
        kept += 1

transforms = {
    'w': W, 'h': H,
    'fl_x': fx, 'fl_y': fy, 'cx': cx, 'cy': cy,
    'k1': 0.0, 'k2': 0.0, 'p1': 0.0, 'p2': 0.0,
    'camera_model': 'OPENCV',
    'frames': frames,
}
with open(os.path.join(OUT_DIR, 'transforms.json'), 'w') as fp:
    json.dump(transforms, fp, indent=2)

# quick sanity: mean camera position and mean look-direction endpoint
P = np.array([np.array(fr['transform_matrix'])[:3, 3] for fr in frames])
print(f"Kept {kept} frames")
print(f"Camera position bounds: "
      f"x[{P[:,0].min():.1f},{P[:,0].max():.1f}] "
      f"y[{P[:,1].min():.1f},{P[:,1].max():.1f}] "
      f"z[{P[:,2].min():.1f},{P[:,2].max():.1f}]")
print(f"Turbine is at world (20, 0, 25) — camera ring should surround it.")
print(f"Wrote {os.path.join(OUT_DIR, 'transforms.json')}")
