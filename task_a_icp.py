#!/usr/bin/env python3
"""
Task A — ICP loop-closure verification.

Tests the suspected face-1/face-2 false loop closure: does ORB-SLAM3's
map data for face 2, once transformed into world frame, actually match
face 2's real geometry -- or does it (wrongly) match face 1's?

Method:
  1. For each face, find its SCAN time-window from ground-truth odometry
     (r ~= SCAN_DIST from turbine center, bearing ~= FACE_BEARINGS[face]).
  2. /map_points is a CUMULATIVE map, so isolate what's genuinely new
     during a face's window: (map at end of window) minus (map at start),
     by nearest-neighbor distance thresholding.
  3. Recover the SLAM-frame -> world-frame transform for that window
     empirically: robot_pose_slam (SLAM frame) vs robot_pose_slam_ekf
     (world frame, output of slam_pose_bridge's rotation-only Procrustes
     fit) give paired correspondences: solve the same closed-form 2D
     rotation+translation fit locally, using only that window's data
     (the live transform is continuously refit, so a single global R,t
     would be wrong).
  4. Reference geometry per face: sonar reconstruction (independent of
     ORB-SLAM3 entirely), restricted to that face's angular sector
     around the turbine center.
  5. Manual point-to-point ICP (KDTree correspondence + Kabsch/SVD),
     since open3d isn't installed and this is a small, well-scoped
     algorithm.
  6. Report fitness (inlier fraction under a distance threshold) and
     RMSE for: face2-new vs face1-ref (suspected false match),
     face1-new vs face1-ref (control), face2-new vs face2-ref (sanity).
"""
import math
import numpy as np
from scipy.spatial import cKDTree

D = np.load("/home/yadunandan/ros2_ws/plots/task_a_topics.npz", allow_pickle=True)
odom_t, odom_xyz, odom_yaw = D['odom_t'], D['odom_xyz'], D['odom_yaw']
slam_t, slam_xyz, slam_yaw = D['slam_t'], D['slam_xyz'], D['slam_yaw']
mapc_t, mapc_pts = D['mapc_t'], D['mapc_pts']
slam_xy = slam_xyz[:, :2]

order = np.argsort(odom_t); odom_t, odom_xyz, odom_yaw = odom_t[order], odom_xyz[order], odom_yaw[order]
order = np.argsort(slam_t); slam_t, slam_xyz, slam_yaw = slam_t[order], slam_xyz[order], slam_yaw[order]
slam_xy = slam_xyz[:, :2]
order = np.argsort(mapc_t); mapc_t = mapc_t[order]; mapc_pts = mapc_pts[order]

TURBINE_X, TURBINE_Y = 20.0, 0.0
SCAN_DIST = 17.0
FACE_BEARINGS = [180.0, 90.0, 0.0, 270.0]


def face_window(face_idx, t_lo_bound=None, t_hi_bound=None):
    """Return (t_start, t_end) of the SCAN dwell for a face, restricted to
    an optional [t_lo_bound, t_hi_bound] search range to avoid picking up
    face 0's second (return) visit."""
    dx = odom_xyz[:, 0] - TURBINE_X
    dy = odom_xyz[:, 1] - TURBINE_Y
    r = np.hypot(dx, dy)
    bearing = np.degrees(np.arctan2(dy, dx)) % 360
    fb = FACE_BEARINGS[face_idx]
    ang_diff = np.abs((bearing - fb + 180) % 360 - 180)
    mask = (np.abs(r - SCAN_DIST) < 2.5) & (ang_diff < 20)
    if t_lo_bound is not None:
        mask &= odom_t >= t_lo_bound
    if t_hi_bound is not None:
        mask &= odom_t <= t_hi_bound
    idxs = np.nonzero(mask)[0]
    return odom_t[idxs[0]], odom_t[idxs[-1]]


def nearest_mapc_idx(t):
    return int(np.searchsorted(mapc_t, t))


def reject_outliers(pts, t_start, t_end, margin=25.0):
    """ORB-SLAM3's raw map has badly-triangulated points scattered far
    beyond anything physically plausible (confirmed: map point range is
    10-70x the vehicle trajectory's own range in the same frame,
    including z -- one snapshot had z in [-72, 68] vs a physically real
    world z of [0, 22]). Keep only points within `margin` of the
    vehicle's actual SLAM-frame trajectory in full 3D -- an XY-only
    filter lets severe z outliers straight through, which then corrupts
    every downstream 3D distance comparison."""
    m = (slam_t >= t_start - 10) & (slam_t <= t_end + 10)
    traj = slam_xyz[m]
    if len(traj) == 0 or len(pts) == 0:
        return pts
    tree = cKDTree(traj)
    dist, _ = tree.query(pts[:, :3], k=1)
    return pts[dist < margin]


def new_points_in_window(t_start, t_end):
    """map_points at end of window minus map_points at (or just before)
    start of window -> points genuinely new during this face's dwell."""
    i0 = max(0, nearest_mapc_idx(t_start) - 1)
    i1 = min(len(mapc_t) - 1, nearest_mapc_idx(t_end))
    before = np.asarray(mapc_pts[i0], dtype=np.float64)
    after = np.asarray(mapc_pts[i1], dtype=np.float64)
    before = reject_outliers(before, t_start, t_end)
    after = reject_outliers(after, t_start, t_end)
    if len(before) == 0:
        return after, mapc_t[i0], mapc_t[i1], len(before), len(after)
    tree = cKDTree(before)
    dist, _ = tree.query(after, k=1)
    new_mask = dist > 0.3
    return after[new_mask], mapc_t[i0], mapc_t[i1], len(before), len(after)


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def reanchor_points(pts_slam, t_start, t_end):
    """Per-point re-anchoring instead of a single rigid-transform fit.

    Both the per-window Procrustes fit (ill-conditioned: SCAN dwells are
    nearly collinear, eigenvalue ratio ~37x) and the whole-mission global
    fit (well-conditioned but wrong: 16.5m mean residual, because the
    SLAM->world relationship genuinely isn't a single rigid transform
    over the whole mission) failed. Instead: assign each map point to its
    nearest SLAM-frame trajectory sample, read that instant's actual
    heading offset directly (ground-truth odometry yaw minus SLAM's own
    yaw -- no curve-fitting, no conditioning problem), and apply that
    local rotation+translation. robot_pose_slam_ekf's orientation field
    is NOT usable for this: slam_pose_bridge.py copies the whole pose
    (`out.pose.pose = m.pose`) and only overwrites x/y, so its
    orientation is just the raw untransformed SLAM orientation.
    """
    m = (slam_t >= t_start - 15) & (slam_t <= t_end + 15)
    traj_xy = slam_xy[m]
    traj_t = slam_t[m]
    traj_yaw = slam_yaw[m]
    traj_z = slam_xyz[m][:, 2]
    if len(traj_xy) == 0 or len(pts_slam) == 0:
        return np.zeros((0, 3))
    tree = cKDTree(traj_xy)
    _, anchor_idx = tree.query(pts_slam[:, :2], k=1)
    anchor_t = traj_t[anchor_idx]
    anchor_slam_xy = traj_xy[anchor_idx]
    anchor_slam_z = traj_z[anchor_idx]
    anchor_slam_yaw = traj_yaw[anchor_idx]

    oi = np.clip(np.searchsorted(odom_t, anchor_t), 0, len(odom_t) - 1)
    anchor_world_xy = odom_xyz[oi, :2]
    anchor_world_z = odom_xyz[oi, 2]
    anchor_world_yaw = odom_yaw[oi]

    theta = wrap_angle(anchor_world_yaw - anchor_slam_yaw)
    cos_t, sin_t = np.cos(theta), np.sin(theta)
    rel = pts_slam[:, :2] - anchor_slam_xy
    rx = cos_t * rel[:, 0] - sin_t * rel[:, 1]
    ry = sin_t * rel[:, 0] + cos_t * rel[:, 1]
    world_xy = np.column_stack([rx, ry]) + anchor_world_xy
    world_z = (pts_slam[:, 2] - anchor_slam_z) + anchor_world_z
    pts_out = np.column_stack([world_xy, world_z])

    # Hard physical sanity bound: the vehicle/turbine cannot be outside
    # the world's actual depth range. A 25m 3D outlier margin still lets
    # a point be up to 25m off in z alone relative to its anchor (the
    # trajectory itself spans a wide depth range across the +-15s
    # window) -- this catches what that margin doesn't.
    z_lo, z_hi = odom_xyz[:, 2].min() - 2, odom_xyz[:, 2].max() + 2
    keep = (world_z > z_lo) & (world_z < z_hi)
    return pts_out[keep], theta[keep]


def icp_bootstrap(src, dst, n_boot=50, frac=0.8, seed=0):
    """Bootstrap: resample `frac` of src (with replacement) n_boot times,
    rerun ICP each time, report mean+-std of fitness/RMSE. dst (the
    reference) is fixed -- only the measured/source side is resampled,
    which is what a bootstrap over sampling noise in the measurement
    should do."""
    rng = np.random.default_rng(seed)
    n = max(10, int(len(src) * frac))
    fits, rmses = [], []
    for _ in range(n_boot):
        idx = rng.choice(len(src), size=n, replace=True)
        f, r = icp_point_to_point(src[idx], dst)
        fits.append(f); rmses.append(r)
    fits, rmses = np.array(fits), np.array(rmses)
    return fits.mean(), fits.std(), rmses.mean(), rmses.std()


def icp_point_to_point(src, dst, max_iters=50, tol=1e-5, inlier_thresh=1.0):
    """Minimal point-to-point ICP (3D). Returns fitness, rmse."""
    if len(src) == 0 or len(dst) < 10:
        return 0.0, float('inf')
    src_pts = src.copy()
    tree = cKDTree(dst)
    prev_rmse = None
    for _ in range(max_iters):
        dist, idx = tree.query(src_pts, k=1)
        matched = dst[idx]
        src_c = src_pts.mean(axis=0)
        dst_c = matched.mean(axis=0)
        A = src_pts - src_c
        B = matched - dst_c
        H = A.T @ B
        U, S, Vt = np.linalg.svd(H)
        Rm = Vt.T @ U.T
        if np.linalg.det(Rm) < 0:
            Vt[-1, :] *= -1
            Rm = Vt.T @ U.T
        t_ = dst_c - Rm @ src_c
        src_pts = src_pts @ Rm.T + t_
        rmse = np.sqrt(np.mean(np.sum((src_pts - matched) ** 2, axis=1)))
        if prev_rmse is not None and abs(prev_rmse - rmse) < tol:
            break
        prev_rmse = rmse
    dist, _ = tree.query(src_pts, k=1)
    fitness = float(np.mean(dist < inlier_thresh))
    inlier_rmse = float(np.sqrt(np.mean(dist[dist < inlier_thresh] ** 2))) if fitness > 0 else float('inf')
    return fitness, inlier_rmse


# ── Reference geometry: sonar cloud, split by face angular sector ──
sonar = np.load("/home/yadunandan/ros2_ws/plots/_mosaic_pts.npy")
dx = sonar[:, 0] - TURBINE_X
dy = sonar[:, 1] - TURBINE_Y
r_s = np.hypot(dx, dy)
sonar = sonar[r_s < 30]  # drop the stray off-structure cluster (anode etc.)
dx = sonar[:, 0] - TURBINE_X
dy = sonar[:, 1] - TURBINE_Y
bearing_s = np.degrees(np.arctan2(dy, dx)) % 360


def face_reference(face_idx, half_width=35):
    fb = FACE_BEARINGS[face_idx]
    ang_diff = np.abs((bearing_s - fb + 180) % 360 - 180)
    return sonar[ang_diff < half_width]


HFOV_DEG = 75.0          # camera HFOV, from extract_splat_data.py's scenario-derived config
CAM_MAX_RANGE = 22.0      # m -- generous vs SCAN_DIST=17, matches extract_splat_data's usable range


def face_reference_frustum(face_idx, t_start, t_end, half_width=35):
    """Tighter reference: instead of the whole angular sector, keep only
    sonar points that fall within the camera's actual HFOV cone and
    range from at least one anchor pose during the window. Camera
    azimuth is approximated as pointing at the turbine center -- this is
    not a simplification of convenience, it's what bearing_hold() in the
    controller actually enforces throughout CLOSE_IN/SCAN (confirmed in
    bluerov2_autonomous_controller.py), so it's a faithful model of
    where the camera was actually looking, without needing to
    reconstruct full IMU orientation."""
    base_ref = face_reference(face_idx, half_width=half_width)
    m = (odom_t >= t_start) & (odom_t <= t_end)
    anchors_xy = odom_xyz[m][:, :2]
    if len(anchors_xy) == 0:
        return base_ref
    anchors_xy = anchors_xy[::max(1, len(anchors_xy) // 60)]  # thin to ~60 anchors, plenty for coverage

    visible = np.zeros(len(base_ref), dtype=bool)
    turbine_xy = np.array([TURBINE_X, TURBINE_Y])
    for ax, ay in anchors_xy:
        cam_forward = math.atan2(turbine_xy[1] - ay, turbine_xy[0] - ax)
        rel = base_ref[:, :2] - np.array([ax, ay])
        rng = np.hypot(rel[:, 0], rel[:, 1])
        az = np.arctan2(rel[:, 1], rel[:, 0])
        ang_diff = np.degrees(np.abs(np.arctan2(np.sin(az - cam_forward), np.cos(az - cam_forward))))
        visible |= (rng < CAM_MAX_RANGE) & (ang_diff < HFOV_DEG / 2)
    return base_ref[visible]


# ── Run ──
print("=== Face windows ===")
f1_start, f1_end = face_window(1)
f2_start, f2_end = face_window(2)
for name, (s, e) in [("face1", (f1_start, f1_end)), ("face2", (f2_start, f2_end))]:
    print(f"  {name}: t=[{s:.1f}, {e:.1f}]  dwell={e-s:.1f}s")

print("\n=== New map points added during each face's dwell ===")
new1, b0t, b1t, nb, na = new_points_in_window(f1_start, f1_end)
print(f"  face1: before(t={b0t:.1f})={nb} pts, after(t={b1t:.1f})={na} pts, NEW={len(new1)}")
new2, b0t, b1t, nb, na = new_points_in_window(f2_start, f2_end)
print(f"  face2: before(t={b0t:.1f})={nb} pts, after(t={b1t:.1f})={na} pts, NEW={len(new2)}")

print("\n=== Per-point re-anchoring (nearest-trajectory-sample local heading offset) ===")
new1_world, theta1 = reanchor_points(new1, f1_start, f1_end)
new2_world, theta2 = reanchor_points(new2, f2_start, f2_end)
for name, theta in [("face1", theta1), ("face2", theta2)]:
    td = np.degrees(theta)
    print(f"  {name}: n_pts={len(td)}  heading-offset mean={td.mean():.1f}deg "
          f"std={td.std():.1f}deg  min={td.min():.1f} max={td.max():.1f}")

ref_face1 = face_reference_frustum(1, f1_start, f1_end)
ref_face2 = face_reference_frustum(2, f2_start, f2_end)
print(f"\nreference cloud sizes (frustum-tightened): "
      f"face1_ref={len(ref_face1)} (was {len(face_reference(1))})  "
      f"face2_ref={len(ref_face2)} (was {len(face_reference(2))})")

print("\n=== ICP results (bootstrap, n=50, 80% resample) ===")
results = []
fm, fs, rm, rs = icp_bootstrap(new1_world, ref_face1)
results.append(("face1-new vs face1-ref (CONTROL)", fm, fs, rm, rs))
fm, fs, rm, rs = icp_bootstrap(new2_world, ref_face1)
results.append(("face2-new vs face1-ref (SUSPECTED FALSE MATCH)", fm, fs, rm, rs))
fm, fs, rm, rs = icp_bootstrap(new2_world, ref_face2)
results.append(("face2-new vs face2-ref (SANITY: own geometry)", fm, fs, rm, rs))

print(f"{'Pair':50s} {'Fitness':>16s} {'RMSE(m)':>16s}")
for name, fm, fs, rm, rs in results:
    print(f"{name:50s} {fm:6.3f} +- {fs:5.3f}  {rm:6.3f} +- {rs:5.3f}")
