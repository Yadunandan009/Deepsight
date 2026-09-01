#!/usr/bin/env python3
"""
DeepSight — Mission Analysis Plot Script
Generates:
  1. Mission analysis (trajectory, depth, distance, SLAM drift/ATE)
  2. Formal metrics (ATE, RPE, drift vs distance)
  3. SLAM map PLY extraction
  4. Sonar point cloud
"""
import math, glob, os, sys, struct
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

try:
    from mcap_ros2.reader import read_ros2_messages
except ImportError:
    print("Install: pip3 install mcap-ros2-support --break-system-packages")
    sys.exit(1)

# ── Config ────────────────────────────────────────────────────────
TURBINE_X, TURBINE_Y = 20.0, 0.0
SCAN_DIST            = 17.0
OUT_DIR              = os.path.expanduser('~/ros2_ws/plots')
os.makedirs(OUT_DIR, exist_ok=True)

# ── Find best bag ─────────────────────────────────────────────────
search_dirs = [
    os.path.expanduser('~/ros2_ws/bags/'),
    '/media/yadunandan/more space/bags/',
    '/media/yadunandan/7D52-4158/',
]

all_dirs = []
for d in search_dirs:
    all_dirs += glob.glob(d + 'DeepSight_*/') + glob.glob(d + 'DeepSight_v5_*/')

all_dirs = sorted(set(all_dirs), key=os.path.getmtime)
all_dirs = [d for d in all_dirs if 'TEST' not in d]

bag_dir = None
best_size = 0
for d in all_dirs:
    candidates = [f for f in glob.glob(d + '*.mcap')
                  if os.path.getsize(f) < 4e9]
    total = sum(os.path.getsize(f) for f in candidates)
    if total > best_size:
        best_size = total
        bag_dir = d

if not bag_dir:
    print("No bags found")
    sys.exit(1)

mcap_files = sorted([f for f in glob.glob(bag_dir + '*.mcap')
                     if os.path.getsize(f) < 4e9])
print(f"Bag: {bag_dir}")
print(f"Files: {len(mcap_files)}, Total: {best_size/1e9:.2f} GB")

# ── Data containers ───────────────────────────────────────────────
ins_yaw_latest                  = 0.0
ins_x, ins_y, ins_z, ins_t      = [], [], [], []
slam_x, slam_y, slam_z, slam_t  = [], [], [], []
mono_x, mono_y, mono_z, mono_t  = [], [], [], []
sonar_ranges, sonar_t           = [], []
sonar_pts                       = []
last_map_msg                    = None
map_count                       = 0

def ts(msg):
    return msg.log_time.timestamp() if hasattr(msg.log_time, 'timestamp') \
           else msg.log_time / 1e9

for mcap_file in mcap_files:
    try:
        for msg in read_ros2_messages(mcap_file):
            t   = ts(msg)
            top = msg.channel.topic
            m   = msg.ros_msg

            if top == '/bluerov2/odometry':
                # Raw dead-reckoned INS (IMU+DVL+pressure), not the EKF's
                # fused output -- '/bluerov2/odometry/filtered' isn't even
                # recorded by record_mission.sh, and semantically the
                # fused topic includes SLAM's own contribution, which
                # would make ATE/RPE partly compare SLAM against itself.
                ins_t.append(t)
                ins_x.append(m.pose.pose.position.x)
                ins_y.append(m.pose.pose.position.y)
                ins_z.append(m.pose.pose.position.z)
                q = m.pose.pose.orientation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z
                ins_yaw_latest = math.atan2(siny, cosy)

            elif top == '/bluerov2/robot_pose_slam_ekf':
                # slam_pose_bridge's rigid-aligned (world_ned-frame) SLAM
                # pose -- NOT raw '/bluerov2/robot_pose_slam', which is
                # in SLAM's own arbitrary map frame and would make ATE
                # mostly measure the frame misalignment rather than
                # actual localization drift. PoseWithCovarianceStamped,
                # so position is nested one level deeper than the raw
                # PoseStamped topic it replaces.
                slam_t.append(t)
                slam_x.append(m.pose.pose.position.x)
                slam_y.append(m.pose.pose.position.y)
                slam_z.append(-m.pose.pose.position.z)

            elif top in ('/bluerov2/multibeam_raw', '/bluerov2/multibeam'):
                ranges = list(m.ranges)
                valid  = [r for r in ranges if 0.1 < r < m.range_max]
                if not valid:
                    continue
                sonar_t.append(t)
                sonar_ranges.append(min(valid))
                if ins_x:
                    rx, ry, rz = ins_x[-1], ins_y[-1], ins_z[-1]
                    # Real heading from the INS orientation quaternion --
                    # NOT direction-of-travel between consecutive position
                    # samples (the old approach). This vehicle routinely
                    # translates sideways relative to its own heading
                    # (CLOSE_IN crabs in, ORBIT moves tangentially while
                    # nose stays locked on the turbine, SCAN is near-
                    # stationary) so velocity direction and heading are
                    # essentially never the same thing here -- that
                    # mismatch was rotating every sonar ping by the wrong
                    # angle, which is what smeared the reconstruction into
                    # a spiral instead of a clean structure.
                    dyaw = ins_yaw_latest
                    a_min = m.angle_min
                    a_inc = m.angle_increment
                    for i, r in enumerate(ranges):
                        if not (0.1 < r < m.range_max):
                            continue
                        beam = a_min + i * a_inc
                        bx   = r * math.cos(beam)
                        by   = r * math.sin(beam)
                        wx   = rx + bx * math.cos(dyaw) - by * math.sin(dyaw)
                        wy   = ry + bx * math.sin(dyaw) + by * math.cos(dyaw)
                        sonar_pts.append((wx, wy, rz, rz))

            elif top == '/bluerov2/map_points':
                last_map_msg = m
                map_count += 1

    except Exception as e:
        print(f"SKIP {os.path.basename(mcap_file)}: {e}")

print(f"INS: {len(ins_x)} | SLAM: {len(slam_x)} | "
      f"Sonar: {len(sonar_t)} | MapMsgs: {map_count}")

if not ins_x:
    print("No INS data — check topic names")
    sys.exit(1)

# ── Normalise time ────────────────────────────────────────────────
t0      = ins_t[0]
ins_t   = [t - t0 for t in ins_t]
slam_t  = [t - t0 for t in slam_t]
mono_t  = [t - t0 for t in mono_t]
sonar_t = [t - t0 for t in sonar_t]

# ── Align SLAM to INS frame ───────────────────────────────────────
if slam_x and ins_x:
    ix0 = np.interp(slam_t[0], ins_t, ins_x)
    iy0 = np.interp(slam_t[0], ins_t, ins_y)
    iz0 = np.interp(slam_t[0], ins_t, ins_z)
    off_x = ix0 - slam_x[0]
    off_y = iy0 - slam_y[0]
    off_z = iz0 - slam_z[0]
    slam_x = [x + off_x for x in slam_x]
    slam_y = [y + off_y for y in slam_y]
    slam_z = [z + off_z for z in slam_z]
    print(f"SLAM offset: dx={off_x:.2f} dy={off_y:.2f} dz={off_z:.2f}")

# ── Compute ATE ───────────────────────────────────────────────────
ate_err = []
if slam_x and ins_x:
    ix = np.interp(slam_t, ins_t, ins_x)
    iy = np.interp(slam_t, ins_t, ins_y)
    iz = np.interp(slam_t, ins_t, ins_z)
    ate_err = [math.sqrt((sx-x)**2 + (sy-y)**2 + (sz-z)**2)
               for sx, sy, sz, x, y, z in
               zip(slam_x, slam_y, slam_z, ix, iy, iz)]
    ate_mean = np.mean(ate_err)
    ate_max  = np.max(ate_err)
    ate_rms  = np.sqrt(np.mean(np.array(ate_err)**2))
    print(f"ATE — mean={ate_mean:.3f}m  max={ate_max:.3f}m  RMS={ate_rms:.3f}m")

# ── Compute RPE ───────────────────────────────────────────────────
def compute_rpe(est_x, est_y, gt_x, gt_y, t, delta_t=1.0):
    """Relative Pose Error over a fixed TIME window (TUM RGB-D
    convention), not a fixed sample count -- SLAM and INS publish at
    different, uneven rates, so a fixed sample-count delta corresponds
    to a different real time span depending on which series' indices
    it's applied to and how dense that stretch of data happens to be.
    est/gt must already share the same timestamps `t` (both resampled
    onto slam_t, as ix/iy already are for ATE)."""
    errors = []
    n = len(t)
    j = 0
    for i in range(n):
        target_t = t[i] + delta_t
        if target_t > t[-1]:
            break
        while j < n - 1 and t[j] < target_t:
            j += 1
        if j <= i:
            continue
        de_x = est_x[j] - est_x[i]
        de_y = est_y[j] - est_y[i]
        dg_x = gt_x[j] - gt_x[i]
        dg_y = gt_y[j] - gt_y[i]
        errors.append(math.sqrt((de_x-dg_x)**2 + (de_y-dg_y)**2))
    return np.sqrt(np.mean(np.array(errors)**2)) if errors else 0.0

rpe = 0.0
if slam_x and ins_x:
    # Reuse ix/iy (INS already resampled onto slam_t for ATE, above) --
    # both series now share timestamps across the FULL mission, not the
    # truncated first-N-samples slice the old interpolation produced.
    rpe = compute_rpe(slam_x, slam_y, list(ix), list(iy), slam_t, delta_t=1.0)
    print(f"RPE (Δt=1.0s): {rpe:.3f}m")

# ── Compute drift vs distance ─────────────────────────────────────
dist_travelled = [0.0]
for i in range(1, len(ins_x)):
    d = math.sqrt((ins_x[i]-ins_x[i-1])**2 + (ins_y[i]-ins_y[i-1])**2)
    dist_travelled.append(dist_travelled[-1] + d)

slam_dist = []
slam_ate_at_dist = []
if ate_err:
    slam_dist_interp = np.interp(slam_t, ins_t, dist_travelled)
    slam_dist = list(slam_dist_interp)
    slam_ate_at_dist = ate_err

# ═══════════════════════════════════════════════════════════════════
# FIGURE 1 — Mission Analysis
# ═══════════════════════════════════════════════════════════════════
fig1, axes = plt.subplots(2, 2, figsize=(16, 11))
fig1.suptitle('DeepSight — Mission Analysis', fontsize=15, fontweight='bold')

# Top-down trajectory
ax = axes[0, 0]
ax.plot(ins_y, ins_x, 'b-', lw=2, label='Ground Truth (INS)')
if mono_x:
    ax.plot(mono_y, mono_x, 'm-', lw=1, alpha=0.6, label='Mono SLAM (down)')
if slam_x:
    ax.plot(slam_y, slam_x, 'r--', lw=1.5, label='DeepSight (SLAM)')
ax.plot(ins_y[0],  ins_x[0],  'go', ms=10, label='Start')
ax.plot(ins_y[-1], ins_x[-1], 'rs', ms=10, label='End')
ax.plot(TURBINE_Y, TURBINE_X, 'k^', ms=12, label='Turbine')
circle = plt.Circle((TURBINE_Y, TURBINE_X), SCAN_DIST,
                     fill=False, color='green', ls='--', lw=1.5,
                     label=f'Orbit r={SCAN_DIST}m')
ax.add_patch(circle)
ax.set_xlabel('East/Y [m]')
ax.set_ylabel('North/X [m]')
ax.set_title('Top-down Trajectory')
ax.legend(fontsize=8)
ax.grid(True)
ax.set_aspect('equal', adjustable='datalim')

# Depth profile
ax = axes[0, 1]
ax.plot(ins_t,  ins_z,  'b-',  lw=2,   label='INS Depth')
if mono_t:
    ax.plot(mono_t, mono_z, 'm-', lw=1, alpha=0.6, label='Mono SLAM Depth')
if slam_x:
    ax.plot(slam_t, slam_z, 'r--', lw=1.5, label='SLAM Depth')
ax.axhline(y=1,  color='cyan',   ls=':',  lw=1, label='Surface ~1m')
ax.axhline(y=18, color='orange', ls='--', lw=1, label='Safe 18m')
ax.axhline(y=23, color='red',    ls='--', lw=1, label='Base 23m')
ax.axhline(y=27, color='pink',   ls='--', lw=1, label='E-Stop 27m')
ax.set_xlabel('Time [s]')
ax.set_ylabel('Depth [m]')
ax.set_title('Depth Profile')
ax.legend(fontsize=8)
ax.grid(True)
ax.invert_yaxis()

# Distance to turbine
ax = axes[1, 0]
dist_ins = [math.sqrt((x-TURBINE_X)**2 + y**2)
            for x, y in zip(ins_x, ins_y)]
ax.plot(ins_t, dist_ins, 'b-', lw=2, label='INS Dist to Turbine')
if slam_x:
    dist_slam = [math.sqrt((x-TURBINE_X)**2 + y**2)
                 for x, y in zip(slam_x, slam_y)]
    ax.plot(slam_t, dist_slam, 'r--', lw=1.5, label='SLAM Dist')
if sonar_t:
    ax.plot(sonar_t, sonar_ranges, 'g-', lw=1, alpha=0.6, label='Sonar Range')
ax.axhspan(0, 3.5,            alpha=0.15, color='red',   label='Collision 3.5m')
ax.axhspan(SCAN_DIST-1, SCAN_DIST+1, alpha=0.10, color='green', label=f'Target r={SCAN_DIST}')
ax.set_xlabel('Time [s]')
ax.set_ylabel('Distance [m]')
ax.set_title('Distance to Turbine + Sonar Range')
ax.legend(fontsize=8)
ax.grid(True)

# ATE (SLAM drift)
ax = axes[1, 1]
if ate_err:
    ax.plot(slam_t, ate_err, color='purple', lw=2)
    ax.fill_between(slam_t, ate_err, alpha=0.2, color='purple')
    stats = (f"ATE Mean : {ate_mean:.3f} m\n"
             f"ATE Max  : {ate_max:.3f} m\n"
             f"ATE RMS  : {ate_rms:.3f} m\n"
             f"RPE      : {rpe:.3f} m")
    ax.text(0.98, 0.05, stats, transform=ax.transAxes,
            ha='right', va='bottom', fontsize=10,
            bbox=dict(boxstyle='round', facecolor='lavender', alpha=0.8))
    ax.set_title(f'SLAM Drift vs INS (ATE max={ate_max:.2f}m, mean={ate_mean:.2f}m)')
else:
    ax.text(0.5, 0.5, 'No SLAM data', transform=ax.transAxes,
            ha='center', va='center', fontsize=14, color='gray')
    ax.set_title('SLAM Drift vs INS')
ax.set_xlabel('Time [s]')
ax.set_ylabel('3D Error [m]')
ax.grid(True)

plt.tight_layout()
out1 = os.path.join(OUT_DIR, 'mission_analysis.png')
fig1.savefig(out1, dpi=150, bbox_inches='tight')
print(f"Saved: {out1}")
plt.close(fig1)

# ═══════════════════════════════════════════════════════════════════
# FIGURE 2 — Formal Metrics
# ═══════════════════════════════════════════════════════════════════
if ate_err and slam_dist:
    fig2, axes2 = plt.subplots(1, 2, figsize=(14, 6))
    fig2.suptitle('DeepSight — Formal Navigation Metrics',
                  fontsize=14, fontweight='bold')

    # Drift vs distance travelled
    ax = axes2[0]
    ax.plot(slam_dist, slam_ate_at_dist, color='purple', lw=2)
    ax.fill_between(slam_dist, slam_ate_at_dist, alpha=0.2, color='purple')
    ax.set_xlabel('Distance Travelled [m]')
    ax.set_ylabel('3D ATE [m]')
    ax.set_title('SLAM Drift vs Distance Travelled')
    ax.grid(True)

    # RPE over different time windows
    ax = axes2[1]
    deltas = [0.5, 1.0, 2.0, 5.0, 10.0]
    rpe_vals = [compute_rpe(slam_x, slam_y, list(ix), list(iy), slam_t, d)
                for d in deltas]
    ax.bar([f"{d}s" for d in deltas], rpe_vals, color='steelblue', alpha=0.8)
    ax.set_xlabel('Window Size [s]')
    ax.set_ylabel('RPE [m]')
    ax.set_title('Relative Pose Error vs Time Window')
    ax.grid(True, axis='y')

    plt.tight_layout()
    out2 = os.path.join(OUT_DIR, 'formal_metrics.png')
    fig2.savefig(out2, dpi=150, bbox_inches='tight')
    print(f"Saved: {out2}")
    plt.close(fig2)

# ═══════════════════════════════════════════════════════════════════
# FIGURE 3 — Sonar Point Cloud
# ═══════════════════════════════════════════════════════════════════
if sonar_pts:
    pts = np.array(sonar_pts)
    wx, wy, wz, dep = pts[:,0], pts[:,1], pts[:,2], pts[:,3]

    fig3, axes3 = plt.subplots(1, 2, figsize=(16, 7))
    fig3.suptitle('DeepSight — Sonar Point Cloud (World Frame)',
                  fontsize=14, fontweight='bold')

    ax = axes3[0]
    sc = ax.scatter(wx, wz, c=dep, cmap='plasma', s=0.5, alpha=0.6)
    if ins_x:
        ax.plot(ins_x, ins_z, 'c-', lw=1.5, label='INS path (X-Z)')
        ax.legend(fontsize=9)
    plt.colorbar(sc, ax=ax, label='Depth [m]')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Z [m]')
    ax.set_title('Sonar Side Projection (X-Z)')
    ax.grid(True, alpha=0.3)
    ax.invert_yaxis()

    ax = axes3[1]
    sc = ax.scatter(wy, wx, c=dep, cmap='plasma', s=0.5, alpha=0.6)
    if ins_x:
        ax.plot(ins_y, ins_x, 'c-', lw=1.5, label='INS path')
    ax.plot(TURBINE_Y, TURBINE_X, 'r^', ms=12, label='Turbine')
    circle2 = plt.Circle((TURBINE_Y, TURBINE_X), SCAN_DIST,
                          fill=False, color='gray', ls='--', lw=1.5)
    ax.add_patch(circle2)
    plt.colorbar(sc, ax=ax, label='Depth [m]')
    ax.set_xlabel('Y [m]')
    ax.set_ylabel('X [m]')
    ax.set_title('Top-down Sonar Projection (coloured by depth)')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal', adjustable='datalim')

    plt.tight_layout()
    out3 = os.path.join(OUT_DIR, 'sonar_cloud.png')
    fig3.savefig(out3, dpi=150, bbox_inches='tight')
    print(f"Saved: {out3}")
    plt.close(fig3)

    # Save sonar PLY
    ply_sonar = os.path.join(OUT_DIR, 'sonar_reconstruction.ply')
    with open(ply_sonar, 'w') as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(sonar_pts)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("end_header\n")
        for (x, y, z, _) in sonar_pts:
            f.write(f"{x:.4f} {y:.4f} {z:.4f}\n")
    print(f"Saved: {ply_sonar}")

# ═══════════════════════════════════════════════════════════════════
# Extract SLAM map PLY
# ═══════════════════════════════════════════════════════════════════
if last_map_msg:
    m = last_map_msg
    slam_pts = []
    data = bytes(m.data)
    for i in range(m.width * m.height):
        offset = i * m.point_step
        x, y, z = struct.unpack_from('fff', data, offset)
        if not (x == 0.0 and y == 0.0 and z == 0.0):
            slam_pts.append((x, y, -z))
    print(f"SLAM map points: {len(slam_pts)}")
    ply_slam = os.path.join(OUT_DIR, 'slam_map.ply')
    with open(ply_slam, 'w') as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(slam_pts)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("end_header\n")
        for x, y, z in slam_pts:
            f.write(f"{x:.4f} {y:.4f} {z:.4f}\n")
    print(f"Saved: {ply_slam}")

# ═══════════════════════════════════════════════════════════════════
# Print summary
# ═══════════════════════════════════════════════════════════════════
print("\n" + "="*50)
print("DEEPSIGHT MISSION SUMMARY")
print("="*50)
if ins_t:
    print(f"Mission duration  : {ins_t[-1]:.1f} s")
    print(f"Distance travelled: {dist_travelled[-1]:.1f} m")
if ate_err:
    print(f"ATE mean          : {ate_mean:.3f} m")
    print(f"ATE max           : {ate_max:.3f} m")
    print(f"ATE RMS           : {ate_rms:.3f} m")
    print(f"RPE (Δt=1.0s)     : {rpe:.3f} m")
if last_map_msg:
    print(f"SLAM map points   : {len(slam_pts)}")
print(f"Sonar points      : {len(sonar_pts)}")
print(f"Output dir        : {OUT_DIR}")
print("="*50)
