#!/usr/bin/env python3
"""
DeepSight Adaptive Fusion Node
-------------------------------
Confidence-driven adaptive fusion of ORB-SLAM3 stereo + EKF (DVL+INS+pressure).

Publishes:
  /deepsight/fused_pose      (nav_msgs/Odometry)  — best estimate of ROV pose
  /deepsight/nav_confidence  (std_msgs/Float32)   — 0.0=EKF only, 1.0=SLAM only
  /deepsight/fusion_status   (std_msgs/String)    — human-readable weight reason

Subscribes:
  /bluerov2/odometry/filtered   — EKF pose (DVL + FOG IMU + pressure)
  /bluerov2/robot_pose_slam     — ORB-SLAM3 stereo pose
  /bluerov2/map_points          — PointCloud2 (map point count)

Fusion weight logic:
  w_slam = density_score × age_score × consistency_score

  density_score    : map point count (gate: >= MAP_PTS_MIN, saturate at MAP_PTS_GOOD)
  age_score        : penalise stale SLAM (tracking lost)
  consistency_score: penalise large SLAM/EKF disagreement

  Hard floor: w_slam = 0.0  (pure EKF when SLAM lost or offset not yet set)
  Hard ceil:  w_slam = 0.95 (never fully abandon EKF)

Offset initialization:
  SLAM frame anchored to EKF world frame once STABLE_REQUIRED consecutive
  SLAM messages are received AND map_pts >= MAP_PTS_LOCK_MIN.
  A slow correction (OFFSET_ALPHA) pulls the offset toward EKF when they agree.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, String


# ── Tuneable constants ────────────────────────────────────────────
MAP_PTS_LOCK_MIN  = 50        # minimum map points before offset can be locked
MAP_PTS_MIN       = 5_000     # below this → density_score = 0 (SLAM not trusted)
MAP_PTS_GOOD      = 50_000    # above this → density_score = 1
MAP_PTS_MAX       = 120_000   # saturation (unused, kept for reference)

SLAM_MAX_AGE      = 1.5       # seconds before SLAM considered stale
SLAM_AGE_HALFLIFE = 0.5       # age penalty ramps over this window

CONSISTENCY_HARD  = 10.0      # metres — above this SLAM is dropped entirely
CONSISTENCY_SOFT  = 3.0       # metres — below this no consistency penalty

W_SLAM_FLOOR      = 0.0
W_SLAM_CEIL       = 0.95

JUMP_THRESHOLD    = 5.0       # metres between consecutive SLAM poses → reject
MAX_OFFSET_SHIFT = 3.0   # metres — reject offset re-lock if it swings more than this from the last good offset

OFFSET_ALPHA      = 0.02      # slow drift correction rate (EKF pulls offset)
STABLE_REQUIRED   = 20        # consecutive SLAM frames needed before locking offset
# ─────────────────────────────────────────────────────────────────


class AdaptiveFusionNode(Node):

    def __init__(self):
        super().__init__('adaptive_fusion_node')

        # ── State ────────────────────────────────────────────────
        self.ekf_pose        = None   # (x, y, z) from EKF
        self._ekf_full       = None   # full Odometry msg for passthrough
        self.slam_pose       = None   # (x, y, z) in world frame after offset
        self.slam_t          = None   # timestamp of last SLAM message
        self.map_pts         = 0      # latest map point count

        self.slam_offset_x   = 0.0
        self.slam_offset_y   = 0.0
        self.slam_offset_set = False
        self.stable_count    = 0
        
        self._prev_offset_x = None
        self._prev_offset_y = None

        self.last_slam_x     = None
        self.last_slam_y     = None

        self.w_slam          = 0.0

        # ── QoS ──────────────────────────────────────────────────
        reliable_qos = QoSProfile(depth=10)
        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5)

        # ── Subscribers ──────────────────────────────────────────
        self.create_subscription(
            Odometry,
            '/bluerov2/odometry/filtered',
            self._ekf_cb, reliable_qos)

        self.create_subscription(
            PoseStamped,
            '/bluerov2/robot_pose_slam',
            self._slam_cb, be_qos)

        self.create_subscription(
            PointCloud2,
            '/bluerov2/map_points',
            self._map_cb, be_qos)

        # ── Publishers ───────────────────────────────────────────
        self.pub_pose   = self.create_publisher(
            Odometry, '/deepsight/fused_pose', reliable_qos)
        self.pub_conf   = self.create_publisher(
            Float32, '/deepsight/nav_confidence', reliable_qos)
        self.pub_status = self.create_publisher(
            String, '/deepsight/fusion_status', reliable_qos)

        self.create_timer(0.1, self._publish)

        self.get_logger().info('AdaptiveFusionNode started')

    # ── Callbacks ─────────────────────────────────────────────────
    def _ekf_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.ekf_pose  = (p.x, p.y, p.z)
        self._ekf_full = msg

    def _slam_cb(self, msg: PoseStamped):
        sx = msg.pose.position.x
        sy = msg.pose.position.y
        sz = msg.pose.position.z

        # Jump detection — guard against relocalization artifacts
        if self.last_slam_x is not None:
            jump = math.sqrt((sx - self.last_slam_x)**2 +
                             (sy - self.last_slam_y)**2)
            if jump > JUMP_THRESHOLD:
                self.get_logger().warn(
                    f'SLAM jump {jump:.2f}m — resetting offset')
                self.slam_offset_set = False
                self.stable_count    = 0
                self.last_slam_x     = sx
                self.last_slam_y     = sy
                return

        self.last_slam_x = sx
        self.last_slam_y = sy
        self.slam_t      = time.time()

        # Offset initialization:
        # Wait for STABLE_REQUIRED consecutive frames AND sufficient map points.
        # This prevents locking the offset when SLAM is still initializing.
        if not self.slam_offset_set:
            if self.ekf_pose is None:
                return
            self.stable_count += 1
            if self.stable_count >= STABLE_REQUIRED:
                if self.map_pts >= MAP_PTS_LOCK_MIN:
                    self.slam_offset_x   = self.ekf_pose[0] - sx
                    self.slam_offset_y   = self.ekf_pose[1] - sy
                    self.slam_offset_set = True
                    self.get_logger().info(
                        f'SLAM offset locked: '
                        f'dx={self.slam_offset_x:.2f} '
                        f'dy={self.slam_offset_y:.2f} '
                        f'map_pts={self.map_pts}')
                else:
                    # Map too sparse — reset counter, try again next frame
                    self.stable_count = 0
                    self.get_logger().warn(
                        f'SLAM offset not locked: '
                        f'map_pts={self.map_pts} < {MAP_PTS_LOCK_MIN} — retrying',
                        throttle_duration_sec=2.0)
            return

        # Apply offset to convert SLAM frame → world frame
        wx = sx + self.slam_offset_x
        wy = sy + self.slam_offset_y

        # Slow drift correction — pull offset toward EKF when they agree
        if self.ekf_pose is not None:
            dx   = self.ekf_pose[0] - wx
            dy   = self.ekf_pose[1] - wy
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < CONSISTENCY_SOFT:
                self.slam_offset_x += OFFSET_ALPHA * dx
                self.slam_offset_y += OFFSET_ALPHA * dy
                wx = sx + self.slam_offset_x
                wy = sy + self.slam_offset_y

        self.slam_pose = (wx, wy, sz)

    def _map_cb(self, msg: PointCloud2):
        self.map_pts = msg.width * msg.height

    # ── Weight computation ────────────────────────────────────────
    def _compute_weight(self):
        """
        Returns (w_slam, reason_string).
        All three component scores are in [0, 1].
        Final weight = their product, clamped to [W_SLAM_FLOOR, W_SLAM_CEIL].
        """
        if self.slam_pose is None or not self.slam_offset_set:
            return 0.0, "no_slam_lock"

        now = time.time()

        # 1. Density score — map point count
        if self.map_pts < MAP_PTS_MIN:
            density = 0.0
        elif self.map_pts >= MAP_PTS_GOOD:
            density = 1.0
        else:
            density = (self.map_pts - MAP_PTS_MIN) / (MAP_PTS_GOOD - MAP_PTS_MIN)

        # 2. Age score — penalise stale SLAM
        age = (now - self.slam_t) if self.slam_t else 999.0
        if age > SLAM_MAX_AGE:
            age_score = 0.0
        elif age < SLAM_AGE_HALFLIFE:
            age_score = 1.0
        else:
            age_score = max(0.0,
                1.0 - (age - SLAM_AGE_HALFLIFE) / (SLAM_MAX_AGE - SLAM_AGE_HALFLIFE))

        # 3. Consistency score — SLAM/EKF spatial agreement
        if self.ekf_pose is None:
            consistency = 1.0
        else:
            dist = math.sqrt(
                (self.slam_pose[0] - self.ekf_pose[0])**2 +
                (self.slam_pose[1] - self.ekf_pose[1])**2)
            if dist > CONSISTENCY_HARD:
                return 0.0, f"slam_ekf_gap={dist:.1f}m>hard_limit"
            elif dist < CONSISTENCY_SOFT:
                consistency = 1.0
            else:
                raw = 1.0 - (dist - CONSISTENCY_SOFT) / (CONSISTENCY_HARD - CONSISTENCY_SOFT)
                consistency = math.sqrt(max(0.0, raw))  # softer curve

        w = density * age_score * consistency
        w = max(W_SLAM_FLOOR, min(W_SLAM_CEIL, w))

        reason = (f"pts={self.map_pts} d={density:.2f} "
                  f"age={age:.2f}s a={age_score:.2f} "
                  f"c={consistency:.2f} → w={w:.2f}")
        return w, reason

    # ── Publish ───────────────────────────────────────────────────
    def _publish(self):
        if self.ekf_pose is None:
            return

        w_slam, reason = self._compute_weight()
        w_ekf  = 1.0 - w_slam
        self.w_slam = w_slam

        ex, ey, ez = self.ekf_pose

        if w_slam > 0.0 and self.slam_pose is not None:
            sx, sy, sz = self.slam_pose
            fx = w_ekf * ex + w_slam * sx
            fy = w_ekf * ey + w_slam * sy
            fz = ez                         # always trust depth sensor for Z
            loc_str = f"adaptive(w_slam={w_slam:.2f})"
        else:
            fx, fy, fz = ex, ey, ez
            loc_str = "ekf_only"

        # Build fused Odometry message
        out = Odometry()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = 'world_ned'
        out.child_frame_id  = 'bluerov2'
        out.pose.pose.position.x = fx
        out.pose.pose.position.y = fy
        out.pose.pose.position.z = fz

        if self._ekf_full is not None:
            out.pose.pose.orientation = self._ekf_full.pose.pose.orientation
            out.twist                 = self._ekf_full.twist

        # Covariance: tighter when SLAM is confident
        base_var = 0.05 + (1.0 - w_slam) * 2.0   # 0.05 (slam) → 2.05 (ekf only)
        c = out.pose.covariance
        c[0]  = base_var
        c[7]  = base_var
        c[14] = 0.01    # Z from depth sensor — always tight
        c[21] = 9999.0  # roll
        c[28] = 9999.0  # pitch
        c[35] = 0.05    # yaw from IMU

        self.pub_pose.publish(out)

        conf_msg      = Float32()
        conf_msg.data = float(w_slam)
        self.pub_conf.publish(conf_msg)

        status      = String()
        status.data = f"{loc_str} | {reason}"
        self.pub_status.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
