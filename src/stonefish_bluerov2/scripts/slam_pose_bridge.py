#!/usr/bin/env python3
"""
Converts SLAM PoseStamped → PoseWithCovarianceStamped in world_ned frame.
Aligns SLAM coordinate frame to EKF world frame using offset computed
at first stable SLAM lock-on.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import math


class SlamPoseBridge(Node):
    def __init__(self):
        super().__init__('slam_pose_bridge')
        # EKF output — world frame anchor
        self.ekf_x = None
        self.ekf_y = None
        self.ekf_z = None
        # Offset from SLAM frame to world_ned frame
        self.offset_x = None
        self.offset_y = None
        self.offset_set = False
        # Stability check — require N consecutive poses within threshold
        self.stable_count = 0
        self.STABLE_REQUIRED = 20  # 20 consecutive poses ~1.5s at 13Hz
        self.last_slam_x = None
        self.last_slam_y = None
        self.JUMP_THRESHOLD = 5.0  # m — reject jumps > 2m between frames

        # Map density gate — minimum map points before SLAM is trusted at all
        self.map_pts = 0
        self.MIN_MAP_PTS = 500

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/bluerov2/robot_pose_slam_ekf', 10)
        self.create_subscription(
            PoseStamped,
            '/bluerov2/robot_pose_slam',
            self.slam_cb, 10)
        self.create_subscription(
            Odometry,
            '/bluerov2/odometry/filtered',
            self.ekf_cb, 10)
        self.create_subscription(
            PointCloud2,
            '/bluerov2/map_points',
            self.map_cb, 10)
        self.get_logger().info('SLAM pose bridge started')

    def ekf_cb(self, m):
        self.ekf_x = m.pose.pose.position.x
        self.ekf_y = m.pose.pose.position.y
        self.ekf_z = m.pose.pose.position.z

    def map_cb(self, m):
        self.map_pts = m.width * m.height

    def slam_cb(self, m):
        sx = m.pose.position.x
        sy = m.pose.position.y

        # Detect jumps (relocalization artifacts)
        if self.last_slam_x is not None:
            jump = math.sqrt((sx - self.last_slam_x)**2 +
                           (sy - self.last_slam_y)**2)
            if jump > self.JUMP_THRESHOLD:
                self.get_logger().warn(
                    f'SLAM jump detected {jump:.2f}m — resetting offset')
                self.offset_set = False
                self.stable_count = 0
        self.last_slam_x = sx
        self.last_slam_y = sy

        # Build stability count
        if not self.offset_set:
            if self.ekf_x is None:
                return

            # GATE 1 — lock gate: don't even count this frame toward
            # stability if the map is too sparse to trust. Prevents the
            # offset from being anchored against a barely-initialized map
            # (e.g. only ~14k points during early descent), which was the
            # root cause of the yaw-flip observed at z≈1m.
            if self.map_pts < self.MIN_MAP_PTS:
                self.stable_count = 0
                self.get_logger().warn(
                    f'SLAM offset lock blocked: map_pts={self.map_pts} '
                    f'< {self.MIN_MAP_PTS} — waiting for denser map',
                    throttle_duration_sec=2.0)
                return

            self.stable_count += 1
            if self.stable_count >= self.STABLE_REQUIRED:
                # Compute offset: world = slam + offset
                self.offset_x = self.ekf_x - sx
                self.offset_y = self.ekf_y - sy
                self.offset_set = True
                self.get_logger().info(
                    f'SLAM offset computed: dx={self.offset_x:.2f} '
                    f'dy={self.offset_y:.2f} after {self.stable_count} frames '
                    f'map_pts={self.map_pts}')
            return

        # GATE 2 — publish gate: even with the offset already locked,
        # skip publishing this particular pose if the map has momentarily
        # gone sparse (motion blur, brief occlusion, texture loss). The
        # EKF simply keeps its prior estimate for this tick rather than
        # being fed a noisy correction.
        if self.map_pts < self.MIN_MAP_PTS:
            self.get_logger().warn(
                f'SLAM pose suppressed: map_pts={self.map_pts} '
                f'< {self.MIN_MAP_PTS}', throttle_duration_sec=2.0)
            return

        # Apply offset and publish
        out = PoseWithCovarianceStamped()
        out.header = m.header
        out.header.frame_id = 'world_ned'
        out.pose.pose = m.pose
        out.pose.pose.position.x = sx + self.offset_x
        out.pose.pose.position.y = sy + self.offset_y
        # SLAM covariance — tune based on tracking quality
        c = out.pose.covariance
        c[0]  = 0.05   # x variance
        c[7]  = 0.05   # y variance
        c[14] = 9999.0 # z — not trusted
        c[21] = 9999.0 # roll
        c[28] = 9999.0 # pitch
        c[35] = 0.05   # yaw variance
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SlamPoseBridge())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
