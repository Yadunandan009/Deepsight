#!/usr/bin/env python3
"""
Converts stonefish_ros2/DVL → geometry_msgs/TwistWithCovarianceStamped
so robot_localization EKF can fuse DVL velocity.

MOUNT ROTATION. The DVL is mounted rolled 180 deg about X:

    <sensor name="dvl" ...>
        <origin rpy="3.1416 0.0 0.0" xyz="0.0 0.0 0.0"/>

so it reports velocity in a frame where Y and Z point opposite to
base_link. This node previously copied m.velocity straight through and
stamped it frame_id='bluerov2/base_link', handing the EKF a Y (and Z)
velocity of the wrong sign.

HOW IT HID. The speed is unchanged by the flip, so every magnitude check
passes -- measured live 2026-10-09 against ground truth: |v| 0.236 m/s
both, while the components were truth (+0.044, +0.228, 0.000) against DVL
(+0.045, -0.229, -0.000). And until odom0 was removed from ekf.yaml on
2026-10-08 the filter was anchored on ground-truth position at 100 Hz,
which corrected the bad velocity away every cycle. It only became
load-bearing -- and only became visible -- once the anchor was cut, where
it drove position error to 15.9 m in 244 s and bearing to 40 deg while
IMU yaw stayed under 0.2 deg.

The rotation is a parameter rather than a hardcoded negation so that
changing the scenario mount does not silently reintroduce this.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TwistWithCovarianceStamped
from stonefish_ros2.msg import DVL

class DVLBridge(Node):
    def __init__(self):
        super().__init__('dvl_bridge')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(
            TwistWithCovarianceStamped,
            '/bluerov2/dvl_twist', 10)
        # Mount orientation of the DVL relative to base_link, radians,
        # matching <origin rpy="..."> in the scenario.
        self.declare_parameter('mount_rpy', [math.pi, 0.0, 0.0])
        r, p_, y = self.get_parameter('mount_rpy').value
        self.R = self._rpy_to_matrix(r, p_, y)
        self.create_subscription(DVL, '/bluerov2/dvl_sim', self.cb, qos)
        self.get_logger().info(
            f'DVL bridge started; mount_rpy=({r:.4f}, {p_:.4f}, {y:.4f}) rad, '
            f'sign map on (x,y,z) = '
            f'({self.R[0][0]:+.0f}, {self.R[1][1]:+.0f}, {self.R[2][2]:+.0f})')

    @staticmethod
    def _rpy_to_matrix(r, p, y):
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(y), math.sin(y)
        return [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp,     cp * sr,                cp * cr],
        ]

    def cb(self, m):
        msg = TwistWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'bluerov2/base_link'
        # Rotate from the DVL's own frame into base_link before publishing,
        # since the frame_id below claims base_link.
        v = (m.velocity.x, m.velocity.y, m.velocity.z)
        vx = self.R[0][0] * v[0] + self.R[0][1] * v[1] + self.R[0][2] * v[2]
        vy = self.R[1][0] * v[0] + self.R[1][1] * v[1] + self.R[1][2] * v[2]
        vz = self.R[2][0] * v[0] + self.R[2][1] * v[1] + self.R[2][2] * v[2]
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.linear.z = vz
        # DVL velocity covariance — tune if needed
        msg.twist.covariance[0]  = 0.01   # vx variance
        msg.twist.covariance[7]  = 0.01   # vy variance
        msg.twist.covariance[14] = 0.01   # vz variance
        msg.twist.covariance[21] = 9999.0 # ignore angular
        msg.twist.covariance[28] = 9999.0
        msg.twist.covariance[35] = 9999.0
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DVLBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
