#!/usr/bin/env python3
"""
Converts stonefish_ros2/DVL → geometry_msgs/TwistWithCovarianceStamped
so robot_localization EKF can fuse DVL velocity.
"""
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
        self.create_subscription(DVL, '/bluerov2/dvl_sim', self.cb, qos)
        self.get_logger().info('DVL bridge started')

    def cb(self, m):
        msg = TwistWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'bluerov2/base_link'
        msg.twist.twist.linear.x = m.velocity.x
        msg.twist.twist.linear.y = m.velocity.y
        msg.twist.twist.linear.z = m.velocity.z
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
