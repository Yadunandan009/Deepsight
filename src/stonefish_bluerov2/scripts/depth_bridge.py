#!/usr/bin/env python3
"""
Depth bridge — converts Stonefish pressure to Z pose for EKF.
Stonefish pressure sensor gives gauge pressure relative to surface.
At spawn (world Z=15m), pressure reads ~434 Pa.
Scale: depth_m = fluid_pressure / 43.4 gives ~10m at spawn.
We publish in world_ned frame where Z is positive downward (depth).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import PoseWithCovarianceStamped

class DepthBridge(Node):
    def __init__(self):
        super().__init__('depth_bridge')
        self.subscription = self.create_subscription(
            FluidPressure, '/bluerov2/pressure', self.cb, 10)
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/bluerov2/depth_pose', 10)
        self.get_logger().info('Depth Bridge started')

    def cb(self, msg):
        # fluid_pressure in Stonefish is gauge pressure
        # empirically: 434 Pa at ~10m odometry depth
        # raw odometry Z at spawn = 0, pressure gives absolute depth offset
        # Use pressure directly scaled to match raw odometry Z convention
        depth_m = msg.fluid_pressure / 43.4

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = 'world_ned'
        pose_msg.pose.pose.position.z = depth_m
        covariance = [0.0] * 36
        covariance[14] = 0.5   # moderate trust — less than raw odometry
        pose_msg.pose.covariance = covariance
        self.publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DepthBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
