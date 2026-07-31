#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class SLAMInitNode(Node):
    def __init__(self):
        super().__init__('slam_init_move')
        self.pub = self.create_publisher(Float64MultiArray, '/bluerov2/setpoint/pwm', 10)
        self.get_logger().info("SLAM Init Movement Started")

    def publish(self, fr, fl, br, bl, dfr, dfl, dbr, dbl):
        msg = Float64MultiArray(data=[fr, fl, br, bl, dfr, dfl, dbr, dbl])
        self.pub.publish(msg)

    def stop(self, duration=1.0):
        self.publish(0,0,0,0,0,0,0,0)
        time.sleep(duration)

    def surge(self, speed, duration):
        self.publish(speed, speed, -speed, -speed, 0,0,0,0)
        time.sleep(duration)

    def heave(self, speed, duration):
        self.publish(0,0,0,0, speed, speed, speed, speed)
        time.sleep(duration)

    def yaw(self, speed, duration):
        self.publish(speed, -speed, speed, -speed, 0,0,0,0)
        time.sleep(duration)

    def run(self):
        # Turbine is at X=20, Z=25. Robot starts at X=15, Z=15.
        # Need to go +5m X and +10m Z (dive down)
        
        self.get_logger().info("Diving down 10m toward turbine depth...")
        self.heave(-1.0, 6.0)
        self.stop(0.5)

        self.get_logger().info("Surging 5m toward turbine...")
        self.surge(1.0, 4.0)
        self.stop(0.5)

        self.get_logger().info("Slow survey around turbine for SLAM mapping...")
        for _ in range(8):
            self.surge(0.4, 2.0)
            self.yaw(0.3, 1.5)
            self.surge(0.4, 2.0)
            self.yaw(-0.3, 1.5)
            self.heave(-0.3, 1.0)
            self.heave(0.3, 1.0)

        self.stop(1.0)
        self.get_logger().info("Survey complete!")

def main(args=None):
    rclpy.init(args=args)
    node = SLAMInitNode()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
