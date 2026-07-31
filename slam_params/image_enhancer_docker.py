#!/usr/bin/env python3
"""Image enhancer that runs INSIDE Docker to avoid DDS type hash issues."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
import numpy as np

class ImageEnhancerDocker(Node):
    def __init__(self):
        super().__init__('image_enhancer_docker',
            parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time',
                    rclpy.parameter.Parameter.Type.BOOL, True)])

        # Use SensorDataQoS to match Stonefish
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            Image, '/bluerov2/down/image_color', self.cb, qos)
        self.pub = self.create_publisher(Image, '/bluerov2/down/enhanced', qos)
        self.get_logger().info('Docker image enhancer started - subscribing to /bluerov2/down/image_color')

    def cb(self, msg):
        data = np.frombuffer(msg.data, dtype=np.uint8).copy()
        mn, mx = data.min(), data.max()
        if mx > mn:
            data = ((data - mn) / (mx - mn) * 255).astype(np.uint8)
        out = Image()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.encoding = msg.encoding
        out.step = msg.step
        out.data = data.tobytes()
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ImageEnhancerDocker())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
