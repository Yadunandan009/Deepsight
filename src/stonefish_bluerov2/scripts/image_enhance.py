#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
import numpy as np

class ImageEnhancer(Node):
    def __init__(self):
        super().__init__('image_enhancer',
            parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time',
                    rclpy.parameter.Parameter.Type.BOOL, True)])
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.sub = self.create_subscription(
            Image, '/bluerov2/down/image_color', self.cb, qos)
        self.pub = self.create_publisher(Image, '/bluerov2/down/enhanced', 10)
        self.get_logger().info('Image enhancer started with sim_time')

    def cb(self, msg):
        data = np.frombuffer(msg.data, dtype=np.uint8).copy()
        mn, mx = data.min(), data.max()
        if mx > mn:
            data = ((data - mn) / (mx - mn) * 255).astype(np.uint8)
        out = Image()
        out.header = msg.header  # preserve original timestamp
        out.height = msg.height
        out.width = msg.width
        out.encoding = msg.encoding
        out.step = msg.step
        out.data = data.tobytes()
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ImageEnhancer())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
