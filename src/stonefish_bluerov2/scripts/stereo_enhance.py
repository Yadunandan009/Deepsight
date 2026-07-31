#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import time
import message_filters
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# CLAHE Configuration
CLIP_LIMIT = 4.0
TILE_SIZE = (8, 8)
TARGET_FPS = 15.0

class StereoEnhancer(Node):
    def __init__(self):
        super().__init__('stereo_enhancer')
        
        # QoS setup for Best Effort compatibility (standard for high-bandwidth images)
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Publishers
        self.pub_l = self.create_publisher(Image, '/bluerov2/left/enhanced', 10)
        self.pub_r = self.create_publisher(Image, '/bluerov2/right/enhanced', 10)

        # Subscribers with synchronization and Best Effort QoS
        self.sub_l = message_filters.Subscriber(self, Image, '/bluerov2/left/image_raw', qos_profile=image_qos)
        self.sub_r = message_filters.Subscriber(self, Image, '/bluerov2/right/image_raw', qos_profile=image_qos)
        
        # Increase slop to 0.1s to be more permissive with simulator delivery
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_l, self.sub_r], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_cb)

        self.last_pub_t = 0.0
        self.target_dt = 1.0 / TARGET_FPS
        self.processed_count = 0
        
        self.clahe = cv2.createCLAHE(clipLimit=CLIP_LIMIT, tileGridSize=TILE_SIZE)
        self.get_logger().info(
            f'Synchronized Stereo CLAHE enhancer started ({TARGET_FPS} FPS, BEST_EFFORT, slop=0.1)')

    def _enhance(self, msg):
        """Apply CLAHE to each channel independently."""
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, -1).copy()

        # Apply CLAHE per channel
        for c in range(img.shape[2]):
            img[:, :, c] = self.clahe.apply(img[:, :, c])

        out = Image()
        out.header = msg.header
        out.height = msg.height
        out.width  = msg.width
        out.encoding = msg.encoding
        out.step = msg.step
        out.data = img.tobytes()
        return out

    def sync_cb(self, left_msg, right_msg):
        """Callback for synchronized image pairs."""
        t_now = self.get_clock().now().nanoseconds / 1e9
        
        # Rate limiting logic
        if t_now - self.last_pub_t >= self.target_dt:
            # Process and publish both
            self.pub_l.publish(self._enhance(left_msg))
            self.pub_r.publish(self._enhance(right_msg))
            self.last_pub_t = t_now
            
            self.processed_count += 1
            if self.processed_count % 100 == 0:
                self.get_logger().info(f'Processed {self.processed_count} stereo pairs')

def main():
    rclpy.init()
    node = StereoEnhancer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
