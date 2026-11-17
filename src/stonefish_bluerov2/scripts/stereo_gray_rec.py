#!/usr/bin/env python3
"""Republish both enhanced stereo images as the grayscale ORB-SLAM3 computes.

For record_run.sh STEREO=1. The bags have only ever kept the left eye at
1 Hz, so no flight could be put through a second SLAM configuration: every
comparison of stock vs heading-checked vs stereo-inertial ORB-SLAM3 needed
its own flight, on its own trajectory. With both eyes at full rate, and
/bluerov2/imu (already recorded at 200 Hz), any flight can be replayed into
any configuration on identical input.

Grayscale, not colour, because it is a third of the size (about 7 GB per
mission instead of 21) and loses nothing ORB-SLAM3 sees: with
Camera.RGB: 1 it converts each 3-channel frame with cv::COLOR_RGB2GRAY
before extracting a single feature (Tracking::GrabImageStereo), and a
1-channel frame is used as is. The enhanced images arrive as bgr8, so that
conversion weights channel 0 (blue) as red. It is reproduced here bit for
bit, in OpenCV's own fixed-point form, rather than corrected: the point is
that a replay gives ORB-SLAM3 exactly what the flight gave it.

numpy only, no cv_bridge or cv2: those are the compiled extensions that die
under the venv's NumPy 2 (see IMAGE_NODE_PY in bluerov2_sim.py).

  /bluerov2/{left,right}/image_enhanced  ->  /bluerov2/{left,right}/image_gray_rec
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

# cv::COLOR_RGB2GRAY on 8-bit input: (R*9798 + G*19235 + B*3735 + 2^14) >> 15.
# The often-quoted 14-bit constants (4899/9617/1868) are older OpenCV and
# differ on ~0.1% of pixels. These match, with zero differing pixels, both
# the OpenCV 4.4 that libORB_SLAM3 links in the SLAM image and the host's 4.13.
W_R, W_G, W_B, SHIFT = 9798, 19235, 3735, 15


def rgb2gray_cv(a):
    """OpenCV's RGB2GRAY, channel 0 taken as R, exactly."""
    a = a.astype(np.uint32)
    return ((a[..., 0] * W_R + a[..., 1] * W_G + a[..., 2] * W_B
             + (1 << (SHIFT - 1))) >> SHIFT).astype(np.uint8)


class StereoGrayRec(Node):
    def __init__(self):
        super().__init__('stereo_gray_rec')
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.n = {}
        for eye in ('left', 'right'):
            pub = self.create_publisher(Image, f'/bluerov2/{eye}/image_gray_rec', qos)
            self.create_subscription(
                Image, f'/bluerov2/{eye}/image_enhanced',
                lambda m, p=pub, e=eye: self.cb(m, p, e), qos)
            self.n[eye] = 0
        self.create_timer(30.0, lambda: self.get_logger().info(
            f'gray frames: left {self.n["left"]}, right {self.n["right"]}'))

    def cb(self, m, pub, eye):
        if m.encoding not in ('bgr8', 'rgb8'):
            self.get_logger().error(f'{eye}: unexpected encoding {m.encoding}',
                                    throttle_duration_sec=10.0)
            return
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step // 3 if m.step else m.width, 3)
        out = Image()
        out.header = m.header
        out.height, out.width = m.height, a.shape[1]
        out.encoding = 'mono8'
        out.step = out.width
        out.data = rgb2gray_cv(a).tobytes()
        pub.publish(out)
        self.n[eye] += 1


def main():
    rclpy.init()
    rclpy.spin(StereoGrayRec())


if __name__ == '__main__':
    main()
