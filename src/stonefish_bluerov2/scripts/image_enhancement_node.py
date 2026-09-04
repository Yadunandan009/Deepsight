#!/usr/bin/env python3
"""
Image enhancement bridge — CLAHE + gamma + unsharp-mask sharpening for the
main inspection camera feed, to counter the contrast/color-cast loss and
scattering blur that come with degraded-visibility water (high Jerlov
turbidity). Subscribes to the raw camera feed, republishes an enhanced
version on a parallel topic -- the raw feed is left untouched so anything
already consuming it (SLAM, splat extraction) is unaffected.

Pipeline, in order:
  1. Gamma correction on the raw image -- underwater attenuation biases
     exposure dark; a gamma < 1 brightens midtones without blowing out
     highlights the way a flat brightness offset would.
  2. CLAHE (Contrast Limited Adaptive Histogram Equalization) on the L
     channel in LAB space, not each RGB channel independently -- doing
     per-channel histogram equalization on raw RGB shifts color balance
     (introduces false color casts); operating on lightness alone
     preserves hue/saturation while still fixing the low local contrast
     that scattering causes.
  3. Unsharp-mask sharpening to partially recover edge definition lost to
     forward-scattering blur.

Standard, well-established underwater image restoration recipe -- not
attempting a physical model of the water column (a la Sea-Thru/UDCP dehazing),
just correcting for the specific degradations added by this project's own
turbidity model.
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageEnhancementNode(Node):
    def __init__(self):
        super().__init__('image_enhancement_node')

        self.declare_parameter('input_topic', '/bluerov2/left/image_color')
        self.declare_parameter('output_topic', '/bluerov2/left/image_enhanced')
        self.declare_parameter('gamma', 0.7)          # <1 brightens midtones
        self.declare_parameter('clahe_clip_limit', 3.0)
        self.declare_parameter('clahe_tile_size', 8)
        self.declare_parameter('sharpen_amount', 0.6)  # 0 = off

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.gamma = float(self.get_parameter('gamma').value)
        clip = float(self.get_parameter('clahe_clip_limit').value)
        tile = int(self.get_parameter('clahe_tile_size').value)
        self.sharpen_amount = float(self.get_parameter('sharpen_amount').value)

        self.clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(tile, tile))
        self._gamma_lut = np.array(
            [((i / 255.0) ** self.gamma) * 255 for i in range(256)]
        ).astype(np.uint8)

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, input_topic, self.cb, 10)
        self.pub = self.create_publisher(Image, output_topic, 10)
        self.get_logger().info(
            f'Image enhancement node started: {input_topic} -> {output_topic} '
            f'(gamma={self.gamma}, clahe_clip={clip}, tile={tile}, sharpen={self.sharpen_amount})'
        )

    def cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge conversion failed: {e}', throttle_duration_sec=5.0)
            return

        enhanced = self.enhance(img)

        out_msg = self.bridge.cv2_to_imgmsg(enhanced, encoding='bgr8')
        out_msg.header = msg.header
        self.pub.publish(out_msg)

    def enhance(self, img):
        # 1. Gamma correction
        img = cv2.LUT(img, self._gamma_lut)

        # 2. CLAHE on L channel only (LAB space)
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        l = self.clahe.apply(l)
        lab = cv2.merge((l, a, b))
        img = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

        # 3. Unsharp-mask sharpening
        if self.sharpen_amount > 0:
            blurred = cv2.GaussianBlur(img, (0, 0), sigmaX=3)
            img = cv2.addWeighted(img, 1 + self.sharpen_amount, blurred, -self.sharpen_amount, 0)

        return img


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ImageEnhancementNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
