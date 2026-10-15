#!/usr/bin/env python3
"""Read the stereo baseline the SIMULATOR is actually running.

check_stereo_baseline.py compares files on disk. It cannot tell you what
Stonefish loaded -- a stale build, an install/ copy that did not get
refreshed, or simply launching the wrong scenario all produce a running
system whose geometry differs from the files in the tree.

The right camera's CameraInfo carries it directly: P[3] = -fx * b, which
Stonefish fills from the scenario's baseline="..." attribute. P[0] is fx.

Prints the baseline in metres on stdout and exits 0.
Exit 2 if no CameraInfo arrived; exit 1 if --expect is given and missed.
"""
import argparse
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/bluerov2/right/camera_info")
    ap.add_argument("--timeout", type=float, default=10.0)
    ap.add_argument("--expect", type=float, default=None,
                    help="baseline in m that the config claims")
    ap.add_argument("--tol", type=float, default=2e-3)
    a = ap.parse_args()

    rclpy.init(args=None)
    node = Node("live_stereo_baseline")
    got = {}

    def cb(msg):
        got["fx"] = msg.p[0]
        got["p3"] = msg.p[3]

    # CameraInfo is latched by nobody here; it comes at camera rate, so a
    # plain volatile subscription with a short spin is enough.
    node.create_subscription(CameraInfo, a.topic, cb, 10)
    end = node.get_clock().now().nanoseconds + int(a.timeout * 1e9)
    while rclpy.ok() and "p3" not in got:
        if node.get_clock().now().nanoseconds > end:
            break
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    if "p3" not in got:
        print(f"no CameraInfo on {a.topic} within {a.timeout:.0f}s",
              file=sys.stderr)
        return 2

    fx, p3 = got["fx"], got["p3"]
    if fx == 0.0:
        print(f"{a.topic} reports fx=0; cannot derive a baseline",
              file=sys.stderr)
        return 2
    b = -p3 / fx
    print(f"{b:.4f}")
    if a.expect is not None and abs(b - a.expect) > a.tol:
        print(f"live baseline {b:.4f} m != expected {a.expect:.4f} m "
              f"(fx={fx:.2f}, P[3]={p3:.2f})", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
