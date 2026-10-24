#!/usr/bin/env python3
"""Synthetic test of slam_pose_bridge.py with the adaptive gate, no simulator, no SLAM.

Creates the bridge node in-process, replaces its publisher with a recorder and feeds its
callbacks fake messages (no spinning). Needs ROS sourced:
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  python3 eval/test_bridge_gate.py

Scenario: a vehicle orbits a circle; the reference (dead reckoning) is the truth plus a
constant 0.3 m offset; SLAM is the truth in its own rotated/translated frame. At step 600
SLAM's frame jumps (rotated 90 degrees) and, from then on, drifts by 0.04 m per step, which
stands for a re-lock whose fit extrapolates badly. Checks:
  1. gate ON still publishes before the jump;
  2. after the jump the gate ON keeps every published pose within 1.5 m of the truth, while
     the gate OFF (legacy 5 m) lets poses through that are more than 2 m off, so the
     scenario does discriminate;
  3. the gate ON withholds poses just after the re-lock (probation).
"""
import importlib.util
import math
import os
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_PARENT = os.path.abspath(os.path.join(HERE, "..", "src", "stonefish_bluerov2"))
sys.path.insert(0, PKG_PARENT)   # the source package, so the test needs no rebuild
SRC = os.path.join(PKG_PARENT, "scripts", "slam_pose_bridge.py")


def load(gate_on):
    spec = importlib.util.spec_from_file_location("bridge_under_test", SRC)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["bridge_under_test"] = mod
    spec.loader.exec_module(mod)
    args = ["--ros-args", "-p", f"adaptive_gate:={'true' if gate_on else 'false'}",
            "-p", "gate_probation_poses:=10", "-p", "gate_probation_spread:=6.0",
            "-p", "slam_frame:=ned"]
    rclpy.init(args=args)
    node = mod.SlamPoseBridge()
    sent = []
    node.pub = type("P", (), {"publish": lambda self, m: sent.append(
        (m.pose.pose.position.x, m.pose.pose.position.y))})()
    return node, sent


def feed(node, t, truth, ref_off, slam_rot_deg, slam_drift=0.0):
    """One 13 Hz step: reference odometry, then a SLAM pose."""
    od = Odometry()
    od.pose.pose.position.x, od.pose.pose.position.y = truth[0] + ref_off, truth[1]
    node.ekf_cb(od)
    c, s = math.cos(math.radians(slam_rot_deg)), math.sin(math.radians(slam_rot_deg))
    sx, sy = c * truth[0] - s * truth[1] + 100.0 + slam_drift, s * truth[0] + c * truth[1] - 50.0   # SLAM's own frame
    ps = PoseStamped()
    ps.pose.position.x, ps.pose.position.y = sx, sy
    ps.pose.orientation.w = 1.0
    node.slam_cb(ps)


def run(gate_on):
    node, sent = load(gate_on)
    pc = PointCloud2()
    pc.width, pc.height = 50000, 1
    node.map_cb(pc)
    node.map_stable_since = 0.0           # the map has been stable "for a long time"
    R, w = 17.0, 0.02                      # radius (m), rad per step
    marks = {}
    n = 0
    for k in range(900):
        ang = w * k
        truth = (20.0 + R * math.cos(ang), R * math.sin(ang))
        rot = 0.0 if k < 600 else 90.0     # the wrong-face jump at step 600
        drift = 0.0 if k < 600 else 0.04 * (k - 600)
        before = len(sent)
        feed(node, k, truth, 0.3, rot, drift)
        if len(sent) > before:
            err = math.hypot(sent[-1][0] - truth[0], sent[-1][1] - truth[1])
            if k >= 600:
                marks["max_err_after_jump"] = max(marks.get("max_err_after_jump", 0.0), err)
        if k == 599:
            marks["published_before_jump"] = len(sent)
        if k == 650:
            marks["published_50_steps_after_jump"] = len(sent) - marks["published_before_jump"]
        if k == 899:
            marks["published_total"] = len(sent)
    marks["aligned_at_end"] = node.aligned
    node.destroy_node()
    rclpy.shutdown()
    return marks, sent


if __name__ == "__main__":
    on, sent_on = run(True)
    off, sent_off = run(False)
    print("gate ON :", on)
    print("gate OFF:", off)
    assert on["published_before_jump"] > 0, "gate ON never published before the jump"
    assert on.get("max_err_after_jump", 0.0) <= 1.5, "gate ON let a pose more than 1.5 m off through"
    assert off.get("max_err_after_jump", 0.0) > 2.0, "scenario does not discriminate: gate OFF stayed within 2 m"
    assert on["published_50_steps_after_jump"] < off["published_50_steps_after_jump"], "no probation effect after the re-lock"
    print("bridge gate test passed")
