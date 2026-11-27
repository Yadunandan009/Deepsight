#!/usr/bin/env python3
"""Replay a flight's navigation offline: dead reckoning, the SLAM pose bridge
and the fused filter, with the IMU heading optionally replaced by a gyro of a
stated grade. Used to make G3 (does SLAM help?) fair: in the simulator the
IMU yaw is truth plus 0.1 deg white noise and never drifts (measured on
a50_stkL0_seed7_20261127_010854: -0.001 deg/h), i.e. a perfect compass, which
a vehicle working next to a steel jacket does not have.

Three passes, in the order the live system computes them (nothing feeds
back into the dead-reckoning filter, so this order is exact):

  1. IMU', DVL, depth             -> real ekf_node, ekf_dr.yaml  -> DR'
  2. raw SLAM pose, map points,
     DR' as the reference         -> real SlamPoseBridge class   -> bridge'
  3. IMU', DVL, depth, bridge'    -> real ekf_node, ekf.yaml     -> fused'

Passes 1 and 3 play a bag written for the purpose through robot_localization
on bag time (--clock-topics-all, use_sim_time), on an isolated ROS domain.
Pass 2 drives the bridge class directly as eval/replay_bridge.py does, with
the bridge's 5 s refit timer on bag time; the SLAM messages keep their
recorded header stamps (the bridge copies them onto its output). Bridge
parameters are the PREREG flights' launch defaults: slam_frame enu,
jump_policy quarantine, adaptive gate off.

Not replayed: the controller (the vehicle flies the recorded path, so an
estimate that degrades does not steer the vehicle anywhere new) and ORB-SLAM3
itself (its heading-check decisions are the ones made in flight, with the
perfect compass).

Gyro model (grade from eval/match_gap.py GRADES: bias sigma deg/h, scale
sigma, ARW deg/sqrt(h)): per run, a bias b ~ N(0, sigma_b) and a scale
error s ~ N(0, sigma_s) drawn from the seed, and angle random walk W(t):
    yaw'(t)  = yaw_imu(t) + s * (yaw_imu(t) - yaw_imu(t0)) + b * (t - t0) + W(t)
    wz'(t)   = wz(t) * (1 + s) + b
Roll and pitch are left alone (gravity makes them observable).

Usage (ROS sourced, system python3):
    python3 eval/replay_nav.py BAG --grade none            # fidelity check
    python3 eval/replay_nav.py BAG --grade "MEMS, bias cal. in water (dT 1 C)" --seed 0
Writes  <bags>/replay_nav/<bag>/<grade-tag>_s<seed>/  and a JSON summary.
"""
import argparse
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import time

import numpy as np
import rosbag2_py
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Imu

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import match_gap as mg       # noqa: E402
import replay_bridge as rb   # noqa: E402

CFG = os.path.join(WS, "src", "stonefish_bluerov2")
OUT_ROOT = os.path.join(rb.BAG_ROOT, "replay_nav")
DOMAIN = os.environ.get("REPLAY_DOMAIN", "73")   # one per parallel replay
# playback rate; fidelity checked at 4x (a50_stkL0_seed7: DR' 1.8 mm, fused' 2.3 cm median)
RATE = float(os.environ.get("REPLAY_RATE", "4.0"))
T_IMU, T_DVL, T_DEPTH = "/bluerov2/imu", "/bluerov2/dvl_twist", "/bluerov2/depth_pose"
T_SLAM, T_MAP = "/bluerov2/robot_pose_slam", "/bluerov2/map_points"
T_DR, T_FILT, T_BR, T_TRUTH = ("/bluerov2/odometry/dr", "/bluerov2/odometry/filtered",
                               "/bluerov2/robot_pose_slam_ekf", "/bluerov2/odometry")
TYPES = {T_IMU: "sensor_msgs/msg/Imu", T_DVL: "geometry_msgs/msg/TwistWithCovarianceStamped",
         T_DEPTH: "geometry_msgs/msg/PoseWithCovarianceStamped",
         T_BR: "geometry_msgs/msg/PoseWithCovarianceStamped"}


# ------------------------------------------------------------------ bag io
def reader(path, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    while r.has_next():
        yield r.read_next()       # (topic, serialized bytes, log time ns)


def writer(path, topics):
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    for i, tp in enumerate(topics):
        w.create_topic(rosbag2_py.TopicMetadata(id=i, name=tp, type=TYPES[tp], serialization_format="cdr"))
    return w


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def odom_track(path, topic):
    out = []
    for _, data, _ in reader(path, [topic]):
        m = deserialize_message(data, Odometry)
        p = m.pose.pose.position
        out.append((stamp(m.header), p.x, p.y, p.z))
    return np.array(out)


# ------------------------------------------------------------------ gyro
class Gyro:
    def __init__(self, grade, seed):
        self.on = grade != "none"
        if self.on:
            sb, ss, arw = mg.GRADES[grade]
            rng = np.random.default_rng(seed)
            self.b = math.radians(rng.normal(0.0, sb)) / 3600.0      # rad/s
            self.s = float(rng.normal(0.0, ss))
            self.arw = math.radians(arw) / 60.0                      # rad/sqrt(s)
            self.rng = rng
        self.t0 = self.t_prev = self.y0 = self.y_prev = None
        self.W = 0.0
        self.unwrap = 0.0

    def params(self):
        return dict(bias_dph=math.degrees(self.b) * 3600.0, scale=self.s) if self.on else {}

    def apply(self, m):
        if not self.on:
            return m
        q = m.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        t = stamp(m.header)
        if self.t0 is None:
            self.t0, self.t_prev, self.y0, self.y_prev = t, t, yaw, yaw
        d = (yaw - self.y_prev + math.pi) % (2 * math.pi) - math.pi
        self.unwrap += d
        self.y_prev = yaw
        dt = max(0.0, t - self.t_prev)
        self.t_prev = t
        self.W += self.arw * math.sqrt(dt) * float(self.rng.normal())
        y = yaw + self.s * self.unwrap + self.b * (t - self.t0) + self.W
        cr, sr_, cp, sp = math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(y / 2), math.sin(y / 2)
        q.w = cr * cp * cy + sr_ * sp * sy
        q.x = sr_ * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr_ * cp * sy
        q.z = cr * cp * sy - sr_ * sp * cy
        m.angular_velocity.z = m.angular_velocity.z * (1.0 + self.s) + self.b
        return m


# ------------------------------------------------------------------ passes
def env():
    e = dict(os.environ, ROS_DOMAIN_ID=DOMAIN, ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
    return e


def run_filter(inp, out, cfg, node, remap_to):
    """Play bag `inp` through ekf_node(cfg) on bag time; record `remap_to` into `out`."""
    E = env()
    procs = [
        subprocess.Popen(["ros2", "run", "tf2_ros", "static_transform_publisher", "--frame-id",
                          "bluerov2/base_link", "--child-frame-id", "bluerov2/imu_filter"],
                         env=E, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True),
        subprocess.Popen(["ros2", "run", "robot_localization", "ekf_node", "--ros-args",
                          "-r", f"__node:={node}", "--params-file", cfg, "-p", "use_sim_time:=true",
                          "-r", f"odometry/filtered:={remap_to}"],
                         env=E, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, start_new_session=True),
    ]
    time.sleep(3.0)
    rec = subprocess.Popen(["ros2", "bag", "record", "-s", "mcap", "-o", out, remap_to],
                           env=E, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    time.sleep(3.0)
    subprocess.run(["ros2", "bag", "play", inp, "-r", str(RATE), "--clock-topics-all",
                    "--disable-keyboard-controls"], env=E, check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)
    # the recorder first, and given time: killed before it closes, the bag has
    # no metadata.yaml and cannot be read (what failed the first 6-worker run)
    os.killpg(rec.pid, signal.SIGINT)
    try:
        rec.wait(timeout=60)
    except subprocess.TimeoutExpired:
        os.killpg(rec.pid, signal.SIGKILL)
    for p in procs:
        os.killpg(p.pid, signal.SIGINT)
    for p in procs:
        try:
            p.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
    if not os.path.exists(os.path.join(out, "metadata.yaml")):
        raise RuntimeError(f"recorder did not close {out}")
    err = procs[1].stderr.read().decode(errors="replace") if procs[1].stderr else ""
    return [ln for ln in err.splitlines() if "WARN" in ln or "ERROR" in ln][:5]


def pass_inputs(bag, path, gyro):
    w = writer(path, [T_IMU, T_DVL, T_DEPTH])
    n = 0
    for topic, data, t in reader(bag, [T_IMU, T_DVL, T_DEPTH]):
        if topic == T_IMU and gyro.on:
            data = serialize_message(gyro.apply(deserialize_message(data, Imu)))
        w.write(topic, data, t)
        n += 1
    del w
    return n


def pass_bridge(bag, dr, path_inputs, path_out):
    """Bridge on the recorded SLAM track against DR'; writes inputs + bridge' to path_out."""
    ev = []
    for topic, data, t in reader(bag, [T_SLAM, T_MAP]):
        if topic == T_SLAM:
            ev.append((t / 1e9, 0, deserialize_message(data, PoseStamped)))
        else:
            from sensor_msgs.msg import PointCloud2
            m = deserialize_message(data, PointCloud2)
            ev.append((t / 1e9, 1, m.width * m.height))
    for row in dr:
        ev.append((row[0], 2, tuple(row[1:4])))
    ev.sort(key=lambda e: (e[0], e[1]))

    rb.rclpy.init(args=["--ros-args", "-p", "slam_frame:=enu", "-p", "jump_policy:=quarantine",
                        "--log-level", "slam_pose_bridge:=error"])
    node = rb.spb.SlamPoseBridge()
    out = []
    node.pub = rb.SimpleNamespace(publish=lambda m: out.append((rb.Clock.t, serialize_message(m))))
    next_refit = None
    for t, kind, v in ev:
        rb.Clock.t = t
        if next_refit is None:
            next_refit = t + node.REFIT_INTERVAL
        while t >= next_refit:
            rb.Clock.t = next_refit
            node._periodic_refit()
            next_refit += node.REFIT_INTERVAL
        rb.Clock.t = t
        if kind == 0:
            node.slam_cb(v)
        elif kind == 1:
            node.map_cb(rb.SimpleNamespace(width=v, height=1))
        else:
            m = Odometry()
            m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = v
            node.ekf_cb(m)
    node.destroy_node()
    rb.rclpy.shutdown()

    w = writer(path_out, [T_IMU, T_DVL, T_DEPTH, T_BR])
    merged = [(t, tp, d) for tp, d, t in reader(path_inputs, [T_IMU, T_DVL, T_DEPTH])]
    merged += [(int(t * 1e9), T_BR, d) for t, d in out]
    merged.sort(key=lambda r: r[0])
    for t, tp, d in merged:
        w.write(tp, d, t)
    del w
    br = []
    for t, d in out:
        m = deserialize_message(d, PoseWithCovarianceStamped)
        br.append((stamp(m.header), m.pose.pose.position.x, m.pose.pose.position.y, m.pose.covariance[0]))
    return np.array(br)


# ------------------------------------------------------------------ compare
def recorded(bag):
    rec = {}
    for tp in (T_DR, T_FILT, T_TRUTH):
        rec[tp] = odom_track(bag, tp)
    br = []
    for _, data, _ in reader(bag, [T_BR]):
        m = deserialize_message(data, PoseWithCovarianceStamped)
        br.append((stamp(m.header), m.pose.pose.position.x, m.pose.pose.position.y, m.pose.covariance[0]))
    rec[T_BR] = np.array(br)
    return rec


def diff(a, b):
    """Horizontal distance from track a to track b, at a's stamps inside b's span."""
    if not len(a) or not len(b):
        return None
    ok = (a[:, 0] >= b[0, 0]) & (a[:, 0] <= b[-1, 0])
    d = np.hypot(a[ok, 1] - np.interp(a[ok, 0], b[:, 0], b[:, 1]),
                 a[ok, 2] - np.interp(a[ok, 0], b[:, 0], b[:, 2]))
    return dict(n=int(ok.sum()), median=float(np.median(d)), p95=float(np.percentile(d, 95)),
                max=float(d.max()), end=float(d[-1]))


def tag(grade):
    return "none" if grade == "none" else re.sub(r"[^a-z0-9]+", "-", grade.lower()).strip("-")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--grade", default="none")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--keep-inputs", action="store_true")
    a = ap.parse_args()
    if a.grade != "none" and a.grade not in mg.GRADES:
        sys.exit(f"unknown grade; one of: none, {', '.join(mg.GRADES)}")
    bag = os.path.abspath(a.bag)
    name = os.path.basename(bag.rstrip("/"))
    work = os.path.join(OUT_ROOT, name, f"{tag(a.grade)}_s{a.seed}")
    shutil.rmtree(work, ignore_errors=True)
    os.makedirs(work)
    gyro = Gyro(a.grade, a.seed)
    t_start = time.time()

    p_in = os.path.join(work, "in_dr")
    n = pass_inputs(bag, p_in, gyro)
    w1 = run_filter(p_in, os.path.join(work, "out_dr"), os.path.join(CFG, "ekf_dr.yaml"), "ekf_dr_node", T_DR)
    dr = odom_track(os.path.join(work, "out_dr"), T_DR)
    br = pass_bridge(bag, dr, p_in, os.path.join(work, "in_fused"))
    w3 = run_filter(os.path.join(work, "in_fused"), os.path.join(work, "out_fused"),
                    os.path.join(CFG, "ekf.yaml"), "ekf_filter_node", T_FILT)
    fu = odom_track(os.path.join(work, "out_fused"), T_FILT)

    rec = recorded(bag)
    g = rec[T_TRUTH]
    res = dict(bag=name, grade=a.grade, seed=a.seed, gyro=gyro.params(), input_msgs=n,
               replay_s=round(time.time() - t_start), filter_warnings=dict(dr=w1, fused=w3),
               counts=dict(dr=len(dr), bridge=len(br), fused=len(fu), rec_dr=len(rec[T_DR]),
                           rec_bridge=len(rec[T_BR]), rec_fused=len(rec[T_FILT])),
               vs_recorded=dict(dr=diff(dr, rec[T_DR]), bridge=diff(br, rec[T_BR]), fused=diff(fu, rec[T_FILT])),
               vs_truth=dict(dr=diff(dr, g), fused=diff(fu, g), bridge=diff(br, g),
                             rec_dr=diff(rec[T_DR], g), rec_fused=diff(rec[T_FILT], g)))
    json.dump(res, open(os.path.join(work, "summary.json"), "w"), indent=1)
    if not a.keep_inputs:     # regenerable from the flight bag; ~200 MB per replay
        for d in ("in_dr", "in_fused"):
            shutil.rmtree(os.path.join(work, d), ignore_errors=True)
    print(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
