#!/usr/bin/env python3
"""
Extract controller-reliability metrics from one trial's bag.

Scoped to ground-truth odometry + thruster setpoint only (no live SLAM in
these trials -- see run_trial.py's docstring for why). Metrics:
  - orbit tracking error: deviation from SCAN_DIST during scan-range dwells
    (a direct measure of controller tracking accuracy against ground
    truth, standing in for "position error vs ground truth" here since
    SLAM isn't part of this ablation's causal chain -- the allocation
    matrix bug doesn't touch SLAM at all)
  - max sway speed (CLOSE_IN crabbing disturbance magnitude)
  - max yaw rate in the first 15s (DESCEND-phase disturbance magnitude)
  - thruster saturation fraction (any channel at |cmd| >= 0.99)
"""
import glob
import math
import os
import sys

import numpy as np

try:
    from mcap_ros2.reader import read_ros2_messages
except ImportError:
    print("pip3 install mcap-ros2-support --break-system-packages")
    sys.exit(1)

TURBINE_X, TURBINE_Y = 20.0, 0.0
SCAN_DIST = 17.0


def ts(msg):
    return msg.log_time.timestamp() if hasattr(msg.log_time, "timestamp") \
        else msg.log_time / 1e9


def extract_metrics(bag_path):
    mcap_files = sorted(glob.glob(os.path.join(bag_path, "*.mcap")))
    if not mcap_files:
        return {"error": f"no mcap files in {bag_path}"}

    t_list, x_list, y_list = [], [], []
    vy_list, yawrate_list = [], []
    pwm_t, pwm_sat = [], []

    for f in mcap_files:
        for msg in read_ros2_messages(f, topics=["/bluerov2/odometry", "/bluerov2/setpoint/pwm"]):
            t = ts(msg)
            m = msg.ros_msg
            if msg.channel.topic == "/bluerov2/odometry":
                p = m.pose.pose.position
                tw = m.twist.twist
                t_list.append(t)
                x_list.append(p.x)
                y_list.append(p.y)
                vy_list.append(tw.linear.y)
                yawrate_list.append(tw.angular.z)
            elif msg.channel.topic == "/bluerov2/setpoint/pwm":
                pwm_t.append(t)
                pwm_sat.append(max(abs(v) for v in m.data) >= 0.99 if m.data else False)

    if not t_list:
        return {"error": "no odometry in bag"}

    t0 = t_list[0]
    t_arr = np.array(t_list) - t0
    x_arr, y_arr = np.array(x_list), np.array(y_list)
    vy_arr, yawrate_arr = np.array(vy_list), np.array(yawrate_list)

    radius = np.hypot(x_arr - TURBINE_X, y_arr - TURBINE_Y)
    scan_mask = np.abs(radius - SCAN_DIST) < 2.5
    orbit_err = np.abs(radius[scan_mask] - SCAN_DIST) if scan_mask.any() else np.array([])

    early_mask = t_arr < 15.0

    result = {
        "duration_recorded_s": float(t_arr[-1]),
        "n_odom_samples": len(t_list),
        "orbit_tracking_err_mean_m": float(orbit_err.mean()) if len(orbit_err) else None,
        "orbit_tracking_err_rms_m": float(np.sqrt(np.mean(orbit_err**2))) if len(orbit_err) else None,
        "orbit_tracking_err_max_m": float(orbit_err.max()) if len(orbit_err) else None,
        "max_abs_sway_mps": float(np.abs(vy_arr).max()) if len(vy_arr) else None,
        "max_abs_yawrate_first15s_degs": float(np.degrees(np.abs(yawrate_arr[early_mask]).max())) if early_mask.any() else None,
        "max_abs_yawrate_overall_degs": float(np.degrees(np.abs(yawrate_arr).max())) if len(yawrate_arr) else None,
        "thruster_saturation_frac": float(np.mean(pwm_sat)) if pwm_sat else None,
    }
    return result


if __name__ == "__main__":
    import json
    bag_path = sys.argv[1]
    print(json.dumps(extract_metrics(bag_path), indent=2))
