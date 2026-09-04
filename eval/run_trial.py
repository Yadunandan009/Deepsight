#!/usr/bin/env python3
"""
Run one autonomous mission trial end-to-end: launch Stonefish + bridges +
controller, record a scoped bag (just what's needed for the evaluation
metrics, not the full camera/sonar/SLAM-map bag used elsewhere in this
project), detect MISSION COMPLETE / ESTOP / timeout by watching the
controller's own log output, then tear everything down cleanly before
returning -- so trials can be run back-to-back unattended.

Usage: python3 run_trial.py <trial_name> [--timeout SECONDS]
Prints one JSON line with the trial result to stdout on completion.
"""
import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time

ROS_SETUP = (
    # This shell's default profile activates venv-ardupilot, which puts its
    # own python3 (numpy 2.x) ahead of the system one on PATH. ros2 launch's
    # python-shebang child nodes inherit that PATH and silently run under
    # the wrong interpreter -- most nodes tolerate it, but cv_bridge's
    # compiled extension (built against numpy<2) hard-crashes. Strip the
    # venv from PATH before sourcing ROS so every launched node gets the
    # correct system python.
    "export PATH=$(echo \"$PATH\" | tr ':' '\\n' | grep -v venv-ardupilot | tr '\\n' ':') && "
    "unset VIRTUAL_ENV && "
    "source /opt/ros/jazzy/setup.bash && source /home/yadunandan/ros2_ws/install/setup.bash"
)
BAG_TOPICS = "/bluerov2/odometry /bluerov2/robot_pose_slam_ekf /bluerov2/setpoint/pwm"


def start(cmd):
    return subprocess.Popen(
        f"{ROS_SETUP} && exec {cmd}",
        shell=True, executable="/bin/bash",
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        preexec_fn=os.setsid,
    )


def stop(proc, name):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        return
    for _ in range(15):
        if proc.poll() is not None:
            return
        time.sleep(1)
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass


def run_trial(trial_name, bag_root, timeout_s=1800, scenario="bluerov2_turbine"):
    bag_path = os.path.join(bag_root, trial_name)
    sim_proc = bag_proc = ctrl_proc = None
    result = {"trial": trial_name, "success": False, "outcome": "unknown",
              "duration_s": None, "bag_path": bag_path}

    try:
        sim_proc = start(
            f"ros2 launch stonefish_bluerov2 bluerov2_sim.py "
            f"scenario:={scenario} rviz:=false"
        )
        # Wait for the sim + bridges to actually come up before recording/
        # launching the controller (matches this project's own documented
        # launch-order requirement).
        time.sleep(25)

        bag_proc = start(f"ros2 bag record {BAG_TOPICS} -o '{bag_path}'")
        time.sleep(3)

        ctrl_proc = start("ros2 run stonefish_bluerov2 bluerov2_autonomous_controller.py")

        start_t = time.time()
        outcome = "timeout"
        while True:
            line = ctrl_proc.stdout.readline()
            if not line:
                if ctrl_proc.poll() is not None:
                    outcome = "controller_crashed"
                    break
                continue
            if "MISSION COMPLETE" in line:
                outcome = "complete"
                break
            if re.search(r"\bESTOP\b", line):
                outcome = "estop"
                break
            if time.time() - start_t > timeout_s:
                outcome = "timeout"
                break

        result["duration_s"] = time.time() - start_t
        result["outcome"] = outcome
        result["success"] = (outcome == "complete")

    finally:
        stop(ctrl_proc, "controller")
        stop(bag_proc, "bag")
        stop(sim_proc, "sim")
        time.sleep(3)

    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("trial_name")
    p.add_argument("--bag-root", default=os.path.expanduser("~/ros2_ws/eval/bags"))
    p.add_argument("--timeout", type=int, default=1800)
    p.add_argument("--scenario", default="bluerov2_turbine")
    args = p.parse_args()
    os.makedirs(args.bag_root, exist_ok=True)

    res = run_trial(args.trial_name, args.bag_root, args.timeout, args.scenario)
    print(json.dumps(res))
