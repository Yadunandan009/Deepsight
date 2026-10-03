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
# /bluerov2/map_points added 2026-10-03: the SLAM point cloud is what the
# mission actually produces, and without it there is no way to show
# reconstruction quality -- or its distortion -- in a figure. It is also a
# candidate second axis for the symmetry dose-response, alongside re-align
# counts. Costs bag size, which is why trials are capped at 180 s anyway.
BAG_TOPICS = ("/bluerov2/odometry /bluerov2/robot_pose_slam_ekf "
              "/bluerov2/setpoint/pwm /bluerov2/map_points")


def start(cmd, extra_env=None):
    # extra_env goes through Popen's own env= rather than shell string
    # concatenation -- confirmed 2026-09-24 that "exec VAR=val cmd" fails
    # ("exec: VAR=val: not found"): a bash assignment prefix only attaches
    # to the command word that immediately follows it, and "exec" IS that
    # command word here, so "VAR=val" was being handed to exec as the
    # literal program name instead of becoming part of cmd's environment.
    env = os.environ.copy()
    if extra_env:
        env.update(extra_env)
    return subprocess.Popen(
        f"{ROS_SETUP} && exec {cmd}",
        shell=True, executable="/bin/bash",
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        preexec_fn=os.setsid,
        env=env,
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


def run_trial(trial_name, bag_root, timeout_s=1800, scenario="bluerov2_turbine",
              buggy_alloc=False):
    bag_path = os.path.join(bag_root, trial_name)
    if os.path.exists(bag_path):
        # ros2 bag record refuses to start if the output dir already exists,
        # but its subprocess failing doesn't stop the sim/controller from
        # running to completion -- which previously produced a trial logged
        # as "success" with a stale or entirely missing bag underneath it
        # (confirmed 2026-09-23: a re-run of fixed_00/fixed_01 collided with
        # bags left from an earlier interrupted batch). Fail before wasting
        # a full mission's wall-clock time on a trial whose telemetry can
        # never be recorded.
        raise FileExistsError(
            f"bag directory already exists: {bag_path} -- remove it or "
            f"choose a different --prefix/trial name before rerunning")

    sim_proc = bag_proc = ctrl_proc = None
    result = {"trial": trial_name, "success": False, "outcome": "unknown",
              "duration_s": None, "bag_path": bag_path, "buggy_alloc": buggy_alloc}

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

        ctrl_extra_env = {"BLUEROV2_BUGGY_ALLOC": "1"} if buggy_alloc else None
        ctrl_proc = start("ros2 run stonefish_bluerov2 bluerov2_autonomous_controller.py",
                           extra_env=ctrl_extra_env)

        # Every line is echoed to a per-trial log -- previously only checked
        # against two magic strings and otherwise discarded, so a fast crash
        # (confirmed 2026-09-24: all 10 buggy_alloc trials died in <0.4s)
        # left literally no record of why.
        ctrl_log_path = bag_path + "_ctrl_stdout.log"
        with open(ctrl_log_path, "w") as ctrl_log:
            start_t = time.time()
            outcome = "timeout"
            while True:
                line = ctrl_proc.stdout.readline()
                if not line:
                    if ctrl_proc.poll() is not None:
                        outcome = "controller_crashed"
                        break
                    continue
                ctrl_log.write(line)
                ctrl_log.flush()
                if "MISSION COMPLETE" in line:
                    outcome = "complete"
                    break
                if re.search(r"\bESTOP\b", line):
                    outcome = "estop"
                    break
                if time.time() - start_t > timeout_s:
                    outcome = "timeout"
                    break
        result["ctrl_log_path"] = ctrl_log_path

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
    p.add_argument("--buggy-alloc", action="store_true",
                    help="Use the pre-fix (surge-column sign error) thruster "
                         "allocation matrix -- Task 9's ablation counterfactual.")
    args = p.parse_args()
    os.makedirs(args.bag_root, exist_ok=True)

    res = run_trial(args.trial_name, args.bag_root, args.timeout, args.scenario,
                     args.buggy_alloc)
    print(json.dumps(res))
