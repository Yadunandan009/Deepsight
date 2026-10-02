#!/bin/bash
# Launches bluerov2_sim.py with venv-ardupilot stripped from PATH first.
#
# ~/.profile unconditionally activates venv-ardupilot in every new
# terminal (kept there on purpose, for the ArduPilot SITL build toolchain
# further down the same file). Its numpy is newer than what cv_bridge was
# built against, so any node that imports cv_bridge -- specifically
# image_enhancement_node.py -- crashes silently at startup in any
# terminal where that env is active, while every other node (no
# cv_bridge dependency) starts fine and looks normal. This has caused
# three separate confusing failures in one night: a cv_bridge crash, an
# image-enhancement node never starting, and ORB-SLAM3 stuck on "WAITING
# FOR IMAGES" because nothing was publishing to what it was listening to.
#
# Output is also teed to logs/launch_<timestamp>.log. ROS launch logs do
# NOT persist node stdout -- checked 2026-10-02, 0 of 340 dirs under
# ~/.ros/log contained slam_pose_bridge's "SLAM alignment solved" lines --
# so without this, the per-run alignment history that
# eval/slam_align_harvest.py analyses is lost as soon as the terminal
# scrolls. Keeping it costs nothing and the harvester needs several runs
# pooled before its 90-degree clustering test has any power.
#
# Usage: bash launch_sim.sh [scenario:=name] [any other ros2 launch args]
# Defaults to the same launch used throughout this project if no args given.

export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v venv-ardupilot | tr '\n' ':')
unset VIRTUAL_ENV
source /opt/ros/jazzy/setup.bash
source /home/yadunandan/ros2_ws/install/setup.bash

mkdir -p /home/yadunandan/ros2_ws/logs
LOG=/home/yadunandan/ros2_ws/logs/launch_$(date +%Y%m%d_%H%M%S).log
echo "logging to $LOG"

ros2 launch stonefish_bluerov2 bluerov2_sim.py "$@" 2>&1 | tee "$LOG"
