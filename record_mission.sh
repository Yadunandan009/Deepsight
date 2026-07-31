#!/bin/bash
# DeepSight — Record all mission topics including SLAM
# Usage: bash record_mission.sh

BAG_NAME="DeepSight_$(date +%Y%m%d_%H%M%S)"

echo "Recording to ~/ros2_ws/bags/${BAG_NAME} ..."
echo "Press Ctrl+C to stop."

ros2 bag record \
  /bluerov2/odometry \
  /bluerov2/robot_pose_slam \
  /bluerov2/robot_pose_slam_fused \
  /bluerov2/map_points \
  /bluerov2/multibeam_raw \
  /bluerov2/multibeam \
  /bluerov2/setpoint/pwm \
  -o ~/ros2_ws/bags/"${BAG_NAME}"
