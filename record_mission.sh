#!/bin/bash
# DeepSight — Record all mission topics including SLAM
# Usage: bash record_mission.sh

BAG_NAME="DeepSight_$(date +%Y%m%d_%H%M%S)"
BAG_ROOT="/media/yadunandan/more space/bags"
mkdir -p "$BAG_ROOT"

# Root filesystem is at 96% (4.3GB free) as of tonight -- map_points in
# particular is large and grows for the whole mission. Recording there
# risks repeating tonight's near-crash from root filling up.
echo "Recording to ${BAG_ROOT}/${BAG_NAME} ..."
echo "Press Ctrl+C to stop."

ros2 bag record \
  /bluerov2/odometry \
  /bluerov2/robot_pose_slam \
  /bluerov2/robot_pose_slam_ekf \
  /bluerov2/map_points \
  /bluerov2/multibeam_raw \
  /bluerov2/multibeam \
  /bluerov2/setpoint/pwm \
  -o "${BAG_ROOT}/${BAG_NAME}"
