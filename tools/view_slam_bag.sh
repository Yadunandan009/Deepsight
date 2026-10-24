#!/usr/bin/env bash
# Replay ORB-SLAM3's map and pose from a recorded flight in RViz2.
#
#   bash tools/view_slam_bag.sh bags/<bag> [start_offset_s] [rate]
#   e.g. bash tools/view_slam_bag.sh bags/a50_ctlL0_seed1_20261021_234743 950 1
#
# Why the extra pieces: the bags record no /tf, and /bluerov2/map_points carries
# frame "map" with a zero timestamp. RViz2 refuses a fixed frame that is not in the
# tf tree ("Fixed Frame [map] does not exist"), so a static transform is published
# to put "map" in the tree. Camera images were never recorded, so this shows the
# map points and the SLAM pose, not the ORB-SLAM3 viewer.
set -euo pipefail
BAG=${1:?usage: view_slam_bag.sh <bag_dir> [start_offset_s] [rate]}
OFF=${2:-0}
RATE=${3:-1}
HERE="$(cd "$(dirname "$0")" && pwd)"
set +u                      # ROS setup scripts read unset variables
source /opt/ros/jazzy/setup.bash
set -u
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id map_anchor >/dev/null 2>&1 &
TF=$!
rviz2 -d "$HERE/slam_bag.rviz" >/dev/null 2>&1 &
RV=$!
trap 'kill $TF $RV 2>/dev/null || true' EXIT
sleep 3
ros2 bag play "$BAG" --start-offset "$OFF" --rate "$RATE"
echo "playback finished; close RViz2 to exit"
wait $RV
