#!/bin/bash
# Record one arm of the DVL error-model experiment.
#
#   bash record_run.sh dr   0      dead reckoning only, dvl_seed 0
#   bash record_run.sh slam 1      with ORB-SLAM3, dvl_seed 1
#
# The arm and seed go in the bag name, because three runs that differ only
# by a launch argument are otherwise impossible to tell apart afterwards.
set -euo pipefail
ARM="${1:?usage: record_run.sh <dr|slam|gated> <seed>}"
SEED="${2:?usage: record_run.sh <dr|slam|gated> <seed>}"
BAG_ROOT="/media/yadunandan/more space/bags"
BAG="${BAG_ROOT}/a50_${ARM}_seed${SEED}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BAG_ROOT"

# /bluerov2/dvl_twist is the bridge OUTPUT, i.e. the corrupted velocity the
# filter actually sees. Without it there is no way to confirm after the fact
# that the a50 error model was switched on, or which unit was simulated.
TOPICS=(
  /bluerov2/odometry            # Stonefish ground truth
  /bluerov2/odometry/filtered   # EKF output -- what the controller consumes
  /bluerov2/dvl_twist
  /bluerov2/depth_pose
  /bluerov2/imu
  /bluerov2/multibeam_raw
)
# map_points grows for the whole mission and is only meaningful with SLAM up.
if [ "$ARM" != "dr" ]; then
  TOPICS+=(/bluerov2/robot_pose_slam /bluerov2/robot_pose_slam_ekf /bluerov2/map_points)
fi

echo "arm=${ARM} seed=${SEED}"
echo "recording ${#TOPICS[@]} topics to ${BAG}"
echo "Ctrl+C to stop -- let it flush, do not kill it, or the bag loses its footer."
ros2 bag record "${TOPICS[@]}" -o "$BAG"
