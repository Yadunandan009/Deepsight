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

# The arm and seed in the name are a claim about a system that is already
# running, and three bags so far have carried the wrong one -- each cost a
# 3x3 operator fit to work out what had actually been flown. Ask the live
# system instead of trusting the two words typed at the prompt.
LIVE_SEED="$(ros2 param get /dvl_bridge error_seed 2>/dev/null \
             | grep -oE '[0-9]+$' || true)"
SLAM_PUBS="$(ros2 topic info /bluerov2/map_points 2>/dev/null \
             | awk '/Publisher count/{print $3}' || true)"
BAD=""
if [ -n "$LIVE_SEED" ] && [ "$LIVE_SEED" != "$SEED" ]; then
  BAD="dvl_bridge is running error_seed=${LIVE_SEED}, you asked for seed ${SEED}"
elif [ -z "$LIVE_SEED" ]; then
  echo "WARNING: could not read /dvl_bridge error_seed -- seed unverified" >&2
fi
if [ -z "$BAD" ] && [ -n "$SLAM_PUBS" ]; then
  if [ "$ARM" = "dr" ] && [ "$SLAM_PUBS" -gt 0 ]; then
    BAD="SLAM is publishing, but the dr arm drops the SLAM topics"
  elif [ "$ARM" != "dr" ] && [ "$SLAM_PUBS" -eq 0 ]; then
    BAD="arm=${ARM} expects SLAM, but nothing is publishing /bluerov2/map_points"
  fi
fi
if [ -n "$BAD" ]; then
  echo "REFUSING TO RECORD: ${BAD}" >&2
  echo "Fix the arguments, or re-run with FORCE=1 to record anyway." >&2
  [ "${FORCE:-0}" = "1" ] || exit 1
  echo "FORCE=1 set -- recording under a name that does not match the system." >&2
fi

echo "arm=${ARM} seed=${SEED}  (live seed='${LIVE_SEED:-?}' slam_pubs='${SLAM_PUBS:-?}')"
echo "recording ${#TOPICS[@]} topics to ${BAG}"
echo "Ctrl+C to stop -- let it flush, do not kill it, or the bag loses its footer."
ros2 bag record "${TOPICS[@]}" -o "$BAG"
