#!/bin/bash
# Record one arm of the DVL error-model experiment.
#
#   bash record_run.sh dr   0      dead reckoning only, dvl_seed 0
#   bash record_run.sh slam 1      with ORB-SLAM3, dvl_seed 1
#
# The arm and seed go in the bag name, because three runs that differ only
# by a launch argument are otherwise impossible to tell apart afterwards.
set -euo pipefail
# ARM is free text and goes straight into the bag name. It is the only
# durable record of which arm a bag belongs to, so it must name the arm
# actually flown. Anything other than "dr" means SLAM is expected.
#   dr         dead reckoning only
#   slamref    reference arm, b=0.145, Camera.fps 30
#   slamref12  reference arm, b=0.145, Camera.fps 12
#   slamb30    wide baseline, b=0.30,  Camera.fps 12
ARM="${1:?usage: record_run.sh <arm> <seed>  (see the list above)}"
SEED="${2:?usage: record_run.sh <arm> <seed>}"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_ROOT="/media/yadunandan/more space/bags"
BAG="${BAG_ROOT}/a50_${ARM}_seed${SEED}_$(date +%Y%m%d_%H%M%S)"
# If the external drive is not mounted, mkdir -p would quietly create the path on
# the root disk (about 5 GB free) and the bag would fill it.
if ! mountpoint -q "/media/yadunandan/more space"; then
  echo "REFUSING TO RECORD: the external drive is not mounted at /media/yadunandan/more space" >&2
  exit 1
fi
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
# The SLAM-free reference filter, when dr_reference:=true put one up. Without
# it there is no way to check afterwards whether the reference the alignment
# was fitted against was itself healthy -- which is the whole claim under test.
if ros2 topic info /bluerov2/odometry/dr >/dev/null 2>&1; then
  TOPICS+=(/bluerov2/odometry/dr)
fi

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
# error_seed alone does NOT establish that the a50 model is running. The
# launch file puts the plain bridge and the a50 bridge under opposite
# conditions with the SAME node name, and both declare error_seed, so a
# launch that simply omitted dvl_error:=a50 answers this question with the
# seed you asked for and a scale_sigma of zero. The seed-2 reference run of
# 2026-10-14 was recorded that way: correct seed, identity error operator,
# white-noise DVL under an a50 bag name. scale_sigma is what separates them.
LIVE_SIGMA="$(ros2 param get /dvl_bridge scale_sigma 2>/dev/null \
              | grep -oE '[0-9]+\.?[0-9]*$' || true)"
SLAM_PUBS="$(ros2 topic info /bluerov2/map_points 2>/dev/null \
             | awk '/Publisher count/{print $3}' || true)"
BAD=""
if [ -n "$LIVE_SEED" ] && [ "$LIVE_SEED" != "$SEED" ]; then
  BAD="dvl_bridge is running error_seed=${LIVE_SEED}, you asked for seed ${SEED}"
elif [ -z "$LIVE_SEED" ]; then
  echo "WARNING: could not read /dvl_bridge error_seed -- seed unverified" >&2
fi
if [ -z "$BAD" ]; then
  if [ -z "$LIVE_SIGMA" ]; then
    echo "WARNING: could not read /dvl_bridge scale_sigma -- error model unverified" >&2
  elif [ "$LIVE_SIGMA" = "0.0" ] || [ "$LIVE_SIGMA" = "0" ]; then
    BAD="dvl_bridge has scale_sigma=0, so the a50 error model is OFF. Relaunch with dvl_error:=a50"
  fi
fi
# The stereo baseline is the wide-baseline experiment's ONLY variable and
# nothing else here would notice it being wrong. Both sides are read from
# the RUNNING system by eval/check_stereo_baseline.py --live:
#
#   simulator -- the .scn on the stonefish process command line, which is
#                the INSTALLED copy, so a source edit without a colcon
#                build is caught too
#   ORB-SLAM3 -- Stereo.b in the yaml bind-mounted into the container
#
# An earlier version of this read the baseline from the right camera's
# CameraInfo P[3] = -fx*b. Stonefish publishes P[3] = 0 and never puts the
# baseline there, so that check refused every SLAM arm, including the
# reference. Do not reintroduce it.
BASE_OUT="$(python3 "${WS}/eval/check_stereo_baseline.py" --live 2>&1 || true)"
BASE_RC=$?
LIVE_B="$(printf '%s' "$BASE_OUT" | sed -n 's/.*both sides are b = \([0-9.]*\).*/\1/p')"
if [ -z "$BAD" ] && [ "$ARM" != "dr" ]; then
  if printf '%s' "$BASE_OUT" | grep -q "MISMATCH:"; then
    BAD="$(printf '%s' "$BASE_OUT" | sed -n '/MISMATCH:/,$p' | tr '\n' ' ')"
  elif printf '%s' "$BASE_OUT" | grep -q "INCOMPLETE:"; then
    echo "WARNING: stereo baseline unverified --" >&2
    printf '%s\n' "$BASE_OUT" | sed 's/^/         /' >&2
  fi
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

# Optional camera frames:  IMAGES=1 bash record_run.sh imgL0 1
# The left camera as rendered (image_color) and as ORB-SLAM3 receives it
# (image_enhanced), each throttled to IMAGE_HZ (default 1 Hz): about 2.4 GB per
# mission at 1 Hz, against roughly 70 GB per topic at the full 30 Hz.
THROTTLE_PIDS=()
if [ "${IMAGES:-0}" = "1" ]; then
  HZ="${IMAGE_HZ:-1.0}"
  for t in image_color image_enhanced; do
    ros2 run topic_tools throttle messages "/bluerov2/left/${t}" "$HZ" "/bluerov2/left/${t}_rec" >/dev/null 2>&1 &
    THROTTLE_PIDS+=($!)
    TOPICS+=("/bluerov2/left/${t}_rec")
  done
  trap 'kill "${THROTTLE_PIDS[@]}" 2>/dev/null || true' EXIT
  sleep 2
  echo "  camera frames: left image_color and image_enhanced at ${HZ} Hz"
fi

echo "arm=${ARM} seed=${SEED}  (live seed='${LIVE_SEED:-?}' scale_sigma='${LIVE_SIGMA:-?}' slam_pubs='${SLAM_PUBS:-?}')"
echo "  stereo baseline: ${LIVE_B:-unverified} m (both sides agree; gate = b*ThDepth)"
echo "recording ${#TOPICS[@]} topics to ${BAG}"
echo "Ctrl+C to stop -- let it flush, do not kill it, or the bag loses its footer."
ros2 bag record "${TOPICS[@]}" -o "$BAG"
