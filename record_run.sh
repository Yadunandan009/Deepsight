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
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
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
# The stereo baseline is the wide-baseline experiment's ONLY variable, and
# nothing else in this script would notice it being wrong. Two numbers have
# to agree and both are read from the running system, not from the tree:
#
#   what Stonefish is simulating  -- right CameraInfo, P[3] = -fx*b
#   what ORB-SLAM3 is solving     -- Stereo.b in the yaml bind-mounted into
#                                    the live container
#
# eval/check_stereo_baseline.py checks the files on disk and cannot see
# either of these. A stale colcon build, or launching bluerov2_turbine with
# bluerov2_stereo_b30.yaml, passes that check and produces a bag whose
# recorded geometry is not the geometry that was flown.
LIVE_B="$(python3 "${WS}/eval/live_stereo_baseline.py" --timeout 10 2>/dev/null || true)"
SLAM_CFG=""
SLAM_B=""
if docker inspect orb_slam3_running >/dev/null 2>&1; then
  # Pick the mount by CONTENT, not by name. bluerov2-stereo-ros-params.yaml
  # matches any sensible name pattern but carries no Stereo.b, and docker
  # does not promise to list mounts in the order they were given.
  for m in $(docker inspect -f '{{range .Mounts}}{{.Source}}{{"\n"}}{{end}}' \
             orb_slam3_running 2>/dev/null || true); do
    [ -f "$m" ] || continue
    grep -q '^Stereo.b:' "$m" || continue
    SLAM_CFG="$m"
    SLAM_B="$(grep -m1 '^Stereo.b:' "$m" | awk '{print $2}')"
    break
  done
fi

if [ -z "$BAD" ]; then
  if [ -z "$LIVE_B" ]; then
    echo "WARNING: no right CameraInfo -- simulated stereo baseline unverified" >&2
  elif [ "$ARM" != "dr" ] && [ -z "$SLAM_B" ]; then
    echo "WARNING: could not read Stereo.b from the running SLAM container" >&2
    echo "         -- simulated baseline is ${LIVE_B} m, SLAM side unverified" >&2
  elif [ "$ARM" != "dr" ] && [ -n "$SLAM_B" ]; then
    # bash has no floats; compare at 1 mm in awk
    if ! awk -v a="$LIVE_B" -v b="$SLAM_B" \
         'BEGIN{exit !(a-b<0.002 && b-a<0.002)}'; then
      BAD="stereo baseline mismatch: Stonefish is simulating ${LIVE_B} m but \
ORB-SLAM3 is solving for ${SLAM_B} m (from ${SLAM_CFG}). The scenario and the \
stereo yaml are from different arms."
    fi
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

echo "arm=${ARM} seed=${SEED}  (live seed='${LIVE_SEED:-?}' scale_sigma='${LIVE_SIGMA:-?}' slam_pubs='${SLAM_PUBS:-?}')"
echo "  stereo baseline: sim='${LIVE_B:-?}' m slam='${SLAM_B:-?}' m  (gate = b*ThDepth)"
echo "recording ${#TOPICS[@]} topics to ${BAG}"
echo "Ctrl+C to stop -- let it flush, do not kill it, or the bag loses its footer."
ros2 bag record "${TOPICS[@]}" -o "$BAG"
