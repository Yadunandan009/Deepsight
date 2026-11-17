#!/bin/bash
# Replay a recorded flight into ORB-SLAM3 offline, and record what it produces.
#
#   bash replay_slam.sh <bag> <label>                       stereo-inertial, current yaml
#   SLAM_MODE=stereo bash replay_slam.sh <bag> <label>      stereo-only on the same images
#   bash replay_slam.sh <bag> <label> <settings.yaml>       another config
#
# Needs a bag recorded with STEREO=1 (both eyes as grayscale, plus /bluerov2/imu).
# Every flight then serves as many SLAM experiments as there are
# configurations, all on the same images and the same trajectory, so a
# difference between two replays comes from the configuration alone.
#
# Half speed by default (RATE=1.0 for real time). The images keep their
# recorded stamps, so ORB-SLAM3's motion and IMU maths see the flight's own
# timing; only the wall-clock time it has per frame doubles. At real time it
# stalled on IMU initialisation and bundle adjustment, dropped the frames
# queued behind the stall, saw >1 s between consecutive frames and reset
# ("Timestamp jump detected", 5x in 23 min of replay vs 0 in the flight),
# which made a replay test the laptop's load rather than the configuration.
#
# Plays only the cameras and the IMU. The simulator must NOT be running: it
# publishes the live versions of these topics on the same ROS domain.
#
# Output: <BAG_ROOT>/replay_<label>_<original bag name>/, holding
# /bluerov2/robot_pose_slam and /bluerov2/map_points with the images' own
# stamps, so they line up with the original bag's ground truth. The SLAM
# console log is captured in slam_atlas/ as for any flight.
set -euo pipefail
BAG="${1:?usage: replay_slam.sh <bag> <label> [settings.yaml]}"
LABEL="${2:?usage: replay_slam.sh <bag> <label> [settings.yaml]}"
CFG="${3:-}"
RATE="${RATE:-0.5}"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG="${BAG%/}"
BAG_ROOT="/media/yadunandan/more space/bags"
OUT="${BAG_ROOT}/replay_${LABEL}_$(basename "$BAG")"

if ! grep -q "name: /bluerov2/left/image_gray_rec" "$BAG/metadata.yaml"; then
  echo "REFUSING: $BAG has no /bluerov2/left/image_gray_rec (record with STEREO=1)" >&2
  exit 1
fi
if ros2 topic list 2>/dev/null | grep -q "^/bluerov2/odometry$"; then
  echo "REFUSING: /bluerov2/odometry is live -- stop the simulator first" >&2
  exit 1
fi
if [ -e "$OUT" ]; then
  echo "REFUSING: $OUT already exists" >&2
  exit 1
fi

export SLAM_MODE="${SLAM_MODE:-stereo_imu}"
export LEFT_TOPIC=/bluerov2/left/image_gray_rec
export RIGHT_TOPIC=/bluerov2/right/image_gray_rec
echo "=== replay: $(basename "$BAG") -> ${SLAM_MODE}${CFG:+ ($CFG)} at ${RATE}x -> $OUT"
bash "$WS/run_stereo_slam.sh" $CFG

# ORB-SLAM3 loads its vocabulary before it subscribes; playing earlier would
# drop the first frames.
echo "waiting for ORB-SLAM3 to subscribe to $LEFT_TOPIC ..."
for i in $(seq 1 120); do
  if ros2 topic info "$LEFT_TOPIC" 2>/dev/null | grep -q "Subscription count: [1-9]"; then
    echo "  subscribed after ${i}s"; break
  fi
  sleep 1
  if [ "$i" -eq 120 ]; then echo "ORB-SLAM3 never subscribed; see the console log" >&2; exit 1; fi
done
sleep 3

# SIGTERM, not SIGINT: bash starts a background job of a script with SIGINT
# ignored, so 'kill -INT' never reached the recorder -- it kept recording,
# never wrote metadata.yaml, and 'wait' hung (2026-11-16). The recorder
# catches SIGTERM and closes the bag the same way.
ros2 bag record /bluerov2/robot_pose_slam /bluerov2/map_points -o "$OUT" >/dev/null 2>&1 &
REC=$!
trap 'kill -TERM $REC 2>/dev/null || true' EXIT
sleep 2

PLAY_RC=0
ros2 bag play "$BAG" --rate "$RATE" --topics "$LEFT_TOPIC" "$RIGHT_TOPIC" /bluerov2/imu || PLAY_RC=$?
if [ "$PLAY_RC" -ne 0 ]; then
  echo "WARNING: bag play stopped early (status $PLAY_RC); the replay is INCOMPLETE" >&2
fi

sleep 5
kill -TERM $REC; wait $REC 2>/dev/null || true
trap - EXIT
bash "$WS/stop_slam.sh"
echo "${RATE}" > "$OUT/replay_rate.txt"
echo "=== replay done: $OUT"
