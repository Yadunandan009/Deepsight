#!/bin/bash
# Which stereo config to mount. The wide-baseline arm needs its own, and
# a run launched with the scenario of one arm and the yaml of the other is
# silently void -- the geometry recorded is not the geometry simulated.
#
#   bash run_stereo_slam.sh                      reference, b = 0.145 m
#   bash run_stereo_slam.sh bluerov2_stereo_b30.yaml   wide, b = 0.30 m
#
# SLAM_MODE=stereo_imu runs stereo-inertial ORB-SLAM3 instead (IMU from
# /bluerov2/imu; settings default to bluerov2_stereo_imu.yaml, which is the
# reference stereo config plus an IMU block) in the bluerov2-heading-vi image
# (Dockerfile.heading-vi), whose stereo-imu node feeds the heading check.
#   SLAM_MODE=stereo_imu bash run_stereo_slam.sh
MODE="${SLAM_MODE:-stereo}"
case "$MODE" in
    stereo)     DEFAULT_CFG=bluerov2_stereo.yaml;     ROSP=bluerov2-stereo-ros-params.yaml;     ROSP_AT=euroc-stereo-ros-params.yaml ;;
    stereo_imu) DEFAULT_CFG=bluerov2_stereo_imu.yaml; ROSP=bluerov2-stereo-imu-ros-params.yaml; ROSP_AT=euroc-stereo-imu-ros-params.yaml ;;
    *) echo "SLAM_MODE must be stereo or stereo_imu, got '$MODE'" >&2; exit 1 ;;
esac
CFG="${1:-$DEFAULT_CFG}"
# Image input. The live system feeds the enhanced images; replay_slam.sh points
# these at the grayscale copies record_run.sh STEREO=1 keeps in the bag.
LEFT_TOPIC="${LEFT_TOPIC:-/bluerov2/left/image_enhanced}"
RIGHT_TOPIC="${RIGHT_TOPIC:-/bluerov2/right/image_enhanced}"
# bluerov2-heading adds the heading-consistency check (Heading.* in the
# stereo yaml; build with ~/ORB-SLAM3-ROS2-Docker/Dockerfile.heading).
# SLAM_IMAGE=orb-slam3-humble:bluerov2 runs the image of every earlier flight.
# Stereo-inertial uses bluerov2-heading-vi (Dockerfile.heading-vi): the same
# image plus the stereo-imu node's heading feed, so stereo runs are untouched.
if [ "$MODE" = "stereo_imu" ]; then DEFAULT_IMAGE=orb-slam3-humble:bluerov2-heading-vi
else DEFAULT_IMAGE=orb-slam3-humble:bluerov2-heading; fi
IMAGE="${SLAM_IMAGE:-$DEFAULT_IMAGE}"
SLAM="$HOME/ros2_ws/slam_params"
if [ ! -f "$SLAM/$CFG" ]; then
    echo "no such stereo config: $SLAM/$CFG" >&2
    exit 1
fi
B="$(grep -m1 '^Stereo.b:' "$SLAM/$CFG" | awk '{print $2}')"
echo "stereo config: $CFG   (Stereo.b = $B m, close-depth gate = b*ThDepth)"
HMODE="$(grep -m1 '^Heading.Mode:' "$SLAM/$CFG" | awk '{print $2}')"
echo "=================================================================="
echo "  Heading.Mode = ${HMODE:-unset}   (1 -> stkL0 / stkL5,  2 -> vetoL0)"
echo "=================================================================="
echo "Check this matches the scenario you launched:"
echo "   b=0.145 -> scenario:=bluerov2_turbine"
echo "   b=0.3   -> scenario:=bluerov2_turbine_b30"
echo "   eval/check_stereo_baseline.py verifies the pairing."
PKG="/root/colcon_ws/src/orb_slam3_ros2_wrapper/params"
INST="/root/colcon_ws/install/orb_slam3_ros2_wrapper/share/orb_slam3_ros2_wrapper/params"
echo "=== ORB-SLAM3 ($MODE) for BlueROV2 ===   image: $IMAGE"
xhost +local:docker 2>/dev/null
docker rm -f orb_slam3_running 2>/dev/null
docker run -d \
    --name orb_slam3_running \
    --network host \
    --ipc=host \
    --env DISPLAY="$DISPLAY" \
    --env QT_X11_NO_MITSHM=1 \
    --env XDG_RUNTIME_DIR=/tmp \
    --env ROS_DOMAIN_ID=0 \
    --env LD_LIBRARY_PATH=/home/orb/ORB_SLAM3/lib:/root/colcon_ws/install/orb_slam3_ros2_wrapper/lib/orb_slam3_ros2_wrapper \
    --device /dev/dri:/dev/dri \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$SLAM/$CFG:$PKG/orb_slam3_params/bluerov2_stereo.yaml" \
    -v "$SLAM/$CFG:$INST/orb_slam3_params/bluerov2_stereo.yaml" \
    -v "$SLAM/$ROSP:$PKG/ros_params/$ROSP_AT" \
    -v ~/ros2_ws/slam_atlas:/tmp/host \
    -v "$SLAM/$ROSP:$INST/ros_params/$ROSP_AT" \
    "$IMAGE" bash -c "\
        source /opt/ros/humble/setup.bash && \
        source /root/colcon_ws/install/setup.bash && \
        ros2 run orb_slam3_ros2_wrapper $MODE \
            /home/orb/ORB_SLAM3/Vocabulary/ORBvoc.txt \
            $PKG/orb_slam3_params/bluerov2_stereo.yaml \
            --ros-args \
            -r __ns:=/bluerov2 \
            -r left/image_raw:=$LEFT_TOPIC \
            -r right/image_raw:=$RIGHT_TOPIC \
            -p approximate_sync:=true \
            --params-file $PKG/ros_params/$ROSP_AT"

# Capture the container's own stdout (ORB-SLAM3 prints keyframe insertion,
# loop-closing, and map creation/merging diagnostics directly) to a file --
# previously this was thrown away once `docker rm -f` wiped the container at
# the top of the next run, so a genuine Atlas-level event (vs. a ROS-side
# publish stall) could never be distinguished after the fact.
LOG_DIR="$HOME/ros2_ws/slam_atlas"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/orb_slam3_console_$(date +%Y%m%d_%H%M%S).log"
docker logs -f orb_slam3_running > "$LOG_FILE" 2>&1 &
echo "Capturing container console output to $LOG_FILE"

echo "SLAM container started. To stop cleanly run: ~/ros2_ws/stop_slam.sh"
