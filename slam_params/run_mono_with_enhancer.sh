#!/bin/bash
# Launch mono SLAM WITH image enhancer running INSIDE Docker
# This avoids Jazzy/Humble DDS type hash incompatibility

# Enable X11 forwarding
xhost +local:docker 2>/dev/null || true

# Kill any existing containers
CONTAINER=$(docker ps --filter ancestor=orb-slam3-humble:bluerov2 --format "{{.ID}}")
[ -n "$CONTAINER" ] && docker kill $CONTAINER 2>/dev/null || true

# Create temp directory for scripts
SCRIPT_DIR=$(mktemp -d)
cat > "$SCRIPT_DIR/run.sh" << 'SCRIPT'
#!/bin/bash
source /opt/ros/humble/setup.bash
source /root/colcon_ws/install/setup.bash

# Start the image enhancer in background
python3 /tmp/image_enhancer_docker.py &
ENHANCER_PID=$!
echo "Started enhancer (PID: $ENHANCER_PID)"
sleep 2

# Start mono SLAM
ros2 run orb_slam3_ros2_wrapper mono \
    /home/orb/ORB_SLAM3/Vocabulary/ORBvoc.txt \
    /tmp/mono_down.yaml \
    --ros-args \
        --remap __ns:=/bluerov2_down \
        --params-file /tmp/mono_down_ros_params.yaml \
        -p use_sim_time:=true &

SLAM_PID=$!
echo "Started mono SLAM (PID: $SLAM_PID)"

# Wait for either process
wait -n
exit_code=$?
echo "A process exited with code $exit_code"
kill $ENHANCER_PID $SLAM_PID 2>/dev/null || true
exit $exit_code
SCRIPT
chmod +x "$SCRIPT_DIR/run.sh"

docker run -it --rm --network host --env DISPLAY=$DISPLAY \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume ~/ros2_ws/slam_params/bluerov2_mono_down.yaml:/tmp/mono_down.yaml:ro \
    --volume ~/ros2_ws/slam_params/bluerov2_mono_down_ros_params.yaml:/tmp/mono_down_ros_params.yaml:ro \
    --volume ~/ros2_ws/slam_params/image_enhancer_docker.py:/tmp/image_enhancer_docker.py:ro \
    --volume "$SCRIPT_DIR/run.sh":/tmp/run.sh:ro \
    orb-slam3-humble:bluerov2 bash /tmp/run.sh

rm -rf "$SCRIPT_DIR"
