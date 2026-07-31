#!/bin/bash
# Fixed mono SLAM launch script
# This script runs mono ORB-SLAM3 subscribing directly to Stonefish raw images
# avoiding the Jazzy/Humble DDS incompatibility issue

# Enable X11 forwarding for visualization
xhost +local:docker 2>/dev/null || true

# Run mono SLAM with direct raw image subscription
docker run -it --rm \
    --network host \
    --env DISPLAY=$DISPLAY \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume ~/ros2_ws/slam_params/bluerov2_mono_down.yaml:/tmp/mono_down.yaml:ro \
    --volume ~/ros2_ws/slam_params/bluerov2_mono_down_ros_params.yaml:/tmp/mono_down_ros_params.yaml:ro \
    orb-slam3-humble:bluerov2 bash -c "
        source /opt/ros/humble/setup.bash &&
        source /root/colcon_ws/install/setup.bash &&
        ros2 run orb_slam3_ros2_wrapper mono \
            /home/orb/ORB_SLAM3/Vocabulary/ORBvoc.txt \
            /tmp/mono_down.yaml \
            --ros-args \
                --remap __ns:=/bluerov2_down \
                --params-file /tmp/mono_down_ros_params.yaml \
                -p use_sim_time:=true
    "

echo "Mono SLAM stopped."
