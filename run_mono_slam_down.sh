#!/bin/bash
# Script to run Mono ORB-SLAM3 for the downward camera with corrected volume mount paths.
# Ensure that image_enhance.py is running to provide the /bluerov2/down/enhanced topic!

echo "Launching ORB-SLAM3 Mono configuration for Downward Camera..."
echo "Make sure bluerov2_sim.py is running to publish /bluerov2/down/image_color!"

# Start the image enhancer QoS bridge in the background
echo "Starting image QoS bridge automatically..."
python3 $HOME/ros2_ws/src/stonefish_bluerov2/scripts/image_enhance.py &
BRIDGE_PID=$!
sleep 2

xhost +local:docker
docker run -it --rm \
    --network host \
    --env DISPLAY=$DISPLAY \
    --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume $HOME/ros2_ws/slam_params/bluerov2_mono_down.yaml:/root/colcon_ws/src/orb_slam3_ros2_wrapper/params/orb_slam3_params/bluerov2_mono_down.yaml \
    --volume $HOME/ros2_ws/slam_params/bluerov2_mono_down_ros_params.yaml:/root/colcon_ws/src/orb_slam3_ros2_wrapper/params/ros_params/bluerov2_mono_down_ros_params.yaml \
    --volume $HOME/ros2_ws/slam_params/bluerov2_mono_down.yaml:/root/colcon_ws/install/orb_slam3_ros2_wrapper/share/orb_slam3_ros2_wrapper/params/orb_slam3_params/bluerov2_mono_down.yaml \
    --volume $HOME/ros2_ws/slam_params/bluerov2_mono_down_ros_params.yaml:/root/colcon_ws/install/orb_slam3_ros2_wrapper/share/orb_slam3_ros2_wrapper/params/ros_params/bluerov2_mono_down_ros_params.yaml \
    orb-slam3-humble:bluerov2 bash -c "
        source /opt/ros/humble/setup.bash &&
        source /root/colcon_ws/install/setup.bash &&
        ros2 run orb_slam3_ros2_wrapper mono \
            /home/orb/ORB_SLAM3/Vocabulary/ORBvoc.txt \
            /root/colcon_ws/src/orb_slam3_ros2_wrapper/params/orb_slam3_params/bluerov2_mono_down.yaml \
            --ros-args \
            -r __ns:=/bluerov2_down \
            -r camera/image_raw:=/bluerov2/down/enhanced \
            -r image_raw:=/bluerov2/down/enhanced \
            -r /camera_topic:=/bluerov2/down/enhanced \
            -r /topic_name:=/bluerov2/down/enhanced \
            -p qos_overrides./camera/image_raw.subscriber.reliability:=best_effort \
            -p qos_overrides./camera/image_raw.subscriber.history:=keep_last \
            -p qos_overrides./camera/image_raw.subscriber.depth:=10 \
            -p 'qos_overrides./bluerov2/down/image_color.subscriber.reliability:=best_effort' \
            -p 'qos_overrides./bluerov2/down/image_color.subscriber.history:=keep_last' \
            -p 'qos_overrides./bluerov2/down/image_color.subscriber.depth:=10' \
            --params-file /root/colcon_ws/src/orb_slam3_ros2_wrapper/params/ros_params/bluerov2_mono_down_ros_params.yaml"

kill $BRIDGE_PID
