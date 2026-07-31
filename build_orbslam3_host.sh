#!/bin/bash
# Build ORB-SLAM3 on ROS2 Jazzy host
# This is the ONLY way to get mono SLAM working with the current setup

set -e

cd ~/ros2_ws/src

# Clone ORB-SLAM3 if not present
if [ ! -d "ORB_SLAM3" ]; then
    echo "Cloning ORB-SLAM3..."
    git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
fi

# Clone ROS2 wrapper
if [ ! -d "orb_slam3_ros2_wrapper" ]; then
    echo "Cloning ROS2 wrapper..."
    git clone https://github.com/zkytony/orb_slam3_ros2_wrapper.git
fi

# Install dependencies
echo "Installing dependencies..."
sudo apt update
sudo apt install -y libglew-dev libopencv-dev libboost-all-dev libssl-dev

# Build ORB-SLAM3
cd ORB_SLAM3
echo "Building ORB-SLAM3 (this takes ~20 minutes)..."
chmod +x build.sh
./build.sh

# Download vocabulary
cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

# Build ROS2 wrapper
cd ~/ros2_ws
echo "Building ROS2 wrapper..."
colcon build --packages-select orb_slam3_ros2_wrapper --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "Build complete! Source with:"
echo "  source ~/ros2_ws/install/setup.bash"
echo ""
echo "Run mono SLAM with:"
echo "  ros2 run orb_slam3_ros2_wrapper mono /home/yadunandan/ros2_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt ~/ros2_ws/slam_params/bluerov2_mono_down.yaml --ros-args --remap __ns:=/bluerov2_down -p image_topic_name:=/bluerov2/down/image_color -p use_sim_time:=true"
