cmake_minimum_required(VERSION 3.8)
project(stonefish_bluerov2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Include Cpp "include" directory
# include_directories(include)

# Create Cpp executable
# add_executable(cpp_executable src/cpp_node.cpp)
# ament_target_dependencies(cpp_executable rclcpp)

# Install Cpp executables
# install(TARGETS
#   cpp_executable
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/ardusim_patch.py
  scripts/bluerov2_teleop.py
  scripts/bluerov2_autonomous_controller.py
  scripts/bluerov2_expo_demo.py
  scripts/depth_bridge.py
  scripts/slam_init_move.py
  scripts/slam_pose_bridge.py
  scripts/dvl_bridge.py
  scripts/adaptive_fusion_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install scenario files
install(DIRECTORY
  scenarios
  DESTINATION share/${PROJECT_NAME}/
)

# Install data files
install(DIRECTORY
  data
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# --- ADDED THIS PART ---
# Install the EKF configuration file
install(FILES
  ekf.yaml
  DESTINATION share/${PROJECT_NAME}
)
# -----------------------

ament_package()
