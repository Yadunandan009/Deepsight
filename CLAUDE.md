# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

```bash
# Build all ROS2 packages
colcon build --event-handlers console_direct+

# Build specific package with symlink install
colcon build --event-handlers console_direct+ --cmake-args --symlink-install --packages-select <package>

# Build ArduPilot SITL (from ardupilot directory)
./waf configure --board SITL
./waf sub

# Source workspace
source install/setup.bash
source /opt/ros/jazzy/setup.bash
```

## Simulation Architecture

### Stack Overview
```
QGroundControl <--MAVLINK--> ArduSub SITL <--JSON--> ardusim_patch.py <--> stonefish_ros2 (simulator)
                                      |
                                      +--> robot_localization (EKF fusion)
                                      +--> bluerov2_interface (MAVLink bridge)
```

### Key Packages
- **stonefish_ros2**: C++ Stonefish simulator ROS2 wrapper (launches `stonefish_simulator` node)
- **stonefish_bluerov2**: BlueROV2 simulation configs, scenarios (.scn), launch files, and Python controllers
- **bluerov2_interface**: MAVLink-to-ROS2 bridge (publishes /bluerov2/imu, /odometry, /altitude; subscribes to /set_pwm)
- **robot_localization**: EKF state estimation (ekf_filter_node) fusing SLAM pose + depth + IMU
- **ardupilot**: ArduPilot firmware including ArduSub for underwater ROVs

### Launch Files
| Launch | Purpose |
|--------|---------|
| `stonefish_bluerov2/bluerov2_sim.py` | Full BlueROV2 simulation: Stonefish + ardusim_patch + depth_bridge + EKF + RViz |
| `stonefish_bluerov2/blueboat_sim.py` | Surface vessel simulation |
| `stonefish_ros2/stonefish_simulator.launch.py` | Base Stonefish simulator only |

### Scenario Files (Stonefish)
Located in `src/stonefish_bluerov2/scenarios/*.scn` - XML descriptions of underwater environments with robot, obstacles, currents.

### EKF Configuration
`src/stonefish_bluerov2/ekf.yaml` - fuses /bluerov2/robot_pose_slam_fused (pose0), /bluerov2/depth_pose (pose1), /bluerov2/imu (imu0)

## Running Simulations

```bash
# 1. Start ArduSub SITL (from ardupilot dir)
cd ardupilot
sim_vehicle.py -v ArduSub --model JSON --map -L PHILL -m --streamrate=-1

# 2. Launch simulation (from ros2_ws)
ros2 launch stonefish_bluerov2 bluerov2_sim.py scenario:=bluerov2_turbine

# 3. Open QGroundControl, set vehicle to Vectored-6DOF, enable joystick

# Record mission bag
bash record_mission.sh
```

## Key Python Scripts

| Script | Purpose |
|--------|---------|
| `scripts/bluerov2_autonomous_controller.py` | DeepSight autonomous inspection controller (DESCEND→CLOSE_IN→SCAN→ROTATE→...) |
| `scripts/ardusim_patch.py` | Bridges Stonefish physics to ArduSub SITL via JSON protocol |
| `scripts/depth_bridge.py` | Converts pressure sensor to depth pose for EKF |
| `scripts/deepsight_sonar_ekf.py` | Sonar-based EKF localization |
| `bluerov2_bridge/bridge.py` | MAVLink-ROS2 message translation |

## Important Topics

**Subscribers:**
- `/bluerov2/setpoint/pwm` - thruster PWM commands
- `/bluerov2/heartbeat` - vehicle heartbeat

**Publishers:**
- `/bluerov2/imu` - IMU data
- `/bluerov2/odometry` - filtered odometry from EKF
- `/bluerov2/altitude`, `/bluerov2/battery`, `/bluerov2/bottle_pressure`

## Notes
- ArduSub must be running before launching stonefish simulation
- stonefish_ros2 v1.3 required (matching Stonefish library v1.3)
- BlueROV2 uses Vectored-6DOF frame in QGroundControl for 6 thruster configuration