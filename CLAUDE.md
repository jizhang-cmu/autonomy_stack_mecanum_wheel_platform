# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

This is a ROS 2 Jazzy autonomy stack for the Mecanum wheel platform (T-Bot), containing:
- SLAM module with Mid-360 lidar support
- Base autonomy system (terrain analysis, collision avoidance, waypoint following)
- Route planner (FAR Planner based)
- Exploration planner (TARE Planner based)
- Unity simulation environment support

## Essential Build Commands

### Full Build
```bash
cd autonomy_stack_mecanum_wheel_platform
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Simulation Build (skips SLAM and Mid-360 driver)
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-skip arise_slam_mid360 arise_slam_mid360_msgs livox_ros_driver2
```

### Single Package Build
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select [package_name]
```

### Clean Build
```bash
rm -rf build install log
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## System Launch Commands

### Simulation
- Base autonomy: `./system_simulation.sh`
- With route planner: `./system_simulation_with_route_planner.sh`
- With exploration planner: `./system_simulation_with_exploration_planner.sh`

### Real Robot
- Base autonomy: `./system_real_robot.sh`
- With route planner: `./system_real_robot_with_route_planner.sh`
- With exploration planner: `./system_real_robot_with_exploration_planner.sh`

### Bagfile Processing
- Base autonomy: `./system_bagfile.sh`
- With route planner: `./system_bagfile_with_route_planner.sh`
- With exploration planner: `./system_bagfile_with_exploration_planner.sh`

## Code Architecture

### Package Structure
- **src/base_autonomy/**: Core navigation modules
  - `local_planner`: Main control and collision avoidance
  - `terrain_analysis` & `terrain_analysis_ext`: Terrain processing
  - `sensor_scan_generation`: Sensor data preprocessing
  - `vehicle_simulator`: Unity integration and simulation launch
  - `visualization_tools`: RVIZ visualization utilities

- **src/slam/**: SLAM implementation
  - `arise_slam_mid360`: Main SLAM node for Mid-360 lidar
  - `arise_slam_mid360_msgs`: Custom message definitions
  - `dependency/`: External libraries (ceres-solver, gtsam, Sophus)

- **src/exploration_planner/**: TARE planner for autonomous exploration
  - `tare_planner`: Main exploration planning module

- **src/route_planner/**: FAR planner for goal navigation
  - `far_planner`: Global route planning module

- **src/utilities/**: Support packages
  - `livox_ros_driver2`: Mid-360 lidar driver
  - `teleop_*`: Teleoperation and control interfaces
  - `domain_bridge`: Network communication for base station
  - ROS plugins for RVIZ buttons and controls

### Key Configuration Files

**Local Planner** (`src/base_autonomy/local_planner/launch/local_planner.launch`):
- `maxSpeed`: Maximum vehicle speed in all modes
- `autonomySpeed`: Speed in waypoint mode
- `obstacleHeightThre`: Obstacle detection threshold
- `config`: "omniDir" for Mecanum wheels, "standard" for regular wheels
- Serial device path for motor controller

**SLAM** (`src/slam/arise_slam_mid360/config/livox_mid360.yaml`):
- `local_mode`: Enable localization mode with saved map
- `blindFront/Back/Left/Right`: Exclude regions from lidar FOV
- `init_x/y/z/yaw`: Initial pose for localization

**Mid-360 Lidar** (`src/utilities/livox_ros_driver2/config/MID360_config.json`):
- IP configuration (192.168.1.1xx where xx = last 2 digits of serial)

### Message Flow

1. **Sensor Input**: 
   - Lidar scan → `/lidar/scan` (custom format)
   - IMU data → `/imu/data`

2. **SLAM Output**:
   - Vehicle pose → `/state_estimation`
   - Registered scan → `/registered_scan`

3. **Navigation**:
   - Waypoints → `/way_point`
   - Joystick commands → `/joy_cmd`
   - Navigation boundary → `/navigation_boundary`

4. **Control Output**:
   - Vehicle commands → `/cmd_vel` (to motor controller via serial)

### Operating Modes

1. **Smart Joystick Mode** (default): Follows joystick with collision avoidance
2. **Waypoint Mode**: Autonomous navigation to waypoints
3. **Manual Mode**: Direct joystick control without collision avoidance

Mode switching via:
- RVIZ control panel and waypoint button
- PS3/4 or Xbox controller buttons
- ROS topic commands

## Testing

No formal test framework configured. System testing done through:
- Simulation with Unity environment models
- Bagfile replay for offline testing
- Manual teleoperation testing with `teleop_joy_controller`

## Development Notes

- Unity environment files must be placed in `src/base_autonomy/vehicle_simulator/mesh/unity/`
- Always source workspace before running: `source install/setup.bash`
- For ARM platforms, replace OR-Tools binaries in exploration planner
- System uses custom scan message format - must source workspace for bagfile operations
- X11 required for RVIZ on Ubuntu 24.04 (disable Wayland if needed)