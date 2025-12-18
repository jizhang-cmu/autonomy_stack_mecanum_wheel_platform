# Autonomy Stack API Documentation

## Quick Start

### Environment Variables

The autonomy stack uses environment variables for easy configuration:

| Variable | Purpose | Example |
|----------|---------|---------|
| `ROBOT_CONFIG_PATH` | Robot-specific configuration | `mechanum_drive`, `unitree/unitree_g1` |
| `MAP_PATH` | Map **file prefix** for localization | `/home/user/maps/warehouse/map` |

**Setting Robot Configuration:**
```bash
export ROBOT_CONFIG_PATH="mechanum_drive"  # or "unitree/unitree_g1" or "unitree/unitree_b1"
```

**Setting Map (for Localization Mode):**
```bash
export MAP_PATH="/home/user/maps/warehouse/map"  # Enables localization mode
```
When `MAP_PATH` is set, the system automatically:

- Enables localization mode for SLAM (uses pre-built map)
- Loads `$MAP_PATH.pcd` (or legacy `.txt`) for SLAM
- Loads `$MAP_PATH_tomogram.pickle` for the PCT route planner

If `MAP_PATH` is not set, the system runs in SLAM/mapping mode and the PCT planner runs in SLAM mode.

**Typical Workflow:**
```bash
# For mapping a new environment
export ROBOT_CONFIG_PATH="mechanum_drive"
./system_real_robot_with_route_planner.sh

# For navigating in a known environment
export ROBOT_CONFIG_PATH="mechanum_drive"
export MAP_PATH="/home/user/maps/warehouse/map"
./system_real_robot_with_route_planner.sh
```

### Map Storage (SLAM + PCT)

Recommended map layout:

```text
/home/user/maps/warehouse/
├── map.pcd
└── map_tomogram.pickle
```

Set:

```bash
export MAP_PATH="/home/user/maps/warehouse/map"
```

To generate the tomogram from a PCD:

```bash
source install/setup.bash
ros2 run pct_planner pcd_to_tomogram.py /home/user/maps/warehouse/map.pcd -o /home/user/maps/warehouse/map_tomogram.pickle
```

## ROS Topics

### Input Topics (Commands)

| Topic | Type | Description |
|-------|------|-------------|
| `/way_point` | `geometry_msgs/PointStamped` | Send navigation goal (position only) |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Send goal with orientation |
| `/cancel_goal` | `std_msgs/Bool` | Cancel current goal (data: true) |
| `/joy` | `sensor_msgs/Joy` | Joystick input |
| `/navigation_boundary` | `geometry_msgs/PolygonStamped` | Set navigation boundaries |
| `/added_obstacles` | `sensor_msgs/PointCloud2` | Virtual obstacles |
| `/speed` | `std_msgs/Float32` | Desired speed input used by the base autonomy controller (m/s) |
| `/stop` | `std_msgs/Int8` | Safety stop input: `>=1` forces stop; `>=2` also forces yaw-rate stop |
| `/clicked_point` | `geometry_msgs/PointStamped` | RViz “Publish Point” input (used by PCT tools/visualizer) |

### Internal Control Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/slow_down` | `std_msgs/Int8` | Slowdown level used by the base autonomy controller (0–3) |

### Output Topics (Status)

| Topic | Type | Description |
|-------|------|-------------|
| `/state_estimation` | `nav_msgs/Odometry` | Robot pose from SLAM |
| `/registered_scan` | `sensor_msgs/PointCloud2` | Aligned lidar point cloud |
| `/terrain_map` | `sensor_msgs/PointCloud2` | Local terrain map |
| `/terrain_map_ext` | `sensor_msgs/PointCloud2` | Extended terrain map |
| `/path` | `nav_msgs/Path` | Local path being followed |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | Velocity commands to motors |
| `/goal_reached` | `std_msgs/Bool` | True when goal reached, false when cancelled/new goal |
| `/global_path` | `nav_msgs/Path` | Global path from route planner (PCT planner) |
| `/tomogram` | `sensor_msgs/PointCloud2` | Tomogram visualization (PCT planner) |
| `/tomogram_debug_grid` | `nav_msgs/OccupancyGrid` | Debug occupancy grid (PCT planner) |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/build_tomogram` | `std_srvs/Trigger` | Build tomogram from `/explored_areas` (PCT planner in SLAM mode) |

### Map Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/overall_map` | `sensor_msgs/PointCloud2` | Global map (only in sim)|
| `/registered_scan` | `sensor_msgs/PointCloud2` | Current scan in map frame |
| `/terrain_map` | `sensor_msgs/PointCloud2` | Local obstacle map |

## Usage Examples

### Send Goal
```bash
ros2 topic pub /way_point geometry_msgs/msg/PointStamped "{
  header: {frame_id: 'map'},
  point: {x: 5.0, y: 3.0, z: 0.0}
}" --once
```

### Cancel Goal
```bash
ros2 topic pub /cancel_goal std_msgs/msg/Bool "data: true" --once
```

### Stop the Robot (Safety Stop)

```bash
# 1 = stop translation, 2 = also stop rotation
ros2 topic pub /stop std_msgs/msg/Int8 "data: 2" --once
```

### Monitor Robot State
```bash
ros2 topic echo /state_estimation
```

## Route Planner Notes (FAR vs PCT)

The stack supports two global route planners:

- **FAR planner** (default)
- **PCT planner** (optional)

When using the PCT planner:

- **Service**: `/build_tomogram` (`std_srvs/Trigger`) builds a tomogram in SLAM mode from `/explored_areas`.
- **Output**: `/global_path` (`nav_msgs/Path`) planned global path.
- **Debug**: `/tomogram`, `/tomogram_debug_grid`.

See `src/route_planner/PCT_planner/README.md` for details and dependencies.

## SLAM / Localization Notes

### Localization Mode (MAP_PATH)

Set `MAP_PATH` to a file prefix to enable localization:

```bash
export MAP_PATH="/home/user/maps/warehouse/map"
```

This causes:

- SLAM to load: `/home/user/maps/warehouse/map.pcd` (or legacy `.txt`)
- PCT planner to load: `/home/user/maps/warehouse/map_tomogram.pickle`

### Launch Argument Overrides

SLAM (explicit map path):

```bash
ros2 launch arise_slam_mid360 arize_slam.launch.py \
  local_mode:=true \
  relocalization_map_path:=/home/user/maps/warehouse/map.pcd \
  init_x:=0.0 init_y:=0.0 init_yaw:=0.0
```

PCT planner (explicit tomogram path):

```bash
ros2 launch pct_planner pct_planner.launch.py \
  local_mode:=true \
  tomogram_path:=/home/user/maps/warehouse/map_tomogram.pickle
```

### RViz Re-localization

1. Launch with a map loaded (MAP_PATH or explicit launch args)
2. In RViz, click **2D Pose Estimate**
3. Click/drag to set the initial pose
4. SLAM will reset and re-localize