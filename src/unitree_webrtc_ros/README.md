# unitree_webrtc_ros

ROS 2 Jazzy package for controlling Unitree Go2 robot via WebRTC connection. Provides a simple interface for movement control using `SPORT_CMD["Move"]` and sport mode commands.

## Features

- **TwistStamped subscriber**: Control robot movement using `geometry_msgs/TwistStamped` messages
- **Sport mode services**: Execute predefined movements (standup, liedown, hello, stretch, recovery_stand)
- **Direct command forwarding**: Each cmd_vel message sends one SPORT_CMD["Move"] command
- **Clean implementation**: No camera/lidar/tf overhead, just movement control

## Prerequisites

- ROS 2 Jazzy
- Python 3.12
- Unitree Go2 robot

## Installation

### Step 1: Install unitree_webrtc_connect in a virtual environment

```bash
# Install system dependencies
sudo apt update
sudo apt install portaudio19-dev

# Create and activate virtual environment
python3 -m venv ~/unitree_venv
source ~/unitree_venv/bin/activate

# Install ROS 2 dependencies
pip install PyYAML

# Install unitree_webrtc_connect
cd ~
git clone https://github.com/VectorRobotics/unitree_webrtc_connect.git
cd unitree_webrtc_connect
pip install -e .
```

### Step 2: Build the ROS 2 package

```bash
# Activate venv (if not already active)
source ~/unitree_venv/bin/activate

# Set up workspace and build
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
cp -r /path/to/unitree_webrtc_ros .
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select unitree_webrtc_ros
```

**Note**: This package uses `#!/usr/bin/env python3` which automatically uses the venv's Python when activated.

## Usage

### Running the node

```bash
# 1. Activate venv
source ~/unitree_venv/bin/activate

# 2. Source ROS 2
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# 3. Launch
ros2 launch unitree_webrtc_ros unitree_control.launch.py
```

**Convenience alias** (add to `~/.bashrc`):
```bash
alias ros_unitree='source ~/unitree_venv/bin/activate && source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash'
```

Then just: `ros_unitree`

### Launch the node

Basic launch with default IP (192.168.8.181):
```bash
ros2 launch unitree_webrtc_ros unitree_control.launch.py
```

Launch with custom IP:
```bash
ros2 launch unitree_webrtc_ros unitree_control.launch.py robot_ip:=192.168.8.100
```

Launch with LocalAP connection (robot's own WiFi at 192.168.12.1):
```bash
ros2 launch unitree_webrtc_ros unitree_control.launch.py connection_method:=LocalAP
```

Launch with wireless controller mode:
```bash
ros2 launch unitree_webrtc_ros unitree_control.launch.py control_mode:=wireless_controller
```

### Control the robot

#### Using cmd_vel topic

Send velocity commands (note: this package uses TwistStamped):
```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"

```

#### Using services

Execute sport mode commands:
```bash
# Stand up
ros2 service call /standup std_srvs/srv/Trigger

# Lie down
ros2 service call /liedown std_srvs/srv/Trigger

# Wave hello
ros2 service call /hello std_srvs/srv/Trigger

# Stretch
ros2 service call /stretch std_srvs/srv/Trigger

# Recovery stand position
ros2 service call /recovery_stand std_srvs/srv/Trigger
```

## Configuration

Edit [config/unitree_params.yaml](config/unitree_params.yaml) to change default parameters:

```yaml
unitree_control:
  ros__parameters:
    robot_ip: "192.168.8.181"
    connection_method: "LocalSTA"  # Options: LocalAP, LocalSTA, Remote
    control_mode: "sport_cmd"  # Options: sport_cmd, wireless_controller
```

## Topics

### Subscribed Topics

- `/cmd_vel` (geometry_msgs/TwistStamped): Velocity commands for robot movement
  - `twist.linear.x`: Forward/backward velocity (m/s)
  - `twist.linear.y`: Left/right velocity (m/s)
  - `twist.angular.z`: Rotation velocity (rad/s)

## Services

- `/standup` (std_srvs/Trigger): Make robot stand up
- `/liedown` (std_srvs/Trigger): Make robot lie down
- `/hello` (std_srvs/Trigger): Make robot wave hello
- `/stretch` (std_srvs/Trigger): Make robot stretch
- `/recovery_stand` (std_srvs/Trigger): Reset to recovery stand position

## Parameters

- `robot_ip` (string, default: "192.168.8.181"): IP address of the robot
- `connection_method` (string, default: "LocalSTA"): Connection method (LocalAP/LocalSTA/Remote)
- `control_mode` (string, default: "sport_cmd"): Control mode for cmd_vel
  - `sport_cmd`: Uses SPORT_CMD["Move"] API (recommended, more reliable)
  - `wireless_controller`: Uses WIRELESS_CONTROLLER topic (mimics joystick control)

## Connection Methods

- **LocalSTA**: Connect to robot on your local network (default, requires robot IP)
- **LocalAP**: Connect directly to robot's WiFi access point (IP: 192.168.12.1)
- **Remote**: Connect via Unitree cloud service (requires authentication - not implemented in launch file)

## Implementation Details

- **Movement**: Supports two control modes via `control_mode` parameter:
  - `sport_cmd` (default): Uses `SPORT_CMD["Move"]` API - more reliable, better for precise control
  - `wireless_controller`: Uses `WIRELESS_CONTROLLER` topic - mimics physical joystick
- **Command forwarding**: Each TwistStamped message triggers one move command
- **Thread-safe async**: Background thread runs asyncio event loop for WebRTC connection
- **No auto-stop**: Robot continues last command until new one received (send zeros to stop)
- **Coordinate mapping**: Automatically handles ROS->Unitree coordinate transformations

## Troubleshooting

### ModuleNotFoundError: No module named 'unitree_webrtc_connect'

Make sure you activated the virtual environment:
```bash
source ~/unitree_venv/bin/activate
```

### Connection fails

1. **Check robot IP**: Verify with `ping <robot_ip>`
2. **Check robot is on**: LED should be lit
3. **Check WiFi connection**:
   - For LocalSTA: Robot and computer on same network
   - For LocalAP: Computer connected to robot's WiFi (Unitree_XXXXXX)
4. **Check firewall**: WebRTC needs UDP ports open

### Robot doesn't move

1. **Check robot mode**: Must be in sport/normal mode (not AI mode which is deprecated)
2. **Check messages**: `ros2 topic echo /cmd_vel`
3. **Check logs**: Look for errors in node output
4. **Try manual command**: Use `ros2 topic pub` to test

### Virtual environment issues

If you get `ModuleNotFoundError`, make sure venv is activated:
```bash
source ~/unitree_venv/bin/activate
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

Always activate venv before building and running.

## Example: Using with teleop

If you have a teleop node that publishes `TwistStamped`:
```bash
# Terminal 1: Launch unitree control (needs venv)
source ~/unitree_venv/bin/activate
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch unitree_webrtc_ros unitree_control.launch.py

# Terminal 2: Run your teleop (doesn't need venv unless it uses Python packages from venv)
source /opt/ros/jazzy/setup.bash
ros2 run <your_package> <teleop_node>
```

## License

Apache-2.0

## Notes

- This package does NOT auto-stop the robot. You must send zero velocities to stop.
- The robot uses its own coordinate frame: x=forward, y=left, z=yaw
- WebRTC connection requires good WiFi signal strength
- First connection may take up to 30 seconds
