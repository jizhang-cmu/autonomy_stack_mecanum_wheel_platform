# DIMOS Setup Guide

## Prerequisites

- Ubuntu 24.04
- [ROS 2 Jazzy Installation](https://docs.ros.org/en/jazzy/Installation.html)

Add the following line to your `~/.bashrc` to source the ROS 2 Jazzy setup script automatically:

``` echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc```

## MID360 Ethernet Configuration

### Step 1: Configure Network Interface

1. Open Network Settings in Ubuntu
2. Find your Ethernet connection to the MID360
3. Click the gear icon to edit settings
4. Go to IPv4 tab
5. Change Method from "Automatic (DHCP)" to "Manual"
6. Add the following settings:
   - **Address**: 192.168.1.5
   - **Netmask**: 255.255.255.0
   - **Gateway**: 192.168.1.1
7. Click "Apply"

### Step 2: Configure MID360 IP in JSON

1. Find your MID360 serial number (on sticker under QR code)
2. Note the last 2 digits (e.g., if serial ends in 89, use 189)
3. Edit the configuration file:

```bash
cd ~/autonomy_stack_mecanum_wheel_platform
nano src/utilities/livox_ros_driver2/config/MID360_config.json
```

4. Update line 28 with your IP (192.168.1.1xx where xx = last 2 digits):

```json
"ip" : "192.168.1.1xx",
```

5. Save and exit

### Step 3: Verify Connection

```bash
ping 192.168.1.1xx  # Replace xx with your last 2 digits
```

## Robot Configuration

### Setting Robot Type

The system supports different robot configurations. Set the `ROBOT_CONFIG_PATH` environment variable to specify which robot configuration to use:

```bash
# For Unitree G1 (default if not set)
export ROBOT_CONFIG_PATH="unitree/unitree_g1"

# Add to ~/.bashrc to make permanent
echo 'export ROBOT_CONFIG_PATH="unitree/unitree_g1"' >> ~/.bashrc
```

Available robot configurations:
- `unitree/unitree_g1` - Unitree G1 robot (default)
- Add your custom robot configs in `src/base_autonomy/local_planner/config/`

## System Launch

### Simulation Mode

```bash
cd ~/autonomy_stack_mecanum_wheel_platform

# Base autonomy only
./system_simulation.sh

# With route planner
./system_simulation_with_route_planner.sh

# With exploration planner
./system_simulation_with_exploration_planner.sh
```

### Real Robot Mode

```bash
cd ~/autonomy_stack_mecanum_wheel_platform

# Base autonomy only
./system_real_robot.sh

# With route planner
./system_real_robot_with_route_planner.sh

# With exploration planner
./system_real_robot_with_exploration_planner.sh
```

## Quick Troubleshooting

- **Cannot ping MID360**: Check Ethernet cable and network settings
- **SLAM drift**: Press clear-terrain-map button on joystick controller
- **Joystick not recognized**: Unplug and replug USB dongle
