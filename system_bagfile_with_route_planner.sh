#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd $SCRIPT_DIR
source ./install/setup.bash
ros2 launch vehicle_simulator system_bagfile_with_route_planner.launch &
sleep 1
ros2 run rviz2 rviz2 -d src/route_planner/far_planner/rviz/default.rviz
