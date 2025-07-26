#!/bin/bash

export ROS_DOMAIN_ID=1

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd $SCRIPT_DIR
ros2 run image_transport republish --ros-args -p in_transport:=compressed -p out_transport:=raw --remap in/compressed:=/camera/image/compressed --remap out:=/camera/image/transmitted &
sleep 1
ros2 run rviz2 rviz2 -d ./base_station.rviz
