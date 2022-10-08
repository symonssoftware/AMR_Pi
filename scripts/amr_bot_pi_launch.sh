#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/ros2_amr_ws/install/setup.bash
/home/ubuntu/Phoenix-Linux-SocketCAN-Example/canableStart.sh
ros2 launch my_robot_bringup amr_bot_launch.py

# exit gracefully by returning a status 
exit 0