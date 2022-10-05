#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/ros2_amr_ws/install/setup.bash
ros2 run my_py_pkg desktop_ui

# exit gracefully by returning a status 
exit 0