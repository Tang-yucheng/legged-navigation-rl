#!/bin/bash
echo "Setup unitree ros2 environment with default interface"
source /opt/ros/humble/setup.bash
source /home/tang-yucheng/Desktop/legged_nav_rl/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
