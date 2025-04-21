#!/bin/bash
echo "Setup unitree ros2 simulation environment"
source /opt/ros/humble/setup.bash
source /home/tang-yucheng/Desktop/legged_nav_rl/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="lo" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
