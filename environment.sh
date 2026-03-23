#!/bin/bash

# Always source the original ROS 2 Humble environment first
source /opt/ros/humble/setup.bash
source /root/openarm/ros2_ws/install/setup.bash
echo "Sourced /opt/ros/humble/setup.bash and /root/openarm/ros2_ws/install/setup.bash"

# Check if an argument was provided for the domain ID
if [ -z "$1" ]; then
    export ROS_DOMAIN_ID=0
else
    export ROS_DOMAIN_ID=$1
fi

echo "ROS_DOMAIN_ID set to: $ROS_DOMAIN_ID"
