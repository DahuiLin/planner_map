#!/bin/bash
set -e

# Source ROS2 setup
source /opt/ros/humble/setup.bash

# Source workspace if it exists
if [ -f /workspace/ros2_ws/install/setup.bash ]; then
    source /workspace/ros2_ws/install/setup.bash
fi

# Execute the command passed to the script
exec "$@"
