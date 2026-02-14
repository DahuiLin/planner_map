#!/bin/bash
set -e

# Source ROS2 setup
source /opt/ros/humble/setup.bash

# Check if workspace needs to be built
if [ ! -d /workspace/ros2_ws/install ]; then
    echo "Install directory not found. Building workspace..."
    cd /workspace/ros2_ws
    colcon build --symlink-install
    echo "Workspace built successfully."
fi

# Source workspace if it exists
if [ -f /workspace/ros2_ws/install/setup.bash ]; then
    source /workspace/ros2_ws/install/setup.bash
fi

# Execute the command passed to the script
exec "$@"
