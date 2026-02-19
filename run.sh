#!/bin/bash

# Ensure this script is run from the workspace root (e.g., ~/ros2_ws)
# Ensure the workspace is built using colcon
# Make executable with: chmod +x run.sh
# Usage: ./run.sh

# Source ROS2 and local workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Set the turtlebot model
export TURTLEBOT3_MODEL=burger

# The `pkill` command finds any process with 'gz'
# in its name (like 'gz sim') and terminates it, ensuring a clean start.
echo "--- Cleaning up any lingering Gazebo processes... ---"
pkill -f gz
sleep 1 # Give a moment for processes to terminate

# Launch the project
ros2 launch robot_navigation robot_navigation.launch.py