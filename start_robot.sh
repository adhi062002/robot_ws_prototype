#!/bin/bash

echo "🚀 Starting Robot System..."

# Source ROS
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash

# Optional: print topics before start
echo "📡 Checking ROS environment..."
ros2 doctor

# Launch robot
echo "🧠 Launching bringup..."
ros2 launch ugv_bringup bringup.launch.py
