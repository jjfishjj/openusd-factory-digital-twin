#!/usr/bin/env bash
# Check the twin's ROS 2 topics from outside the simulator, using the system
# ros2 CLI - a different Python and a different rclpy, talking over DDS.
# Run while ros2_bridge.py is streaming.
set -o pipefail
source /opt/ros/jazzy/setup.bash

echo "=== topic list ==="
ros2 topic list

echo; echo "=== /factory/joint_states type + one message ==="
ros2 topic type /factory/joint_states
timeout 15 ros2 topic echo /factory/joint_states --once --truncate-length 400

echo; echo "=== /factory/amr_route type + vertex count ==="
ros2 topic type /factory/amr_route
timeout 15 ros2 topic echo /factory/amr_route --once --truncate-length 200 | head -30

echo; echo "=== publish rate ==="
timeout 12 ros2 topic hz /factory/joint_states --window 30 2>&1 | head -4
