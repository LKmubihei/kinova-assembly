#!/bin/bash

echo "Killing ROS2 processes..."

pkill -f move_group
pkill -f rviz2
pkill -f ros2_control_node
pkill -f robot_state_publisher
pkill -f controller_manager
pkill -f spawner
pkill -f ros2

echo "Done."