#!/bin/bash
# Start ROS2 joy_node (apt package: ros-jazzy-joy).
# Must be run outside Bazel (separate terminal).
# Publishes /joy (sensor_msgs/Joy) for joy_teleop_node to consume.
ros2 run joy joy_node
