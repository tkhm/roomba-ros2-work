#!/bin/bash
# Start foxglove_bridge for Foxglove Studio visualization.
# Must be run outside Bazel (separate terminal).
# Connect Foxglove Studio to: ws://<raspberry-pi-ip>:8765
ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 -p address:=0.0.0.0
