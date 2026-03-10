#!/bin/bash
# Start USB camera node (640x480 @ 15fps).
# Must be run outside Bazel (separate terminal).
# Publishes /image_raw and /image_raw/compressed.
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video0 \
  -p image_size:=[640,480] \
  -p time_per_frame:=[1,15]
