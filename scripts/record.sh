#!/bin/bash
# Record sensor data and camera footage to an MCAP bag file.
# Usage: ./scripts/record.sh [output_name]
#   output_name: bag directory name under ~/bags/ (default: session_YYYYMMDD_HHMMSS)
NAME=${1:-session_$(date +%Y%m%d_%H%M%S)}
OUTPUT=~/bags/${NAME}
echo "Recording to: ${OUTPUT}"
echo "Press Ctrl+C to stop."
ros2 bag record -s mcap -o "${OUTPUT}" \
  /image_raw/compressed \
  /roomba/sensors \
  /roomba/drive_command \
  /tof/distance_mm \
  /planner/state
