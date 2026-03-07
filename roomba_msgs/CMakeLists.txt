# For colcon build only — used to make custom messages visible to ros2 CLI tools
# (e.g. ros2 topic echo). Bazel is the primary build system.
cmake_minimum_required(VERSION 3.8)
project(roomba_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DriveCommand.msg"
  "msg/RoombaSensors.msg"
)

ament_package()
