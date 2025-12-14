# Bazel x C++ x ROS2 trial
Use the following example very helpful implementation of ROS2 Rules.

https://github.com/mvukov/rules_ros2/tree/main/examples

## Changes
Copied the examples from there and change some minor points

- Add clang-tidy
- Add other modules for import check
- Remove Rust related dependencies
- Change `bazel_dep` from `local_path_override` to `git_override`

Those are based on https://github.com/tkhm/bazel-cpp-template/tree/main

Confirmed build and run on Ubuntu 24 + ROS2 Jazzy.

note: for running clang-tidy, installing them with `sudo apt install clang-tidy`
