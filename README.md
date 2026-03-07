# roomba-ros2-work

ROS2 / C++ / Bazel-based control system for iRobot Roomba 600 series,
running on Raspberry Pi 5 via serial (UART) using the
[Roomba Open Interface (OI)](https://edu.irobot.com/learning-library/roomba-open-interface) protocol.

## System Overview

| Item | Details |
|---|---|
| Language | C++17 (nodes) / Python 3.12 (launch, tests) |
| Build system | Bazel 8.4.1 (bzlmod) |
| ROS2 | Jazzy ([rules_ros2](https://github.com/mvukov/rules_ros2)) |
| Hardware | Raspberry Pi 5 |
| Robot | iRobot Roomba 692 / 643 |
| Communication | UART serial (Roomba Open Interface, 115200 baud) |

## Prerequisites

- Ubuntu 24.04 + ROS2 Jazzy
- [Bazel 8.4.1](https://bazel.build/)
- `clang-format` and `clang-tidy` (LLVM 18)

```bash
sudo apt install clang-format clang-tidy
```

> For real hardware setup on Raspberry Pi 5, see [Hardware Setup](#hardware-setup).

---

## Build

```bash
# Build all targets
bazel build //...

# Build with static analysis (clang-tidy)
bazel build //... --config clang-tidy

# Build for real hardware (enables serial driver compilation)
bazel build //... --define=real_serial=true
```

---

## Test

```bash
# Run all tests
bazel test //...

# Run by test level
bazel test //libroomba/tests/...   # Level 1: unit tests (GoogleTest, no hardware)
```

### Test levels

| Level | Target | Framework | Hardware |
|---|---|---|---|
| Level 1 | libroomba (OI commands, serial driver) | GoogleTest | Not required |
| Level 2 | Node topic I/O | launch_testing | Not required |
| Level 3 | Multi-node pipeline | launch_testing | Not required |
| Level 4 | Real hardware | Manual | **Required** |

> Levels 2 and 3 will be added in later phases.

---

## Code Style

C++ source code follows [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html).

| Tool | Config |
|---|---|
| clang-format | [`.clang-format`](.clang-format) (BasedOnStyle: Google, ColumnLimit: 100) |
| clang-tidy | [`.clang-tidy`](.clang-tidy) |

File extensions: `.cpp` (implementation), `.hpp` (headers)

```bash
# Apply clang-format to all source files
bazel run //:format

# Check for clang-format violations (also included in bazel test //...)
bazel test //:format_check
```

---

## Running Nodes

> This section will be updated as nodes are implemented (Phase 4 onward).

### Stub mode (no hardware required)

<!-- Phase 4: roomba_node stub mode -->
<!-- Phase 5: keyboard control stub mode -->
<!-- Phase 7: Foxglove visualization -->

### Real hardware mode

<!-- Phase 8: real hardware operation -->

---

## Foxglove Visualization

> This section will be updated in Phase 7.

[Foxglove Studio](https://foxglove.dev/) can be used to visualize sensor data and drive commands in real time.
Install `foxglove_bridge` on Raspberry Pi 5:

```bash
sudo apt install ros-jazzy-foxglove-bridge
```

Topics visualized in Foxglove:

| Topic | Type | Display |
|---|---|---|
| `/roomba/sensors` | `roomba_msgs/RoombaSensors` | Bumper/cliff indicators, battery graph |
| `/roomba/drive_command` | `roomba_msgs/DriveCommand` | Wheel velocity time series |

Camera images (`/image_raw`) can be added to the same Foxglove session in a future phase,
enabling synchronized playback of camera footage and sensor data via `.mcap` recording.

---

## ROS2 Topic Layout

```
[keyboard_node]
  └─ /roomba/drive_command (DriveCommand) ──▶ [roomba_node] ──── UART ──── Roomba 600
                                                    │
                                                    └─ /roomba/sensors (RoombaSensors)
                                                              ├──▶ [monitor_node]
                                                              └──▶ [foxglove_bridge] ──▶ Foxglove Studio
```

---

## Directory Structure

```
roomba-ros2-work/
├── MODULE.bazel              # Bazel dependency definitions
├── BUILD.bazel               # Root build file (clang-format, config_setting)
├── .bazelrc / .bazelversion  # Bazel configuration
├── .clang-format / .clang-tidy
├── tools/
│   ├── apply_clang_format.sh # bazel run //:format
│   └── check_clang_format.sh # bazel test //:format_check
│
├── roomba_msgs/              # Custom ROS2 message definitions
│   └── msg/
│       ├── DriveCommand.msg  # left_mm_s, right_mm_s
│       └── RoombaSensors.msg # bumpers, cliff, odometry, battery
│
├── libroomba/                # Core library — ROS2-independent (planned)
│   ├── include/
│   │   ├── serial_driver.hpp     # SerialDriver abstract base (HAL)
│   │   ├── stub_serial_driver.hpp
│   │   └── roomba_oi.hpp         # OI command constants and helpers
│   └── tests/
│
├── roomba_node/              # Serial communication node (planned)
├── keyboard_node/            # Keyboard teleoperation node (planned)
├── monitor_node/             # Terminal dashboard node (planned)
│
├── config/
│   └── roomba_params.yaml    # All node parameters (planned)
└── launch/
    ├── roomba_keyboard.py    # keyboard_node + roomba_node (planned)
    ├── roomba_monitor.py     # + monitor_node (planned)
    └── roomba_foxglove.py    # + foxglove_bridge (planned)
```

---

## Hardware Setup

> This section will be updated in Phase 8 after real hardware verification.

### Serial connection

Roomba 600 series uses a 7-pin Mini-DIN connector.
Connect Raspberry Pi 5 UART pins to the Roomba serial port via the cable.

Expected device: `/dev/ttyS0` or `/dev/ttyAMA0` (verify with `ls /dev/tty*`)

### Enable UART on Raspberry Pi 5

Edit `/boot/firmware/config.txt`:

```
enable_uart=1
```

Reboot and verify the device is available:

```bash
ls /dev/ttyAMA*
```

### Using `ros2 topic echo` with custom messages

`ros2 topic echo` requires the message package to be built with colcon:

```bash
mkdir -p ~/ros2_ws/src
cp -r roomba_msgs ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select roomba_msgs
source ~/ros2_ws/install/setup.bash  # add to ~/.bashrc
```

---

## Roomba / Hardware Connectivity

Work is separated into two categories:

| Category | Roomba connection |
|---|---|
| Build, unit tests (GoogleTest), format check | **Not required** |
| Node operation with `use_stub:=true` | **Not required** |
| Real hardware verification (Phase 8+) | **Required** |
