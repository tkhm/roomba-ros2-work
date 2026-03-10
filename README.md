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
```

---

## Test

```bash
# Run all tests
bazel test //...

# Run unit tests only
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

For full session setup (Foxglove, camera, recording), see **[docs/runbook.md](docs/runbook.md)**.

### Quick start — stub mode (no hardware required)

```bash
# Terminal 1: roomba_node (stub) + monitor_node
bazel run //launch:roomba_bringup_stub

# Terminal 2: operation source
bazel run //keyboard_node:keyboard_node
```

### Quick start — real hardware

Requires Roomba connected via serial. See [Hardware Setup](#hardware-setup).

```bash
# Terminal 1: roomba_node (real) + monitor_node
bazel run //launch:roomba_bringup

# Terminal 2: operation source
bazel run //keyboard_node:keyboard_node
```

### Operation sources

The bringup launch (`roomba_bringup`) starts `roomba_node` and `monitor_node` only.
An operation source is started separately and can be swapped independently:

| Operation source | Command |
|---|---|
| Keyboard teleoperation | `bazel run //keyboard_node:keyboard_node` |
| Game controller (future) | `bazel run //joy_teleop_node:joy_teleop_node` |
| Autonomous (future) | `bazel run //planner_node:planner_node` |

### Keyboard controls

| Key | Action |
|-----|--------|
| `w` | Forward |
| `s` | Backward |
| `a` | Spin left |
| `d` | Spin right |
| `Space` | Stop |
| `q` | Quit |

Auto-stop: Roomba stops automatically if no key is pressed for 500 ms.

---

## Foxglove Visualization

[Foxglove Studio](https://foxglove.dev/) can be used to visualize sensor data and drive commands in real time.

### Setup

Install `foxglove_bridge` on Raspberry Pi 5:

```bash
sudo apt install ros-jazzy-foxglove-bridge
```

`foxglove_bridge` also requires `roomba_msgs` to be built with colcon (same as `ros2 topic echo`).
See [Using `ros2 topic echo` with custom messages](#using-ros2-topic-echo-with-custom-messages).

### Launch

`foxglove_bridge` must be started in a **separate terminal** from the Bazel launch.
Bazel overrides `AMENT_PREFIX_PATH` to its own generated setup, which does not include
the system ROS2 typesupport libraries that `foxglove_bridge` requires.
DDS topic discovery still works between Bazel-launched nodes and a standalone `foxglove_bridge` process.

```bash
./scripts/foxglove_bridge.sh
```

Then open [Foxglove Studio desktop app](https://foxglove.dev/download) and connect:

1. Click **Open connection**
2. Select **Foxglove WebSocket** (not "ROS 2" or "Rosbridge")
3. Enter URL: `ws://<raspberry-pi-ip>:8765`

> **Note:** The browser version of Foxglove Studio (foxglove.dev) cannot connect to `ws://`
> due to browser mixed content restrictions (HTTPS page → non-TLS WebSocket).
> Use the desktop app instead.

### Topics visualized

| Topic | Type | Display |
|---|---|---|
| `/roomba/sensors` | `roomba_msgs/RoombaSensors` | Bumper/cliff indicators, battery voltage graph |
| `/roomba/drive_command` | `roomba_msgs/DriveCommand` | Left/right wheel velocity time series |
| `/image_raw/compressed` | `sensor_msgs/CompressedImage` | Camera feed (requires USB camera) |

---

## Camera (USB)

### Setup

Install `v4l2_camera` and compressed image transport on Raspberry Pi 5:

```bash
sudo apt install ros-jazzy-v4l2-camera ros-jazzy-compressed-image-transport
```

Add yourself to the `video` group (required once, then re-login):

```bash
sudo usermod -aG video $USER
```

### Usage

Start the camera node in a separate terminal (outside Bazel, same as `foxglove_bridge`):

```bash
./scripts/camera.sh
```

Check the device path with `ls /dev/video*` if `/dev/video0` is not found.

This publishes `/image_raw` and `/image_raw/compressed` (`sensor_msgs/CompressedImage`).

**Why 640x480 @ 15fps:**
- Resolution: sufficient to identify the Roomba and surroundings at 0.5–2m range,
  both for post-session review and future camera-based Roomba tracking
- Frame rate: Roomba's typical operating speed (~150–200 mm/s) is slow enough that
  15fps captures motion without gaps; 30fps is overkill and doubles file size
- Processing: Raspberry Pi 5 can run image recognition on this stream in real time,
  leaving headroom for other nodes

### Recording

```bash
./scripts/record.sh              # auto-named: session_YYYYMMDD_HHMMSS
./scripts/record.sh my_session   # custom name → ~/bags/my_session/
```

Use `/image_raw/compressed` (JPEG) instead of `/image_raw` to keep file sizes manageable.
Open the saved `.mcap` file in Foxglove Studio on Ubuntu to replay camera footage and sensor data in sync.

---

## ROS2 Topic Layout

```
[operation source]           (keyboard_node / joy_teleop_node / planner_node)
  └─ /roomba/drive_command (DriveCommand) ──▶ [roomba_node] ──── UART ──── Roomba 600
                                                    │
                                                    └─ /roomba/sensors (RoombaSensors)
                                                              ├──▶ [monitor_node]
                                                              └──▶ [foxglove_bridge] ──▶ Foxglove Studio

[v4l2_camera_node]
  └─ /image_raw/compressed (CompressedImage) ──▶ [foxglove_bridge] ──▶ Foxglove Studio
```

---

## Directory Structure

```
roomba-ros2-work/
├── MODULE.bazel              # Bazel dependency definitions
├── BUILD.bazel               # Root build file (clang-format, config_setting)
├── .bazelrc / .bazelversion  # Bazel configuration
├── .clang-format / .clang-tidy
│
├── tools/
│   ├── apply_clang_format.sh # bazel run //:format
│   └── check_clang_format.sh # bazel test //:format_check
│
├── scripts/                  # Helper scripts for common operations
│   ├── foxglove_bridge.sh    # Start foxglove_bridge
│   ├── camera.sh             # Start USB camera node (640x480 @ 15fps)
│   └── record.sh             # Start MCAP bag recording
│
├── docs/
│   └── runbook.md            # Step-by-step operation guide
│
├── roomba_msgs/              # Custom ROS2 message definitions
│   └── msg/
│       ├── DriveCommand.msg  # left_mm_s, right_mm_s
│       └── RoombaSensors.msg # bumpers, cliff, odometry, battery
│
├── libroomba/                # Core library — ROS2-independent
│   ├── include/
│   │   ├── serial_driver.hpp     # SerialDriver abstract base (HAL)
│   │   ├── stub_serial_driver.hpp
│   │   └── roomba_oi.hpp         # OI command constants, builders, parsers
│   └── tests/
│       └── roomba_oi_test.cpp    # GoogleTest (command encoding, sensor parsing)
│
├── roomba_node/              # Serial communication node
├── keyboard_node/            # Keyboard teleoperation node
├── monitor_node/             # Terminal dashboard node
│
├── config/
│   └── roomba_params.yaml    # All node parameters
└── launch/
    ├── roomba_bringup.py       # roomba_node (real) + monitor_node
    └── roomba_bringup_stub.py  # roomba_node (stub) + monitor_node
```

---

## Hardware Setup

### Serial connection

Roomba 600 series uses a 7-pin Mini-DIN connector.

**Using a USB-to-serial (USB-FTDI) cable** (verified on Raspberry Pi 5):

Connect the cable between the Raspberry Pi USB port and the Roomba Mini-DIN connector.
The device appears as `/dev/ttyUSB0` (verify with `ls /dev/ttyUSB*`).

```bash
# Add your user to the dialout group to access the device without sudo
sudo usermod -a -G dialout $USER
# Log out and back in for the change to take effect
```

Update `config/roomba_params.yaml` with the correct device name:

```yaml
roomba_node:
  ros__parameters:
    serial_port: "/dev/ttyUSB0"
```

**Using a GPIO UART (direct wiring):**

Enable UART on Raspberry Pi 5 by editing `/boot/firmware/config.txt`:

```
enable_uart=1
```

Reboot and verify: `ls /dev/ttyAMA*`

The device name varies by Raspberry Pi model and OS configuration.

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
| Real hardware verification | **Required** |
