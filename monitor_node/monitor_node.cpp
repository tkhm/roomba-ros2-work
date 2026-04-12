#include <chrono>
#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "roomba_msgs/msg/drive_command.hpp"
#include "roomba_msgs/msg/roomba_sensors.hpp"
#include "std_msgs/msg/u_int16.hpp"

namespace {

// ANSI escape codes
constexpr const char* kRed = "\033[31m";
constexpr const char* kYellow = "\033[33m";
constexpr const char* kBold = "\033[1m";
constexpr const char* kReset = "\033[0m";
constexpr const char* kClear = "\033[K";  // erase to end of line

// Bumper/cliff: red when triggered, normal otherwise.
const char* FlagColor(bool triggered) {
  return triggered ? kRed : kReset;
}

// Battery voltage color: red below 12 V, yellow below 14 V.
const char* VoltColor(uint16_t mv) {
  if (mv < 12000) return kRed;
  if (mv < 14000) return kYellow;
  return kReset;
}

// Action label from drive command.
const char* ActionLabel(int16_t left, int16_t right) {
  if (left == 0 && right == 0) return "stopped";
  if (left > 0 && right > 0) return "forward";
  if (left < 0 && right < 0) return "backward";
  if (left < 0 && right > 0) return "spin left";
  if (left > 0 && right < 0) return "spin right";
  return "moving";
}

}  // namespace

// ===== MonitorNode =====
// Subscribes to /roomba/sensors and /roomba/drive_command and renders
// a live terminal dashboard using ANSI escape codes.
//
// Run in a dedicated terminal to avoid mixing output with other nodes.
class MonitorNode : public rclcpp::Node {
 public:
  MonitorNode(const MonitorNode&) = delete;
  MonitorNode& operator=(const MonitorNode&) = delete;
  MonitorNode(MonitorNode&&) = delete;
  MonitorNode& operator=(MonitorNode&&) = delete;

  MonitorNode() : Node("monitor_node") {
    sensors_sub_ = create_subscription<roomba_msgs::msg::RoombaSensors>(
        "/roomba/sensors", 10,
        [this](roomba_msgs::msg::RoombaSensors::UniquePtr msg) { sensors_ = *msg; });

    drive_sub_ = create_subscription<roomba_msgs::msg::DriveCommand>(
        "/roomba/drive_command", 10,
        [this](roomba_msgs::msg::DriveCommand::UniquePtr msg) { drive_ = *msg; });

    tof_sub_ = create_subscription<std_msgs::msg::UInt16>(
        "/tof/distance_mm", 10,
        [this](std_msgs::msg::UInt16::UniquePtr msg) { tof_distance_mm_ = msg->data; });

    int refresh_ms{static_cast<int>(declare_parameter("refresh_rate_ms", 500))};
    timer_ = create_wall_timer(std::chrono::milliseconds(refresh_ms), [this]() { Render(); });

    printf("\033[2J\033[H");
    fflush(stdout);
  }

 private:
  void Render() {
    // From the second render onwards, move cursor back to the top of the dashboard
    if (rendered_) {
      printf("\033[%dA\r", kLines);
    }
    rendered_ = true;

    const auto& s = sensors_;
    const auto& d = drive_;

    printf("%s----------------------------------------%s%s\n", kBold, kReset, kClear);
    printf("  %sRoomba monitor%s%s\n", kBold, kReset, kClear);
    printf("%s----------------------------------------%s%s\n", kBold, kReset, kClear);

    // Bumpers
    printf("  Bumpers :  left=[%s%s%s]  right=[%s%s%s]%s\n", FlagColor(s.bump_left),
           s.bump_left ? "BUMP" : " OK ", kReset, FlagColor(s.bump_right),
           s.bump_right ? "BUMP" : " OK ", kReset, kClear);

    // Cliff sensors
    printf("  Cliff   :  L=[%s%s%s]  FL=[%s%s%s]  FR=[%s%s%s]  R=[%s%s%s]%s\n",
           FlagColor(s.cliff_left), s.cliff_left ? "!!" : "OK", kReset,
           FlagColor(s.cliff_front_left), s.cliff_front_left ? "!!" : "OK", kReset,
           FlagColor(s.cliff_front_right), s.cliff_front_right ? "!!" : "OK", kReset,
           FlagColor(s.cliff_right), s.cliff_right ? "!!" : "OK", kReset, kClear);

    // Wheel drops
    printf("  WheelDrp:  left=[%s%s%s]  right=[%s%s%s]%s\n", FlagColor(s.wheel_drop_left),
           s.wheel_drop_left ? "DROP" : " OK ", kReset, FlagColor(s.wheel_drop_right),
           s.wheel_drop_right ? "DROP" : " OK ", kReset, kClear);

    printf("%s\n", kClear);

    // ToF distance
    printf("  ToF dist:  %4u mm%s\n", tof_distance_mm_, kClear);

    // Odometry
    printf("  Odometry:  dist=%+6d mm   angle=%+5d deg%s\n", s.distance_mm, s.angle_degrees,
           kClear);

    printf("%s\n", kClear);

    // Battery
    printf("  Battery :  %s%5u mV%s   %+6d mA%s\n", VoltColor(s.battery_voltage_mv),
           s.battery_voltage_mv, kReset, s.battery_current_ma, kClear);

    printf("%s\n", kClear);
    printf("%s----------------------------------------%s%s\n", kBold, kReset, kClear);

    // Drive command
    printf("  Drive   :  left=%+4d mm/s  right=%+4d mm/s%s\n", d.left_mm_s, d.right_mm_s, kClear);
    printf("  Action  :  %s%s\n", ActionLabel(d.left_mm_s, d.right_mm_s), kClear);

    printf("%s----------------------------------------%s%s\n", kBold, kReset, kClear);

    fflush(stdout);
  }

  // Number of lines printed by Render() — must match the \n count above.
  static constexpr int kLines = 15;

  bool rendered_{false};

  roomba_msgs::msg::RoombaSensors sensors_{};
  roomba_msgs::msg::DriveCommand drive_{};
  uint16_t tof_distance_mm_{0};

  rclcpp::Subscription<roomba_msgs::msg::RoombaSensors>::SharedPtr sensors_sub_;
  rclcpp::Subscription<roomba_msgs::msg::DriveCommand>::SharedPtr drive_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr tof_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MonitorNode>());
  rclcpp::shutdown();
  return 0;
}
