#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "roomba_msgs/msg/drive_command.hpp"
#include "roomba_msgs/msg/drive_mode.hpp"

// Drive multiplexer node.
//
// Arbitrates between drive command sources based on the current mode and
// publishes the selected command to /roomba/drive_command.
//
// Topics:
//   Sub  /roomba/cmd/keyboard  (DriveCommand) — manual keyboard commands
//   Sub  /roomba/cmd/planner   (DriveCommand) — autonomous planner commands
//   Sub  /roomba/mode          (DriveMode)    — mode selection from keyboard_node
//   Pub  /roomba/drive_command (DriveCommand) — forwarded to roomba_node
//
// Mode behaviour:
//   MANUAL (autonomous=false): forwards /roomba/cmd/keyboard
//   AUTO   (autonomous=true) : forwards /roomba/cmd/planner
//
// On transition AUTO → MANUAL: publishes a zero command immediately so the
// robot stops before the operator takes control.
class DriveMuxNode : public rclcpp::Node {
 public:
  ~DriveMuxNode() override = default;
  DriveMuxNode(const DriveMuxNode&) = delete;
  DriveMuxNode& operator=(const DriveMuxNode&) = delete;
  DriveMuxNode(DriveMuxNode&&) = delete;
  DriveMuxNode& operator=(DriveMuxNode&&) = delete;

  DriveMuxNode() : Node("drive_mux") {
    drive_pub_ =
        create_publisher<roomba_msgs::msg::DriveCommand>("/roomba/drive_command", 10);

    keyboard_sub_ = create_subscription<roomba_msgs::msg::DriveCommand>(
        "/roomba/cmd/keyboard", 10,
        [this](roomba_msgs::msg::DriveCommand::UniquePtr msg) { OnKeyboard(std::move(msg)); });

    planner_sub_ = create_subscription<roomba_msgs::msg::DriveCommand>(
        "/roomba/cmd/planner", 10,
        [this](roomba_msgs::msg::DriveCommand::UniquePtr msg) { OnPlanner(std::move(msg)); });

    mode_sub_ = create_subscription<roomba_msgs::msg::DriveMode>(
        "/roomba/mode", 10,
        [this](roomba_msgs::msg::DriveMode::UniquePtr msg) { OnMode(std::move(msg)); });

    RCLCPP_INFO(get_logger(), "DriveMuxNode started (initial mode: MANUAL)");
  }

 private:
  void OnKeyboard(roomba_msgs::msg::DriveCommand::UniquePtr msg) {
    if (mode_ == roomba_msgs::msg::DriveMode::MANUAL) {
      drive_pub_->publish(*msg);
    }
  }

  void OnPlanner(roomba_msgs::msg::DriveCommand::UniquePtr msg) {
    if (mode_ == roomba_msgs::msg::DriveMode::WALL_FOLLOW) {
      drive_pub_->publish(*msg);
    }
  }

  void OnMode(roomba_msgs::msg::DriveMode::UniquePtr msg) {
    uint8_t prev_mode{mode_};
    mode_ = msg->mode;

    if (prev_mode != roomba_msgs::msg::DriveMode::MANUAL &&
        mode_ == roomba_msgs::msg::DriveMode::MANUAL) {
      // Autonomous → MANUAL: publish zero command so the robot stops
      // before the operator takes over.
      roomba_msgs::msg::DriveCommand stop;
      stop.left_mm_s = 0;
      stop.right_mm_s = 0;
      drive_pub_->publish(stop);
    }

    RCLCPP_INFO(get_logger(), "Mode: %s", ModeName(mode_));
  }

  static const char* ModeName(uint8_t mode) {
    switch (mode) {
      case roomba_msgs::msg::DriveMode::MANUAL:
        return "MANUAL";
      case roomba_msgs::msg::DriveMode::WALL_FOLLOW:
        return "WALL_FOLLOW";
      case roomba_msgs::msg::DriveMode::FOLLOW_ROOMBA:
        return "FOLLOW_ROOMBA";
      default:
        return "UNKNOWN";
    }
  }

  uint8_t mode_{roomba_msgs::msg::DriveMode::MANUAL};

  rclcpp::Publisher<roomba_msgs::msg::DriveCommand>::SharedPtr drive_pub_;
  rclcpp::Subscription<roomba_msgs::msg::DriveCommand>::SharedPtr keyboard_sub_;
  rclcpp::Subscription<roomba_msgs::msg::DriveCommand>::SharedPtr planner_sub_;
  rclcpp::Subscription<roomba_msgs::msg::DriveMode>::SharedPtr mode_sub_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriveMuxNode>());
  rclcpp::shutdown();
  return 0;
}
