#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "roomba_msgs/msg/drive_command.hpp"
#include "roomba_msgs/msg/drive_mode.hpp"
#include "sensor_msgs/msg/joy.hpp"

// Gamepad teleoperation node for Roomba.
//
// Subscribes to /joy (sensor_msgs/Joy from ros-jazzy-joy joy_node) and
// publishes drive commands to /roomba/cmd/manual and mode changes to
// /roomba/mode.  drive_mux selects which source reaches /roomba/drive_command.
//
// Default mapping (Switch Pro Controller via hid-generic):
//   axes[1]    : left stick vertical    (forward/backward, +1 = up)
//   axes[2]    : right stick horizontal (turn, sign inverted internally)
//   buttons[0] : B  → all-stop while held
//   buttons[8] : -  → quit
//   buttons[9] : +  → toggle MANUAL / WALL_FOLLOW
//
// Watchdog: if /joy is silent for watchdog_ms, target speeds are zeroed
// (Bluetooth disconnect failsafe).
namespace roomba_ros2 {

class JoyTeleopNode : public rclcpp::Node {
 public:
  ~JoyTeleopNode() override = default;
  JoyTeleopNode(const JoyTeleopNode&) = delete;
  JoyTeleopNode& operator=(const JoyTeleopNode&) = delete;
  JoyTeleopNode(JoyTeleopNode&&) = delete;
  JoyTeleopNode& operator=(JoyTeleopNode&&) = delete;

  JoyTeleopNode() : Node("joy_teleop_node") {
    axis_forward_ = static_cast<std::size_t>(
        static_cast<int32_t>(declare_parameter("axis_forward", 1)));
    axis_turn_ = static_cast<std::size_t>(
        static_cast<int32_t>(declare_parameter("axis_turn", 2)));
    invert_turn_ = declare_parameter("invert_turn", true);
    button_stop_ = static_cast<std::size_t>(
        static_cast<int32_t>(declare_parameter("button_stop", 0)));
    button_mode_toggle_ = static_cast<std::size_t>(
        static_cast<int32_t>(declare_parameter("button_mode_toggle", 9)));
    button_quit_ = static_cast<std::size_t>(
        static_cast<int32_t>(declare_parameter("button_quit", 8)));
    forward_speed_mm_s_ = static_cast<int16_t>(
        static_cast<int32_t>(declare_parameter("forward_speed_mm_s", 200)));
    turn_speed_mm_s_ = static_cast<int16_t>(
        static_cast<int32_t>(declare_parameter("turn_speed_mm_s", 150)));
    const int32_t publish_rate_hz{
        static_cast<int32_t>(declare_parameter("publish_rate_hz", 20))};
    watchdog_ms_ =
        static_cast<double>(static_cast<int32_t>(declare_parameter("watchdog_ms", 500)));

    drive_pub_ = create_publisher<roomba_msgs::msg::DriveCommand>("/roomba/cmd/manual", 10);
    mode_pub_ = create_publisher<roomba_msgs::msg::DriveMode>("/roomba/mode", 10);

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        [this](sensor_msgs::msg::Joy::UniquePtr msg) { OnJoy(std::move(msg)); });

    last_joy_ = now();

    const auto period{std::chrono::milliseconds(1000 / std::max(publish_rate_hz, int32_t{1}))};
    timer_ = create_wall_timer(period, [this]() { Tick(); });

    RCLCPP_INFO(get_logger(), "JoyTeleopNode started (initial mode: MANUAL)");
  }

 private:
  void OnJoy(sensor_msgs::msg::Joy::UniquePtr msg) {
    last_joy_ = now();

    const bool mode_pressed{button_mode_toggle_ < msg->buttons.size() &&
                            msg->buttons[button_mode_toggle_] != 0};
    const bool quit_pressed{button_quit_ < msg->buttons.size() &&
                            msg->buttons[button_quit_] != 0};
    const bool stop_pressed{button_stop_ < msg->buttons.size() &&
                            msg->buttons[button_stop_] != 0};

    if (mode_pressed && !prev_mode_pressed_) {
      ToggleMode();
    }
    prev_mode_pressed_ = mode_pressed;

    if (quit_pressed && !prev_quit_pressed_) {
      RCLCPP_INFO(get_logger(), "Quit pressed");
      PublishDrive(0, 0);
      rclcpp::shutdown();
    }
    prev_quit_pressed_ = quit_pressed;

    if (stop_pressed) {
      target_left_ = 0;
      target_right_ = 0;
      return;
    }

    const float fwd{axis_forward_ < msg->axes.size() ? msg->axes[axis_forward_] : 0.0F};
    float turn{axis_turn_ < msg->axes.size() ? msg->axes[axis_turn_] : 0.0F};
    if (invert_turn_) {
      turn = -turn;
    }

    // Arcade drive: forward translation + differential turn.
    const float left_f{fwd * static_cast<float>(forward_speed_mm_s_) +
                       turn * static_cast<float>(turn_speed_mm_s_)};
    const float right_f{fwd * static_cast<float>(forward_speed_mm_s_) -
                        turn * static_cast<float>(turn_speed_mm_s_)};

    target_left_ = static_cast<int16_t>(std::clamp(left_f, -kMaxSpeed, kMaxSpeed));
    target_right_ = static_cast<int16_t>(std::clamp(right_f, -kMaxSpeed, kMaxSpeed));
  }

  void ToggleMode() {
    if (mode_ == roomba_msgs::msg::DriveMode::MANUAL) {
      mode_ = roomba_msgs::msg::DriveMode::WALL_FOLLOW;
    } else {
      mode_ = roomba_msgs::msg::DriveMode::MANUAL;
      target_left_ = 0;
      target_right_ = 0;
    }
    PublishMode(mode_);
    RCLCPP_INFO(get_logger(), "Mode: %s",
                mode_ == roomba_msgs::msg::DriveMode::MANUAL ? "MANUAL" : "WALL_FOLLOW");
  }

  void Tick() {
    if (mode_ != roomba_msgs::msg::DriveMode::MANUAL) {
      return;  // planner drives via drive_mux in autonomous modes
    }
    const double elapsed_ms{static_cast<double>((now() - last_joy_).nanoseconds()) / 1e6};
    if (elapsed_ms > watchdog_ms_) {
      target_left_ = 0;
      target_right_ = 0;
    }
    PublishDrive(target_left_, target_right_);
  }

  void PublishDrive(int16_t left, int16_t right) {
    roomba_msgs::msg::DriveCommand msg;
    msg.left_mm_s = left;
    msg.right_mm_s = right;
    drive_pub_->publish(msg);
  }

  void PublishMode(uint8_t mode) {
    roomba_msgs::msg::DriveMode msg;
    msg.mode = mode;
    mode_pub_->publish(msg);
  }

  static constexpr float kMaxSpeed{500.0F};

  std::size_t axis_forward_{1};
  std::size_t axis_turn_{2};
  bool invert_turn_{true};
  std::size_t button_stop_{0};
  std::size_t button_mode_toggle_{9};
  std::size_t button_quit_{8};
  int16_t forward_speed_mm_s_{200};
  int16_t turn_speed_mm_s_{150};
  double watchdog_ms_{500.0};

  uint8_t mode_{roomba_msgs::msg::DriveMode::MANUAL};
  int16_t target_left_{0};
  int16_t target_right_{0};
  bool prev_mode_pressed_{false};
  bool prev_quit_pressed_{false};
  rclcpp::Time last_joy_;

  rclcpp::Publisher<roomba_msgs::msg::DriveCommand>::SharedPtr drive_pub_;
  rclcpp::Publisher<roomba_msgs::msg::DriveMode>::SharedPtr mode_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace roomba_ros2

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<roomba_ros2::JoyTeleopNode>());
  rclcpp::shutdown();
  return 0;
}
