#include <chrono>
#include <memory>

#include "libroomba/include/wall_follower.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roomba_msgs/msg/drive_command.hpp"
#include "roomba_msgs/msg/roomba_sensors.hpp"
#include "std_msgs/msg/u_int16.hpp"

namespace roomba_ros2 {

// PlannerNode — wall-following autonomous mode.
//
// Subscribes to /roomba/sensors and publishes /roomba/drive_command using
// the WallFollower state machine.  Run alongside roomba_node:
//
//   Terminal 1: bazel run //launch:roomba_bringup
//   Terminal 2: bazel run //planner_node:planner_node
class PlannerNode : public rclcpp::Node {
 public:
  ~PlannerNode() override = default;
  PlannerNode(const PlannerNode&) = delete;
  PlannerNode& operator=(const PlannerNode&) = delete;
  PlannerNode(PlannerNode&&) = delete;
  PlannerNode& operator=(PlannerNode&&) = delete;

  PlannerNode() : Node("planner_node") {
    // Declare parameters (mirror WallFollowerConfig fields)
    WallFollowerConfig cfg;
    cfg.base_speed_mm_s =
        static_cast<int16_t>(declare_parameter("base_speed_mm_s", cfg.base_speed_mm_s));
    cfg.target_wall_signal =
        static_cast<uint16_t>(declare_parameter("target_wall_signal", cfg.target_wall_signal));
    cfg.kp = static_cast<float>(declare_parameter("kp", static_cast<double>(cfg.kp)));
    cfg.kd = static_cast<float>(declare_parameter("kd", static_cast<double>(cfg.kd)));
    cfg.search_turn_bias_mm_s =
        static_cast<int16_t>(declare_parameter("search_turn_bias_mm_s", cfg.search_turn_bias_mm_s));
    cfg.wall_detect_threshold =
        static_cast<uint16_t>(declare_parameter("wall_detect_threshold", cfg.wall_detect_threshold));
    cfg.max_correction_mm_s =
        static_cast<int16_t>(declare_parameter("max_correction_mm_s", cfg.max_correction_mm_s));
    cfg.cliff_debounce_count = static_cast<int32_t>(
        declare_parameter("cliff_debounce_count", cfg.cliff_debounce_count));
    cfg.wall_too_close_threshold = static_cast<uint16_t>(
        declare_parameter("wall_too_close_threshold", cfg.wall_too_close_threshold));
    cfg.emergency_turn_speed_mm_s = static_cast<int16_t>(
        declare_parameter("emergency_turn_speed_mm_s", cfg.emergency_turn_speed_mm_s));
    cfg.recovery_backup_ms = static_cast<int32_t>(
        declare_parameter("recovery_backup_ms", cfg.recovery_backup_ms));
    cfg.recovery_backup_speed_mm_s = static_cast<int16_t>(
        declare_parameter("recovery_backup_speed_mm_s", cfg.recovery_backup_speed_mm_s));
    cfg.recovery_turn_ms = static_cast<int32_t>(
        declare_parameter("recovery_turn_ms", cfg.recovery_turn_ms));
    cfg.recovery_turn_speed_mm_s = static_cast<int16_t>(
        declare_parameter("recovery_turn_speed_mm_s", cfg.recovery_turn_speed_mm_s));

    update_rate_ms_ = static_cast<int32_t>(declare_parameter("update_rate_ms", 50));  // 20 Hz

    // ToF sensor integration.
    // When use_tof=true, /tof/distance_mm overrides the Roomba IR wall_signal.
    // Conversion: wall_signal = max(0, tof_max_range_mm - distance_mm)
    // so that closer distance → higher signal, matching P-controller expectations.
    use_tof_ = declare_parameter("use_tof", false);
    tof_max_range_mm_ =
        static_cast<uint16_t>(declare_parameter("tof_max_range_mm", 400));

    wall_follower_ = std::make_unique<WallFollower>(cfg);

    sensor_sub_ = create_subscription<roomba_msgs::msg::RoombaSensors>(
        "/roomba/sensors", 10,
        [this](roomba_msgs::msg::RoombaSensors::UniquePtr msg) { OnSensors(std::move(msg)); });

    if (use_tof_) {
      tof_sub_ = create_subscription<std_msgs::msg::UInt16>(
          "/tof/distance_mm", 10,
          [this](std_msgs::msg::UInt16::UniquePtr msg) { OnTof(std::move(msg)); });
      RCLCPP_INFO(get_logger(), "PlannerNode: ToF mode (max_range=%d mm)", tof_max_range_mm_);
    }

    drive_pub_ = create_publisher<roomba_msgs::msg::DriveCommand>("/roomba/cmd/planner", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(update_rate_ms_),
                               [this]() { OnTimer(); });

    RCLCPP_INFO(get_logger(),
                "PlannerNode started: base=%d mm/s, target_wall=%d, kp=%.2f",
                cfg.base_speed_mm_s, cfg.target_wall_signal, cfg.kp);
  }

 private:
  void OnSensors(roomba_msgs::msg::RoombaSensors::UniquePtr msg) {
    latest_sensors_.bump_left = msg->bump_left;
    latest_sensors_.bump_right = msg->bump_right;
    latest_sensors_.cliff_left = msg->cliff_left;
    latest_sensors_.cliff_front_left = msg->cliff_front_left;
    latest_sensors_.cliff_front_right = msg->cliff_front_right;
    latest_sensors_.cliff_right = msg->cliff_right;
    // When use_tof=true, wall_signal is updated by OnTof() instead.
    if (!use_tof_) {
      latest_sensors_.wall_signal = msg->wall_signal;
    }
    has_sensors_ = true;
  }

  void OnTof(std_msgs::msg::UInt16::UniquePtr msg) {
    const uint16_t dist_mm{msg->data};
    // Convert distance to wall_signal: closer → higher signal.
    // wall_signal = max(0, tof_max_range_mm - dist_mm)
    if (dist_mm == 0 || dist_mm >= tof_max_range_mm_) {
      latest_sensors_.wall_signal = 0;
    } else {
      latest_sensors_.wall_signal =
          static_cast<uint16_t>(tof_max_range_mm_ - dist_mm);
    }
  }

  void OnTimer() {
    if (!has_sensors_) {
      return;  // wait for first sensor reading
    }

    wall_follower_->Update(latest_sensors_, update_rate_ms_);
    auto speeds{wall_follower_->GetWheelSpeeds()};

    roomba_msgs::msg::DriveCommand cmd;
    cmd.left_mm_s = speeds.left_mm_s;
    cmd.right_mm_s = speeds.right_mm_s;
    drive_pub_->publish(cmd);

    RCLCPP_DEBUG(get_logger(), "state=%d wall=%d L=%d R=%d",
                 static_cast<int>(wall_follower_->GetState()),
                 latest_sensors_.wall_signal, speeds.left_mm_s, speeds.right_mm_s);
  }

  std::unique_ptr<WallFollower> wall_follower_;
  rclcpp::Subscription<roomba_msgs::msg::RoombaSensors>::SharedPtr sensor_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr tof_sub_;
  rclcpp::Publisher<roomba_msgs::msg::DriveCommand>::SharedPtr drive_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  WallFollowerSensors latest_sensors_{};
  bool has_sensors_{false};
  int32_t update_rate_ms_{50};
  bool use_tof_{false};
  uint16_t tof_max_range_mm_{400};
};

}  // namespace roomba_ros2

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<roomba_ros2::PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
