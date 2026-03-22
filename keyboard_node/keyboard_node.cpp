#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "roomba_msgs/msg/drive_command.hpp"
#include "roomba_msgs/msg/drive_mode.hpp"

// Keyboard teleoperation node for Roomba.
//
// Publishes drive commands to /roomba/cmd/keyboard and mode changes to
// /roomba/mode.  drive_mux selects which source reaches /roomba/drive_command.
//
// IMPORTANT: stdin is not available via ros2 launch.
//   Run directly: bazel run //keyboard_node:keyboard_node
//
// Key bindings:
//   w / s        : forward / backward
//   a / d        : spin left / spin right
//   [space]      : stop
//   [Tab]        : toggle manual / autonomous mode
//   q            : quit
//
// Auto-stop: if no key is pressed for auto_stop_ms in MANUAL mode, Roomba stops.
class KeyboardNode : public rclcpp::Node {
 public:
  KeyboardNode(const KeyboardNode&) = delete;
  KeyboardNode& operator=(const KeyboardNode&) = delete;
  KeyboardNode(KeyboardNode&&) = delete;
  KeyboardNode& operator=(KeyboardNode&&) = delete;

  KeyboardNode() : Node("keyboard_node") {
    forward_speed_mm_s_ =
        static_cast<int16_t>(static_cast<int>(declare_parameter("forward_speed_mm_s", 150)));
    turn_speed_mm_s_ =
        static_cast<int16_t>(static_cast<int>(declare_parameter("turn_speed_mm_s", 100)));
    auto_stop_ms_ = static_cast<double>(static_cast<int>(declare_parameter("auto_stop_ms", 500)));

    drive_pub_ =
        create_publisher<roomba_msgs::msg::DriveCommand>("/roomba/cmd/keyboard", 10);
    mode_pub_ = create_publisher<roomba_msgs::msg::DriveMode>("/roomba/mode", 10);

    // Disable canonical mode and echo for raw key input
    tcgetattr(STDIN_FILENO, &old_tio_);
    struct termios new_tio{old_tio_};
    new_tio.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
    new_tio.c_cc[VMIN] = 0;   // return immediately even with 0 chars
    new_tio.c_cc[VTIME] = 0;  // no timeout
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

    last_input_ = now();

    // Poll key input at 20 Hz
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { Tick(); });
    PrintHelp();
  }

  ~KeyboardNode() noexcept override {
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
    try {
      PublishDrive(0, 0);
    } catch (...) {
    }  // NOLINT(bugprone-empty-catch): publish after shutdown may fail
    printf("\nKeyboard control stopped.\n");
    fflush(stdout);
  }

 private:
  // Non-blocking stdin poll. Returns true and fills *c if a key is available.
  bool PollKey(char* c) {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    struct timeval tv{0, 0};
    return select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) > 0 &&
           read(STDIN_FILENO, c, 1) == 1;
  }

  void Tick() {
    char c{0};
    if (PollKey(&c)) {
      if (c == '\033') {
        // Discard escape sequence bytes (arrow keys, etc.)
        std::array<char, 4> seq{};
        read(STDIN_FILENO, seq.data(), seq.size());
      } else {
        ProcessKey(c);
      }
    }

    if (mode_ == roomba_msgs::msg::DriveMode::MANUAL) {
      // Auto-stop if no key input for auto_stop_ms_ in MANUAL mode
      if (input_received_) {
        double elapsed_ms{static_cast<double>((now() - last_input_).nanoseconds()) / 1e6};
        if (elapsed_ms > auto_stop_ms_) {
          left_ = 0;
          right_ = 0;
        }
      }
      PublishDrive(left_, right_);
    }
    // In autonomous modes: do not publish drive commands; planner drives via drive_mux.

    PrintStatus();
  }

  void ProcessKey(char c) {
    if (c == '\t') {
      // Tab: toggle between MANUAL and WALL_FOLLOW
      if (mode_ == roomba_msgs::msg::DriveMode::MANUAL) {
        mode_ = roomba_msgs::msg::DriveMode::WALL_FOLLOW;
      } else {
        mode_ = roomba_msgs::msg::DriveMode::MANUAL;
        // Returning to MANUAL: reset speeds so robot doesn't lurch
        left_ = 0;
        right_ = 0;
        input_received_ = false;
      }
      PublishMode(mode_);
      return;
    }

    if (mode_ != roomba_msgs::msg::DriveMode::MANUAL) {
      return;  // ignore drive keys in autonomous modes
    }

    last_input_ = now();
    input_received_ = true;

    switch (c) {
      case 'w':
      case 'W':
        left_ = forward_speed_mm_s_;
        right_ = forward_speed_mm_s_;
        break;
      case 's':
      case 'S':
        left_ = static_cast<int16_t>(-forward_speed_mm_s_);
        right_ = static_cast<int16_t>(-forward_speed_mm_s_);
        break;
      case 'a':
      case 'A':
        left_ = static_cast<int16_t>(-turn_speed_mm_s_);
        right_ = turn_speed_mm_s_;
        break;
      case 'd':
      case 'D':
        left_ = turn_speed_mm_s_;
        right_ = static_cast<int16_t>(-turn_speed_mm_s_);
        break;
      case ' ':
        left_ = 0;
        right_ = 0;
        break;
      case 'q':
      case 'Q':
        PublishDrive(0, 0);
        rclcpp::shutdown();
        break;
      default:
        break;
    }
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

  void PrintHelp() {
    printf("\033[2J\033[H");
    printf("----------------------------------------\n");
    printf("  Roomba keyboard control\n");
    printf("  w: forward    s: backward\n");
    printf("  a: spin left  d: spin right\n");
    printf("  [space]: stop  [Tab]: toggle mode  q: quit\n");
    printf("----------------------------------------\n");
    printf("  [MANUAL]  left=   0 mm/s  right=   0 mm/s  [stopped]\n");
    fflush(stdout);
  }

  void PrintStatus() {
    printf("\033[1A\r\033[K");
    if (mode_ == roomba_msgs::msg::DriveMode::WALL_FOLLOW) {
      printf("  [WALL FOLLOW]  (planner active)\n");
    } else {
      printf("  [MANUAL]  left=%4d mm/s  right=%4d mm/s  [%s]\n", left_, right_,
             ActionLabel(left_, right_));
    }
    fflush(stdout);
  }

  static const char* ActionLabel(int16_t left, int16_t right) {
    if (left == 0 && right == 0) return "stopped";
    if (left > 0 && right > 0) return "forward";
    if (left < 0 && right < 0) return "backward";
    if (left < 0 && right > 0) return "spin left";
    if (left > 0 && right < 0) return "spin right";
    return "moving";
  }

  int16_t forward_speed_mm_s_{150};
  int16_t turn_speed_mm_s_{100};
  double auto_stop_ms_{500.0};

  uint8_t mode_{roomba_msgs::msg::DriveMode::MANUAL};
  int16_t left_{0};
  int16_t right_{0};
  rclcpp::Time last_input_;
  bool input_received_{false};

  struct termios old_tio_{};

  rclcpp::Publisher<roomba_msgs::msg::DriveCommand>::SharedPtr drive_pub_;
  rclcpp::Publisher<roomba_msgs::msg::DriveMode>::SharedPtr mode_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardNode>());
  rclcpp::shutdown();
  return 0;
}
