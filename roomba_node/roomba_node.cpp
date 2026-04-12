#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "libroomba/include/roomba_oi.hpp"
#include "libroomba/include/serial_driver.hpp"
#include "libroomba/include/stub_serial_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roomba_msgs/msg/drive_command.hpp"
#include "roomba_msgs/msg/roomba_sensors.hpp"

namespace roomba_ros2 {

// ===== LinuxSerialDriver =====
// POSIX serial port driver for Roomba OI (115200 baud, 8N1).
class LinuxSerialDriver : public SerialDriver {
 public:
  LinuxSerialDriver() = default;
  ~LinuxSerialDriver() override {
    if (fd_ >= 0) {
      close(fd_);
    }
  }
  LinuxSerialDriver(const LinuxSerialDriver&) = delete;
  LinuxSerialDriver& operator=(const LinuxSerialDriver&) = delete;
  LinuxSerialDriver(LinuxSerialDriver&&) = delete;
  LinuxSerialDriver& operator=(LinuxSerialDriver&&) = delete;

  bool Open(const char* port, uint32_t baud_rate) override {
    fd_ = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ < 0) {
      return false;
    }

    struct termios tty {};
    if (tcgetattr(fd_, &tty) != 0) {
      close(fd_);
      fd_ = -1;
      return false;
    }

    speed_t speed{B115200};
    if (baud_rate != 115200) {
      // Roomba OI requires 115200 baud; other rates are not supported.
      close(fd_);
      fd_ = -1;
      return false;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // 8N1: 8 data bits, no parity, 1 stop bit
    tty.c_cflag &= ~static_cast<tcflag_t>(PARENB);
    tty.c_cflag &= ~static_cast<tcflag_t>(CSTOPB);
    tty.c_cflag &= ~static_cast<tcflag_t>(CSIZE);
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~static_cast<tcflag_t>(CRTSCTS);
    tty.c_cflag |= CREAD | CLOCAL;

    // Raw mode: no echo, no canonical, no signal chars
    tty.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~static_cast<tcflag_t>(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~static_cast<tcflag_t>(ICRNL | INLCR);
    tty.c_oflag &= ~static_cast<tcflag_t>(OPOST);

    // Non-blocking read by default; select() handles per-call timeout
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      close(fd_);
      fd_ = -1;
      return false;
    }
    return true;
  }

  void Close() override {
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
  }

  bool Write(const uint8_t* data, std::size_t len) override {
    return write(fd_, data, len) == static_cast<ssize_t>(len);
  }

  // Read up to `len` bytes within `timeout_ms` milliseconds.
  std::size_t Read(uint8_t* buf, std::size_t len, uint32_t timeout_ms) override {
    std::size_t total{0};
    while (total < len) {
      fd_set read_fds;
      FD_ZERO(&read_fds);
      FD_SET(fd_, &read_fds);

      struct timeval tv {};
      // NOLINTNEXTLINE(google-runtime-int): tv_sec/tv_usec are POSIX long fields
      tv.tv_sec = static_cast<long>(timeout_ms / 1000);
      // NOLINTNEXTLINE(google-runtime-int): tv_sec/tv_usec are POSIX long fields
      tv.tv_usec = static_cast<long>((timeout_ms % 1000) * 1000);

      int ret{select(fd_ + 1, &read_fds, nullptr, nullptr, &tv)};
      if (ret <= 0) {
        break;  // timeout or error
      }
      // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic): POSIX read() requires raw pointer offset
      ssize_t n{read(fd_, buf + total, len - total)};
      if (n <= 0) {
        break;
      }
      total += static_cast<std::size_t>(n);
    }
    return total;
  }

 private:
  int fd_{-1};
};

}  // namespace roomba_ros2

// ===== Sensor query configuration =====
// QUERY_LIST requests these packets in order; response bytes follow the same order.
//   Packet  7 (BumpsDrops):  1 byte
//   Packet  9 (Cliff):       1 byte
//   Packet 19 (Distance):    2 bytes  (signed, mm)
//   Packet 20 (Angle):       2 bytes  (signed, degrees)
//   Packet 25 (Voltage):     2 bytes  (unsigned, mV)
//   Packet 26 (Current):     2 bytes  (signed, mA)
//   Packet 27 (WallSignal):  2 bytes  (unsigned, 0-4095)
namespace {
constexpr std::array<uint8_t, 7> kSensorPackets{
    roomba_ros2::oi::kPacketBumpsDrops,  roomba_ros2::oi::kPacketCliff,
    roomba_ros2::oi::kPacketDistance,    roomba_ros2::oi::kPacketAngle,
    roomba_ros2::oi::kPacketVoltage,     roomba_ros2::oi::kPacketCurrent,
    roomba_ros2::oi::kPacketWallSignal,
};
constexpr std::size_t kSensorResponseBytes{12};
}  // namespace

namespace roomba_ros2 {

// ===== RoombaNode =====
class RoombaNode : public rclcpp::Node {
 public:
  RoombaNode(const RoombaNode&) = delete;
  RoombaNode& operator=(const RoombaNode&) = delete;
  RoombaNode(RoombaNode&&) = delete;
  RoombaNode& operator=(RoombaNode&&) = delete;

  RoombaNode() : Node("roomba_node") {
    // Declare parameters
    bool use_stub{declare_parameter("use_stub", false)};
    std::string serial_port{declare_parameter("serial_port", std::string("/dev/ttyS0"))};
    int32_t max_speed_raw{static_cast<int32_t>(declare_parameter("max_speed_mm_s", 200))};
    max_speed_mm_s_ = static_cast<int16_t>(std::clamp(max_speed_raw, 0, 500));
    std::string oi_mode{declare_parameter("oi_mode", std::string("safe"))};

    // Create serial driver (stub or real)
    if (use_stub) {
      serial_driver_ = std::make_unique<StubSerialDriver>();
      RCLCPP_INFO(get_logger(), "RoombaNode: stub mode (no serial I/O)");
    } else {
      auto driver{std::make_unique<LinuxSerialDriver>()};
      if (!driver->Open(serial_port.c_str(), 115200)) {
        RCLCPP_ERROR(get_logger(), "RoombaNode: failed to open serial port %s",
                     serial_port.c_str());
        throw std::runtime_error("Serial port open failed");
      }
      serial_driver_ = std::move(driver);
      RCLCPP_INFO(get_logger(), "RoombaNode: real mode (%s)", serial_port.c_str());
    }

    // Initialize Roomba OI
    SendByte(oi::kStart);
    if (oi_mode == "full") {
      SendByte(oi::kFull);
      RCLCPP_INFO(get_logger(), "RoombaNode: OI mode = Full");
    } else {
      SendByte(oi::kSafe);
      RCLCPP_INFO(get_logger(), "RoombaNode: OI mode = Safe");
    }

    // Subscribe to drive commands
    drive_sub_ = create_subscription<roomba_msgs::msg::DriveCommand>(
        "/roomba/drive_command", 10,
        [this](roomba_msgs::msg::DriveCommand::UniquePtr msg) { OnDriveCommand(std::move(msg)); });

    // Publish sensor data
    sensor_pub_ = create_publisher<roomba_msgs::msg::RoombaSensors>("/roomba/sensors", 10);

    // Poll sensors at 20 Hz
    sensor_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { OnSensorTimer(); });

    RCLCPP_INFO(get_logger(), "RoombaNode started (max_speed=%d mm/s)", max_speed_mm_s_);
  }

  ~RoombaNode() noexcept override {
    // Safety stop before shutdown
    auto stop_cmd{oi::BuildDriveDirectCmd(0, 0)};
    serial_driver_->Write(stop_cmd.data(), stop_cmd.size());
    SendByte(oi::kStop);
    RCLCPP_INFO(get_logger(), "RoombaNode: safety stop on shutdown");
  }

 private:
  void SendByte(uint8_t byte) { serial_driver_->Write(&byte, 1); }

  void OnDriveCommand(roomba_msgs::msg::DriveCommand::UniquePtr msg) {
    int16_t left{
        std::clamp(msg->left_mm_s, static_cast<int16_t>(-max_speed_mm_s_), max_speed_mm_s_)};
    int16_t right{
        std::clamp(msg->right_mm_s, static_cast<int16_t>(-max_speed_mm_s_), max_speed_mm_s_)};
    auto cmd{oi::BuildDriveDirectCmd(left, right)};
    serial_driver_->Write(cmd.data(), cmd.size());
  }

  void OnSensorTimer() {
    // Request sensor data
    auto query{oi::BuildQueryListCmd(kSensorPackets)};
    serial_driver_->Write(query.data(), query.size());

    // Read response within 50 ms (one timer period)
    std::array<uint8_t, kSensorResponseBytes> buf{};
    std::size_t n{serial_driver_->Read(buf.data(), buf.size(), 50)};

    if (n < kSensorResponseBytes) {
      // Insufficient data — normal in stub mode; skip publishing
      return;
    }

    // Parse response
    auto bumps{oi::ParseBumpsDrops(buf[0])};
    auto cliff{oi::ParseCliff(buf[1])};

    roomba_msgs::msg::RoombaSensors msg;
    msg.bump_left = bumps.bump_left;
    msg.bump_right = bumps.bump_right;
    msg.wheel_drop_left = bumps.wheel_drop_left;
    msg.wheel_drop_right = bumps.wheel_drop_right;
    msg.cliff_left = cliff.cliff_left;
    msg.cliff_front_left = cliff.cliff_front_left;
    msg.cliff_front_right = cliff.cliff_front_right;
    msg.cliff_right = cliff.cliff_right;
    msg.distance_mm = oi::ParseInt16(buf[2], buf[3]);
    msg.angle_degrees = oi::ParseInt16(buf[4], buf[5]);
    msg.battery_voltage_mv = oi::ParseUint16(buf[6], buf[7]);
    msg.battery_current_ma = oi::ParseInt16(buf[8], buf[9]);
    msg.wall_signal = oi::ParseUint16(buf[10], buf[11]);

    sensor_pub_->publish(msg);
  }

  std::unique_ptr<SerialDriver> serial_driver_;
  rclcpp::Subscription<roomba_msgs::msg::DriveCommand>::SharedPtr drive_sub_;
  rclcpp::Publisher<roomba_msgs::msg::RoombaSensors>::SharedPtr sensor_pub_;
  rclcpp::TimerBase::SharedPtr sensor_timer_;
  int16_t max_speed_mm_s_{200};
};

}  // namespace roomba_ros2

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<roomba_ros2::RoombaNode>());
  rclcpp::shutdown();
  return 0;
}
