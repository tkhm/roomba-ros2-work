#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <memory>
#include <string>

#include "libroomba/include/stub_vl53l1x_driver.hpp"
#include "libroomba/include/vl53l1x_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"

namespace roomba_ros2 {

// ===== LinuxVl53l1xDriver =====
// Linux I2C driver for the VL53L1X Time-of-Flight distance sensor.
//
// Uses /dev/i2c-N via ioctl (no linux/i2c-dev.h required).
// The 91-byte default configuration block is from the public ST VL53L1X
// ULD API source (registers 0x002D–0x0087).
//
// Measurement sequence:
//   1. Init() — write default config, clear interrupt, start continuous ranging
//   2. ReadDistanceMm() — wait for data-ready flag, read 2-byte distance, clear interrupt
//   3. Close() — stop ranging, close fd
class LinuxVl53l1xDriver : public Vl53l1xDriver {
 public:
  explicit LinuxVl53l1xDriver(std::string device, uint8_t address = kDefaultAddress)
      : device_{std::move(device)}, address_{address} {}

  ~LinuxVl53l1xDriver() override { Close(); }

  LinuxVl53l1xDriver(const LinuxVl53l1xDriver&) = delete;
  LinuxVl53l1xDriver& operator=(const LinuxVl53l1xDriver&) = delete;
  LinuxVl53l1xDriver(LinuxVl53l1xDriver&&) = delete;
  LinuxVl53l1xDriver& operator=(LinuxVl53l1xDriver&&) = delete;

  bool Init() override {
    fd_ = open(device_.c_str(), O_RDWR);
    if (fd_ < 0) {
      return false;
    }
    // I2C_SLAVE = 0x0703 (hardcoded to avoid linux/i2c-dev.h dependency)
    if (ioctl(fd_, 0x0703, address_) < 0) {
      Close();
      return false;
    }
    return InitSensor();
  }

  int16_t ReadDistanceMm() override {
    if (fd_ < 0) {
      return -1;
    }
    // Non-blocking check: return -1 immediately if new data is not ready yet.
    // The caller retains the last valid reading.  This avoids blocking the
    // ROS2 executor thread, which would cause timer drift and I2C instability.
    uint8_t status{0};
    if (!ReadReg8(kRegGpioStatus, status)) {
      return -1;
    }
    if ((status & 0x01) == 0) {
      return -1;  // measurement not complete yet
    }
    uint16_t dist{0};
    if (!ReadReg16(kRegFinalRange, dist)) {
      return -1;
    }
    ClearInterrupt();
    return static_cast<int16_t>(dist);
  }

  void Close() override {
    if (fd_ >= 0) {
      WriteReg8(kRegModeStart, 0x00);  // stop ranging
      close(fd_);
      fd_ = -1;
    }
  }

 private:
  static constexpr uint8_t kDefaultAddress{0x29};
  static constexpr uint16_t kRegModelId{0x010F};
  static constexpr uint16_t kRegGpioStatus{0x0031};
  static constexpr uint16_t kRegInterruptClear{0x0086};
  static constexpr uint16_t kRegModeStart{0x0087};
  static constexpr uint16_t kRegFinalRange{0x0096};
  static constexpr uint16_t kRegConfigStart{0x002D};
  static constexpr uint8_t kExpectedModelId{0xEA};
  // Default configuration for VL53L1X, registers 0x002D–0x0087 (91 bytes).
  // Source: STMicroelectronics VL53L1X ULD API (public domain initialization block).
  // clang-format off
  static constexpr std::array<uint8_t, 91> kDefaultConfig{
      0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08,
      0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,
      0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x0A, 0x21,
      0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xC8,
      0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00,
      0x00, 0x01, 0xCC, 0x0F, 0x01, 0xF1, 0x0D, 0x01,
      0x68, 0x00, 0x80, 0x08, 0xB8, 0x00, 0x00, 0x00,
      0x00, 0x0F, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x01, 0x0F, 0x0D, 0x0E, 0x0E, 0x00,
      0x00, 0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00, 0x00,
      0x01, 0x00, 0x00,
  };
  // clang-format on

  bool InitSensor() {
    // Verify model ID (0xEA) to confirm the sensor is present and responsive.
    uint8_t model_id{0};
    if (!ReadReg8(kRegModelId, model_id)) {
      return false;
    }
    if (model_id != kExpectedModelId) {
      return false;
    }

    // Write the 91-byte default configuration block.
    // I2C packet: [addrHigh, addrLow, data[0..90]]
    std::array<uint8_t, 2 + kDefaultConfig.size()> buf{};
    buf[0] = static_cast<uint8_t>(kRegConfigStart >> 8);
    buf[1] = static_cast<uint8_t>(kRegConfigStart & 0xFF);
    for (std::size_t i{0}; i < kDefaultConfig.size(); ++i) {
      buf[2 + i] = kDefaultConfig[i];
    }
    if (write(fd_, buf.data(), buf.size()) != static_cast<ssize_t>(buf.size())) {
      return false;
    }

    // Clear any pending interrupt, then start continuous ranging (0x40).
    if (!ClearInterrupt()) {
      return false;
    }
    return WriteReg8(kRegModeStart, 0x40);
  }

  bool ClearInterrupt() { return WriteReg8(kRegInterruptClear, 0x01); }

  // Write a single byte to a 16-bit register address.
  bool WriteReg8(uint16_t reg, uint8_t value) {
    const std::array<uint8_t, 3> buf{
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xFF),
        value,
    };
    return write(fd_, buf.data(), buf.size()) == static_cast<ssize_t>(buf.size());
  }

  // Read a single byte from a 16-bit register address.
  bool ReadReg8(uint16_t reg, uint8_t& value) {
    const std::array<uint8_t, 2> addr{
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xFF),
    };
    if (write(fd_, addr.data(), addr.size()) != 2) {
      return false;
    }
    return read(fd_, &value, 1) == 1;
  }

  // Read two bytes (big-endian) from a 16-bit register address.
  bool ReadReg16(uint16_t reg, uint16_t& value) {
    const std::array<uint8_t, 2> addr{
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xFF),
    };
    if (write(fd_, addr.data(), addr.size()) != 2) {
      return false;
    }
    std::array<uint8_t, 2> data{};
    if (read(fd_, data.data(), 2) != 2) {
      return false;
    }
    value = static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
    return true;
  }

  std::string device_;
  uint8_t address_;
  int fd_{-1};
};

// ===== TofNode =====
// Reads the VL53L1X ToF sensor and publishes distance on /tof/distance_mm.
//
// Parameters:
//   i2c_device   (string, default "/dev/i2c-1") — Linux I2C device path
//   i2c_address  (int,    default 41 = 0x29)    — I2C slave address
//   use_stub     (bool,   default false)         — use StubVl53l1xDriver
//   poll_rate_ms (int,    default 50)            — sensor poll interval [ms]
//
// Topics:
//   Pub  /tof/distance_mm  (std_msgs/UInt16)  — distance in mm (0 = invalid)
class TofNode : public rclcpp::Node {
 public:
  TofNode(const TofNode&) = delete;
  TofNode& operator=(const TofNode&) = delete;
  TofNode(TofNode&&) = delete;
  TofNode& operator=(TofNode&&) = delete;

  TofNode() : Node("tof_node") {
    const std::string i2c_device{declare_parameter("i2c_device", std::string("/dev/i2c-1"))};
    const int i2c_address{static_cast<int>(declare_parameter("i2c_address", 0x29))};
    const bool use_stub{declare_parameter("use_stub", false)};
    const int poll_rate_ms{static_cast<int>(declare_parameter("poll_rate_ms", 50))};

    if (use_stub) {
      driver_ = std::make_unique<StubVl53l1xDriver>();
      RCLCPP_INFO(get_logger(), "TofNode: stub mode (distance=200 mm)");
    } else {
      driver_ = std::make_unique<LinuxVl53l1xDriver>(
          i2c_device, static_cast<uint8_t>(i2c_address));
      RCLCPP_INFO(get_logger(), "TofNode: real mode (%s, addr=0x%02X)",
                  i2c_device.c_str(), i2c_address);
    }

    if (!driver_->Init()) {
      RCLCPP_ERROR(get_logger(), "VL53L1X init failed — check wiring and I2C bus");
    }

    pub_ = create_publisher<std_msgs::msg::UInt16>("/tof/distance_mm", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(poll_rate_ms),
                               [this]() { OnTimer(); });
  }

  ~TofNode() override {
    if (driver_) {
      driver_->Close();
    }
  }

 private:
  void OnTimer() {
    int16_t dist_mm{driver_->ReadDistanceMm()};

    // Retain the last valid reading when the sensor has no new data yet.
    // Avoids publishing spurious 0s when the poll rate exceeds the sensor's
    // measurement rate (~10 Hz default for VL53L1X).
    if (dist_mm > 0) {
      last_valid_dist_mm_ = dist_mm;
    }

    std_msgs::msg::UInt16 msg;
    msg.data = static_cast<uint16_t>(last_valid_dist_mm_);
    pub_->publish(msg);

    RCLCPP_DEBUG(get_logger(), "ToF distance: %d mm", last_valid_dist_mm_);
  }

  std::unique_ptr<Vl53l1xDriver> driver_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int16_t last_valid_dist_mm_{0};
};

}  // namespace roomba_ros2

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<roomba_ros2::TofNode>());
  rclcpp::shutdown();
  return 0;
}
