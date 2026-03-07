#ifndef LIBROOMBA_INCLUDE_SERIAL_DRIVER_HPP_
#define LIBROOMBA_INCLUDE_SERIAL_DRIVER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace roomba {

// Abstract serial port interface for Roomba OI communication.
// Real implementation: LinuxSerialDriver (in roomba_node.cpp)
// Test implementation: StubSerialDriver (stub_serial_driver.hpp)
class SerialDriver {
 public:
  SerialDriver() = default;
  virtual ~SerialDriver() = default;
  SerialDriver(const SerialDriver&) = delete;
  SerialDriver& operator=(const SerialDriver&) = delete;
  SerialDriver(SerialDriver&&) = delete;
  SerialDriver& operator=(SerialDriver&&) = delete;

  // Open the serial port. Returns true on success.
  // port: device path (e.g. "/dev/ttyS0")
  // baud_rate: must be 115200 for Roomba OI
  virtual bool Open(const char* port, uint32_t baud_rate) = 0;

  // Close the serial port.
  virtual void Close() = 0;

  // Write bytes to the serial port. Returns true if all bytes were written.
  virtual bool Write(const uint8_t* data, std::size_t len) = 0;

  // Read up to len bytes. Returns the number of bytes actually read.
  // Blocks for up to timeout_ms milliseconds.
  virtual std::size_t Read(uint8_t* buf, std::size_t len, uint32_t timeout_ms) = 0;
};

}  // namespace roomba

#endif  // LIBROOMBA_INCLUDE_SERIAL_DRIVER_HPP_
