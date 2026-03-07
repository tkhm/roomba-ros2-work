#ifndef LIBROOMBA_INCLUDE_STUB_SERIAL_DRIVER_HPP_
#define LIBROOMBA_INCLUDE_STUB_SERIAL_DRIVER_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

#include "libroomba/include/serial_driver.hpp"

namespace roomba {

// Test stub for SerialDriver.
//
// Usage:
//   StubSerialDriver stub;
//   stub.Open("/dev/null", 115200);  // always succeeds
//
//   // Verify bytes sent to Roomba:
//   stub.Write(data, len);
//   auto sent = stub.GetWrittenBytes();  // inspect what was written
//
//   // Simulate Roomba sensor response:
//   stub.InjectReadData({0x03, 0x00, 0x64});  // inject bytes returned by Read()
class StubSerialDriver : public SerialDriver {
 public:
  // Always succeeds.
  bool Open(const char* /*port*/, uint32_t /*baud_rate*/) override {
    is_open_ = true;
    return true;
  }

  void Close() override { is_open_ = false; }

  // Records all written bytes. Always returns true.
  bool Write(const uint8_t* data, std::size_t len) override {
    for (std::size_t i = 0; i < len; ++i) {
      written_bytes_.push_back(data[i]);
    }
    return true;
  }

  // Returns bytes from the injected queue. Returns 0 if the queue is empty.
  std::size_t Read(uint8_t* buf, std::size_t len, uint32_t /*timeout_ms*/) override {
    std::size_t count{0};
    while (count < len && !read_queue_.empty()) {
      buf[count++] = read_queue_.front();
      read_queue_.pop_front();
    }
    return count;
  }

  // --- Test helpers ---

  // Returns all bytes written via Write() since the last call to ClearWrittenBytes().
  const std::vector<uint8_t>& GetWrittenBytes() const { return written_bytes_; }

  // Clears the written bytes log.
  void ClearWrittenBytes() { written_bytes_.clear(); }

  // Injects bytes to be returned by the next Read() call(s).
  void InjectReadData(const std::vector<uint8_t>& data) {
    for (uint8_t b : data) {
      read_queue_.push_back(b);
    }
  }

  bool IsOpen() const { return is_open_; }

 private:
  bool is_open_{false};
  std::vector<uint8_t> written_bytes_;
  std::deque<uint8_t> read_queue_;
};

}  // namespace roomba

#endif  // LIBROOMBA_INCLUDE_STUB_SERIAL_DRIVER_HPP_
