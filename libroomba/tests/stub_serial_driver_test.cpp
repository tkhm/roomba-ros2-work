#include "libroomba/include/stub_serial_driver.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>

namespace roomba::test {

class StubSerialDriverTest : public ::testing::Test {
 protected:
  void SetUp() override { stub_.Open("/dev/null", 115200); }

  StubSerialDriver stub_;  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes)
};

TEST_F(StubSerialDriverTest, OpenAlwaysSucceeds) {
  EXPECT_TRUE(stub_.IsOpen());
}

TEST_F(StubSerialDriverTest, WriteRecordsBytes) {
  const std::array<uint8_t, 3> data{0x80, 0x83, 0x91};
  stub_.Write(data.data(), data.size());

  const auto& written = stub_.GetWrittenBytes();
  ASSERT_EQ(written.size(), data.size());
  for (std::size_t i{0}; i < data.size(); ++i) {
    EXPECT_EQ(written[i], data[i]);
  }
}

TEST_F(StubSerialDriverTest, ReadFromInjectedData) {
  stub_.InjectReadData({0x03, 0x00, 0x64});

  std::array<uint8_t, 3> buf{};
  std::size_t n{stub_.Read(buf.data(), buf.size(), 10)};

  EXPECT_EQ(n, 3);
  EXPECT_EQ(buf[0], 0x03);
  EXPECT_EQ(buf[1], 0x00);
  EXPECT_EQ(buf[2], 0x64);
}

TEST_F(StubSerialDriverTest, ReadReturnsZeroWhenEmpty) {
  std::array<uint8_t, 4> buf{};
  std::size_t n{stub_.Read(buf.data(), buf.size(), 10)};
  EXPECT_EQ(n, 0);
}

TEST_F(StubSerialDriverTest, ReadConsumesQueueSequentially) {
  stub_.InjectReadData({0xAA, 0xBB});

  std::array<uint8_t, 1> buf{};
  stub_.Read(buf.data(), 1, 10);
  EXPECT_EQ(buf[0], 0xAA);

  stub_.Read(buf.data(), 1, 10);
  EXPECT_EQ(buf[0], 0xBB);

  std::size_t n = stub_.Read(buf.data(), 1, 10);
  EXPECT_EQ(n, 0);
}

TEST_F(StubSerialDriverTest, ClearWrittenBytes) {
  const uint8_t byte{0x80};
  stub_.Write(&byte, 1);
  EXPECT_EQ(stub_.GetWrittenBytes().size(), 1);

  stub_.ClearWrittenBytes();
  EXPECT_TRUE(stub_.GetWrittenBytes().empty());
}

}  // namespace roomba::test
