#include "libroomba/include/roomba_oi.hpp"

#include <gtest/gtest.h>

#include <array>

namespace roomba_ros2::oi::test {

// ---------------------------------------------------------------------------
// BuildDriveDirectCmd
// ---------------------------------------------------------------------------

TEST(BuildDriveDirectCmdTest, ForwardStraight) {
  // left=200, right=200 → both wheels forward at 200 mm/s
  auto cmd = BuildDriveDirectCmd(200, 200);
  EXPECT_EQ(cmd[0], kDriveDirect);
  // right=200 (0x00C8): high=0x00, low=0xC8
  EXPECT_EQ(cmd[1], 0x00);
  EXPECT_EQ(cmd[2], 0xC8);
  // left=200 (0x00C8): high=0x00, low=0xC8
  EXPECT_EQ(cmd[3], 0x00);
  EXPECT_EQ(cmd[4], 0xC8);
}

TEST(BuildDriveDirectCmdTest, Stop) {
  auto cmd = BuildDriveDirectCmd(0, 0);
  EXPECT_EQ(cmd[0], kDriveDirect);
  EXPECT_EQ(cmd[1], 0x00);
  EXPECT_EQ(cmd[2], 0x00);
  EXPECT_EQ(cmd[3], 0x00);
  EXPECT_EQ(cmd[4], 0x00);
}

TEST(BuildDriveDirectCmdTest, Reverse) {
  // left=-100, right=-100 → 0xFF9C in two's complement
  auto cmd = BuildDriveDirectCmd(-100, -100);
  EXPECT_EQ(cmd[0], kDriveDirect);
  EXPECT_EQ(cmd[1], 0xFF);
  EXPECT_EQ(cmd[2], 0x9C);
  EXPECT_EQ(cmd[3], 0xFF);
  EXPECT_EQ(cmd[4], 0x9C);
}

TEST(BuildDriveDirectCmdTest, TurnLeft) {
  // left=-100, right=100 → spin left in place
  auto cmd = BuildDriveDirectCmd(-100, 100);
  EXPECT_EQ(cmd[0], kDriveDirect);
  // right=100 (0x0064)
  EXPECT_EQ(cmd[1], 0x00);
  EXPECT_EQ(cmd[2], 0x64);
  // left=-100 (0xFF9C)
  EXPECT_EQ(cmd[3], 0xFF);
  EXPECT_EQ(cmd[4], 0x9C);
}

TEST(BuildDriveDirectCmdTest, ClampToMaxSpeed) {
  // 600 is out of range; must be clamped to 500 (0x01F4)
  auto cmd = BuildDriveDirectCmd(600, -600);
  // right=-500 (0xFE0C)
  EXPECT_EQ(cmd[1], 0xFE);
  EXPECT_EQ(cmd[2], 0x0C);
  // left=500 (0x01F4)
  EXPECT_EQ(cmd[3], 0x01);
  EXPECT_EQ(cmd[4], 0xF4);
}

// ---------------------------------------------------------------------------
// BuildSensorsCmd
// ---------------------------------------------------------------------------

TEST(BuildSensorsCmdTest, BumpsDropsPacket) {
  auto cmd = BuildSensorsCmd(kPacketBumpsDrops);
  EXPECT_EQ(cmd[0], kSensors);
  EXPECT_EQ(cmd[1], kPacketBumpsDrops);
}

TEST(BuildSensorsCmdTest, CliffPacket) {
  auto cmd = BuildSensorsCmd(kPacketCliff);
  EXPECT_EQ(cmd[0], kSensors);
  EXPECT_EQ(cmd[1], kPacketCliff);
}

// ---------------------------------------------------------------------------
// BuildQueryListCmd
// ---------------------------------------------------------------------------

TEST(BuildQueryListCmdTest, TwoPackets) {
  auto cmd = BuildQueryListCmd(std::array<uint8_t, 2>{kPacketBumpsDrops, kPacketCliff});
  EXPECT_EQ(cmd[0], kQueryList);
  EXPECT_EQ(cmd[1], 2);  // packet count
  EXPECT_EQ(cmd[2], kPacketBumpsDrops);
  EXPECT_EQ(cmd[3], kPacketCliff);
}

// ---------------------------------------------------------------------------
// ParseBumpsDrops
// ---------------------------------------------------------------------------

TEST(ParseBumpsDropsTest, NoBumps) {
  auto result = ParseBumpsDrops(0x00);
  EXPECT_FALSE(result.bump_right);
  EXPECT_FALSE(result.bump_left);
  EXPECT_FALSE(result.wheel_drop_right);
  EXPECT_FALSE(result.wheel_drop_left);
}

TEST(ParseBumpsDropsTest, BumpRight) {
  auto result = ParseBumpsDrops(0x01);
  EXPECT_TRUE(result.bump_right);
  EXPECT_FALSE(result.bump_left);
}

TEST(ParseBumpsDropsTest, BumpLeft) {
  auto result = ParseBumpsDrops(0x02);
  EXPECT_FALSE(result.bump_right);
  EXPECT_TRUE(result.bump_left);
}

TEST(ParseBumpsDropsTest, BothBumps) {
  auto result = ParseBumpsDrops(0x03);
  EXPECT_TRUE(result.bump_right);
  EXPECT_TRUE(result.bump_left);
}

TEST(ParseBumpsDropsTest, WheelDropBoth) {
  auto result = ParseBumpsDrops(0x0C);
  EXPECT_FALSE(result.bump_right);
  EXPECT_FALSE(result.bump_left);
  EXPECT_TRUE(result.wheel_drop_right);
  EXPECT_TRUE(result.wheel_drop_left);
}

// ---------------------------------------------------------------------------
// ParseCliff
// ---------------------------------------------------------------------------

TEST(ParseCliffTest, NoCliff) {
  auto result = ParseCliff(0x00);
  EXPECT_FALSE(result.cliff_right);
  EXPECT_FALSE(result.cliff_front_right);
  EXPECT_FALSE(result.cliff_front_left);
  EXPECT_FALSE(result.cliff_left);
}

TEST(ParseCliffTest, AllCliff) {
  auto result = ParseCliff(0x0F);
  EXPECT_TRUE(result.cliff_right);
  EXPECT_TRUE(result.cliff_front_right);
  EXPECT_TRUE(result.cliff_front_left);
  EXPECT_TRUE(result.cliff_left);
}

TEST(ParseCliffTest, CliffFrontLeft) {
  auto result = ParseCliff(0x04);
  EXPECT_FALSE(result.cliff_right);
  EXPECT_FALSE(result.cliff_front_right);
  EXPECT_TRUE(result.cliff_front_left);
  EXPECT_FALSE(result.cliff_left);
}

// ---------------------------------------------------------------------------
// ParseInt16 / ParseUint16
// ---------------------------------------------------------------------------

TEST(ParseInt16Test, Positive) {
  // 0x00C8 = 200
  EXPECT_EQ(ParseInt16(0x00, 0xC8), 200);
}

TEST(ParseInt16Test, Negative) {
  // 0xFF9C = -100 in two's complement
  EXPECT_EQ(ParseInt16(0xFF, 0x9C), -100);
}

TEST(ParseInt16Test, Zero) {
  EXPECT_EQ(ParseInt16(0x00, 0x00), 0);
}

TEST(ParseUint16Test, BatteryVoltage) {
  // 0x4E20 = 20000 mV = 20 V
  EXPECT_EQ(ParseUint16(0x4E, 0x20), 20000);
}

}  // namespace roomba_ros2::oi::test
