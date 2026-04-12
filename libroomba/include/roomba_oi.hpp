#ifndef LIBROOMBA_INCLUDE_ROOMBA_OI_HPP_
#define LIBROOMBA_INCLUDE_ROOMBA_OI_HPP_

#include <algorithm>
#include <array>
#include <cstdint>

// Roomba Open Interface (OI) command constants and helper functions.
//
// Reference: iRobot Roomba 600 OI Specification
// https://edu.irobot.com/learning-library/roomba-open-interface
//
// Serial settings: 115200 baud, 8N1
// All multi-byte values are big-endian (high byte first).

namespace roomba_ros2::oi {

// ---------------------------------------------------------------------------
// Opcode constants
// ---------------------------------------------------------------------------

// Start the OI. Must be sent before any other command.
inline constexpr uint8_t kStart = 128;

// Enter Safe mode: full control, but Roomba stops on cliff/bump detection.
inline constexpr uint8_t kSafe = 131;

// Enter Full mode: full control, all safety features disabled.
inline constexpr uint8_t kFull = 132;

// Stop the OI and return to Passive mode.
inline constexpr uint8_t kStop = 173;

// Power off Roomba.
inline constexpr uint8_t kPower = 133;

// Drive both wheels at independent velocities.
// Payload: [right_vel_high] [right_vel_low] [left_vel_high] [left_vel_low]
// Velocity range: -500 to 500 mm/s (signed 16-bit, big-endian)
inline constexpr uint8_t kDriveDirect = 145;

// Drive with velocity and radius.
// Payload: [vel_high] [vel_low] [radius_high] [radius_low]
// radius = 32767 or 32768 → straight; 1 → spin left; -1 → spin right
inline constexpr uint8_t kDrive = 137;

// Request a single sensor packet.
// Payload: [packet_id]
inline constexpr uint8_t kSensors = 142;

// Request a list of sensor packets once.
// Payload: [num_packets] [packet_id_1] ... [packet_id_n]
inline constexpr uint8_t kQueryList = 149;

// ---------------------------------------------------------------------------
// Sensor packet IDs
// ---------------------------------------------------------------------------

// Bumps and wheel drops (1 byte)
// Bit 0: bump right, Bit 1: bump left
// Bit 2: wheel drop right, Bit 3: wheel drop left
inline constexpr uint8_t kPacketBumpsDrops = 7;

// Cliff sensors (1 byte)
// Bit 0: cliff right, Bit 1: cliff front right
// Bit 2: cliff front left, Bit 3: cliff left
inline constexpr uint8_t kPacketCliff = 9;

// Distance traveled since last request (2 bytes, signed, mm)
inline constexpr uint8_t kPacketDistance = 19;

// Angle turned since last request (2 bytes, signed, degrees)
inline constexpr uint8_t kPacketAngle = 20;

// Battery voltage (2 bytes, unsigned, mV)
inline constexpr uint8_t kPacketVoltage = 25;

// Battery current (2 bytes, signed, mA; negative = discharging)
inline constexpr uint8_t kPacketCurrent = 26;

// Wall sensor analog signal (2 bytes, unsigned, 0–4095; higher = closer to wall)
inline constexpr uint8_t kPacketWallSignal = 27;

// ---------------------------------------------------------------------------
// Command builders
// ---------------------------------------------------------------------------

// Build a DRIVE_DIRECT command (5 bytes).
// left_mm_s, right_mm_s: clamped to [-500, 500] mm/s.
inline std::array<uint8_t, 5> BuildDriveDirectCmd(int16_t left_mm_s, int16_t right_mm_s) {
  left_mm_s = std::clamp(left_mm_s, static_cast<int16_t>(-500), static_cast<int16_t>(500));
  right_mm_s = std::clamp(right_mm_s, static_cast<int16_t>(-500), static_cast<int16_t>(500));
  return {
      kDriveDirect,
      static_cast<uint8_t>((static_cast<uint16_t>(right_mm_s) >> 8) & 0xFF),
      static_cast<uint8_t>(static_cast<uint16_t>(right_mm_s) & 0xFF),
      static_cast<uint8_t>((static_cast<uint16_t>(left_mm_s) >> 8) & 0xFF),
      static_cast<uint8_t>(static_cast<uint16_t>(left_mm_s) & 0xFF),
  };
}

// Build a SENSORS command (2 bytes) for a single packet.
inline std::array<uint8_t, 2> BuildSensorsCmd(uint8_t packet_id) {
  return {kSensors, packet_id};
}

// Build a QUERY_LIST command for a set of packets.
// Returns the byte sequence: [kQueryList, n, id_1, ..., id_n]
template <std::size_t N>
std::array<uint8_t, N + 2> BuildQueryListCmd(const std::array<uint8_t, N>& packet_ids) {
  std::array<uint8_t, N + 2> cmd{};
  cmd[0] = kQueryList;
  cmd[1] = static_cast<uint8_t>(N);
  for (std::size_t i = 0; i < N; ++i) {
    cmd[i + 2] = packet_ids[i];
  }
  return cmd;
}

// ---------------------------------------------------------------------------
// Sensor response parsers
// ---------------------------------------------------------------------------

// Parsed bumps and wheel drops from packet 7.
struct BumpsDrops {
  bool bump_right{false};
  bool bump_left{false};
  bool wheel_drop_right{false};
  bool wheel_drop_left{false};
};

// Parse a Bumps and Wheel Drops byte (packet 7).
inline BumpsDrops ParseBumpsDrops(uint8_t byte) {
  return {(byte & 0x01) != 0, (byte & 0x02) != 0, (byte & 0x04) != 0, (byte & 0x08) != 0};
}

// Parsed cliff sensor flags from packet 9.
struct CliffSensors {
  bool cliff_right{false};
  bool cliff_front_right{false};
  bool cliff_front_left{false};
  bool cliff_left{false};
};

// Parse a Cliff byte (packet 9).
inline CliffSensors ParseCliff(uint8_t byte) {
  return {(byte & 0x01) != 0, (byte & 0x02) != 0, (byte & 0x04) != 0, (byte & 0x08) != 0};
}

// Decode a big-endian signed 16-bit value from two bytes.
inline int16_t ParseInt16(uint8_t high, uint8_t low) {
  return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}

// Decode a big-endian unsigned 16-bit value from two bytes.
inline uint16_t ParseUint16(uint8_t high, uint8_t low) {
  return static_cast<uint16_t>((static_cast<uint16_t>(high) << 8) | low);
}

}  // namespace roomba_ros2::oi

#endif  // LIBROOMBA_INCLUDE_ROOMBA_OI_HPP_
