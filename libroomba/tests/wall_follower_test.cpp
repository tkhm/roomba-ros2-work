#include "libroomba/include/wall_follower.hpp"

#include <gtest/gtest.h>

namespace roomba_ros2::test {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

roomba_ros2::WallFollowerSensors NoSensors() {
  return {};
}

roomba_ros2::WallFollowerSensors WithWall(uint16_t signal) {
  roomba_ros2::WallFollowerSensors s;
  s.wall_signal = signal;
  return s;
}

roomba_ros2::WallFollowerSensors WithBump() {
  roomba_ros2::WallFollowerSensors s;
  s.bump_left = true;
  return s;
}

roomba_ros2::WallFollowerSensors WithCliff() {
  roomba_ros2::WallFollowerSensors s;
  s.cliff_front_right = true;
  return s;
}

roomba_ros2::WallFollowerSensors WithWallAndCliff(uint16_t signal) {
  roomba_ros2::WallFollowerSensors s;
  s.wall_signal = signal;
  s.cliff_front_right = true;
  return s;
}

// ---------------------------------------------------------------------------
// SEARCHING state
// ---------------------------------------------------------------------------

TEST(WallFollowerTest, StartsInSearchingState) {
  roomba_ros2::WallFollower wf;
  EXPECT_EQ(wf.GetState(), roomba_ros2::WallFollower::State::kSearching);
}

TEST(WallFollowerTest, SearchingAppliesRightBias) {
  roomba_ros2::WallFollowerConfig cfg;
  cfg.base_speed_mm_s = 100;
  cfg.search_turn_bias_mm_s = 30;
  cfg.wall_detect_threshold = 50;

  roomba_ros2::WallFollower wf{cfg};
  wf.Update(NoSensors(), 50);

  auto speeds{wf.GetWheelSpeeds()};
  EXPECT_EQ(speeds.left_mm_s, 100);
  EXPECT_EQ(speeds.right_mm_s, 70);
  EXPECT_EQ(wf.GetState(), roomba_ros2::WallFollower::State::kSearching);
}

TEST(WallFollowerTest, SearchingTransitionsToFollowingWhenWallDetected) {
  roomba_ros2::WallFollowerConfig cfg;
  cfg.wall_detect_threshold = 50;

  roomba_ros2::WallFollower wf{cfg};
  wf.Update(WithWall(60), 50);

  EXPECT_EQ(wf.GetState(), roomba_ros2::WallFollower::State::kFollowing);
}

TEST(WallFollowerTest, SearchingTransitionsToRecoveringOnBump) {
  roomba_ros2::WallFollower wf;
  wf.Update(WithBump(), 50);
  EXPECT_EQ(wf.GetState(), roomba_ros2::WallFollower::State::kRecovering);
}

// ---------------------------------------------------------------------------
// FOLLOWING state — P-control
// ---------------------------------------------------------------------------

roomba_ros2::WallFollower MakeFollower(roomba_ros2::WallFollowerConfig cfg = {}) {
  cfg.wall_detect_threshold = 50;
  cfg.wall_too_close_threshold = 300;
  roomba_ros2::WallFollower wf{cfg};
  wf.Update(WithWall(100), 50);
  return wf;
}

TEST(WallFollowerTest, FollowingAtTargetProducesNoCorrection) {
  roomba_ros2::WallFollowerConfig cfg;
  cfg.base_speed_mm_s = 100;
  cfg.target_wall_signal = 100;
  cfg.kp = 0.5F;

  auto wf{MakeFollower(cfg)};
  ASSERT_EQ(wf.GetState(), roomba_ros2::WallFollower::State::kFollowing);

  wf.Update(WithWall(100), 50);

  auto speeds{wf.GetWheelSpeeds()};
  EXPECT_EQ(speeds.left_mm_s, 100);
  EXPECT_EQ(speeds.right_mm_s, 100);
}

TEST(WallFollowerTest, FollowingTooCloseCorrectsTurnLeft) {
  roomba_ros2::WallFollowerConfig cfg;
  cfg.base_speed_mm_s = 100;
  cfg.target_wall_signal = 100;
  cfg.kp = 1.0F;
  cfg.max_correction_mm_s = 80;

  auto wf{MakeFollower(cfg)};
  wf.Update(WithWall(140), 50);

  auto speeds{wf.GetWheelSpeeds()};
  EXPECT_EQ(speeds.left_mm_s, 60);
  EXPECT_EQ(speeds.right_mm_s, 140);
}

TEST(WallFollowerTest, FollowingTooFarCorrectsTurnRight) {
  roomba_ros2::WallFollowerConfig cfg;
  cfg.base_speed_mm_s = 100;
  cfg.target_wall_signal = 100;
  cfg.kp = 1.0F;
  cfg.max_correction_mm_s = 80;

  auto wf{MakeFollower(cfg)};
  wf.Update(WithWall(60), 50);

  auto speeds{wf.GetWheelSpeeds()};
  EXPECT_EQ(speeds.left_mm_s, 140);
  EXPECT_EQ(speeds.right_mm_s, 60);
}

TEST(WallFollowerTest, FollowingCorrectionClampedByMax) {
  roomba_ros2::WallFollowerConfig cfg;
  cfg.base_speed_mm_s = 100;
  cfg.target_wall_signal = 100;
  cfg.kp = 2.0F;
  cfg.max_correction_mm_s = 50;

  auto wf{MakeFollower(cfg)};
  wf.Update(WithWall(200), 50);

  auto speeds{wf.GetWheelSpeeds()};
  EXPECT_EQ(speeds.left_mm_s, 50);
  EXPECT_EQ(speeds.right_mm_s, 150);
}

TEST(WallFollowerTest, FollowingTransitionsToRecoveringOnBump) {
  auto wf{MakeFollower()};
  wf.Update(WithBump(), 50);
  EXPECT_EQ(wf.GetState(), roomba_ros2::WallFollower::State::kRecovering);
}

TEST(WallFollowerTest, FollowingTransitionsToSearchingWhenWallLost) {
  roomba_ros2::WallFollowerConfig cfg;
  cfg.wall_detect_threshold = 50;

  auto wf{MakeFollower(cfg)};
  wf.Update(WithWall(20), 50);
  EXPECT_EQ(wf.GetState(), roomba_ros2::WallFollower::State::kSearching);
}

// ---------------------------------------------------------------------------
// FOLLOWING state — cliff debounce
// ---------------------------------------------------------------------------

TEST(WallFollowerTest, CliffSingleCycleDoesNotTrigger) {
  roomba_ros2::WallFollowerConfig cfg;
  cfg.cliff_debounce_count = 3;

  auto wf{MakeFollower(cfg)};
  wf.Update(WithWallAndCliff(100), 50);  // count = 1
  EXPECT_EQ(wf.GetState(), roomba_ros2::WallFollower::State::kFollowing);
}

TEST(WallFollowerTest, CliffTriggersAfterDebounceCount) {
  roomba_ros2::WallFollowerConfig cfg;
  cfg.cliff_debounce_count = 3;

  auto wf{MakeFollower(cfg)};
  wf.Update(WithWallAndCliff(100), 50);  // count = 1
  wf.Update(WithWallAndCliff(100), 50);  // count = 2
  EXPECT_EQ(wf.GetState(), roomba_ros2::WallFollower::State::kFollowing);

  wf.Update(WithWallAndCliff(100), 50);  // count = 3 → triggers
  EXPECT_EQ(wf.GetState(), roomba_ros2::WallFollower::State::kSearching);

  auto s{wf.GetWheelSpeeds()};
  EXPECT_EQ(s.left_mm_s, 0);
  EXPECT_EQ(s.right_mm_s, 0);
}

TEST(WallFollowerTest, CliffCountResetsOnClear) {
  roomba_ros2::WallFollowerConfig cfg;
  cfg.cliff_debounce_count = 3;

  auto wf{MakeFollower(cfg)};
  wf.Update(WithWallAndCliff(100), 50);  // count = 1
  wf.Update(WithWallAndCliff(100), 50);  // count = 2
  wf.Update(WithWall(100), 50);          // cleared → count = 0
  wf.Update(WithWallAndCliff(100), 50);  // count = 1 (fresh)
  EXPECT_EQ(wf.GetState(), roomba_ros2::WallFollower::State::kFollowing);
}

// ---------------------------------------------------------------------------
// FOLLOWING state — emergency avoidance
// ---------------------------------------------------------------------------

TEST(WallFollowerTest, EmergencyAvoidanceFiresWhenTooClose) {
  roomba_ros2::WallFollowerConfig cfg;
  cfg.base_speed_mm_s = 100;
  cfg.target_wall_signal = 100;
  cfg.wall_too_close_threshold = 250;
  cfg.emergency_turn_speed_mm_s = 80;

  auto wf{MakeFollower(cfg)};
  wf.Update(WithWall(300), 50);

  auto speeds{wf.GetWheelSpeeds()};
  EXPECT_EQ(speeds.left_mm_s, 0);
  EXPECT_EQ(speeds.right_mm_s, 80);
  EXPECT_EQ(wf.GetState(), roomba_ros2::WallFollower::State::kFollowing);
}

TEST(WallFollowerTest, EmergencyAvoidanceNotFiredBelowThreshold) {
  roomba_ros2::WallFollowerConfig cfg;
  cfg.base_speed_mm_s = 100;
  cfg.target_wall_signal = 100;
  cfg.wall_too_close_threshold = 250;
  cfg.kp = 1.0F;
  cfg.max_correction_mm_s = 80;

  auto wf{MakeFollower(cfg)};
  wf.Update(WithWall(200), 50);

  auto speeds{wf.GetWheelSpeeds()};
  EXPECT_EQ(speeds.left_mm_s, 20);
  EXPECT_EQ(speeds.right_mm_s, 180);
}

// ---------------------------------------------------------------------------
// RECOVERING state — two-phase (backup + spin)
// ---------------------------------------------------------------------------

TEST(WallFollowerTest, RecoveringPhase1StraightReverse) {
  roomba_ros2::WallFollowerConfig cfg;
  cfg.recovery_backup_ms = 600;
  cfg.recovery_backup_speed_mm_s = 80;
  cfg.recovery_turn_ms = 800;
  cfg.recovery_turn_speed_mm_s = 80;

  auto wf{MakeFollower(cfg)};
  wf.Update(WithBump(), 50);
  ASSERT_EQ(wf.GetState(), roomba_ros2::WallFollower::State::kRecovering);

  wf.Update(NoSensors(), 300);  // within backup phase
  auto s{wf.GetWheelSpeeds()};
  EXPECT_EQ(s.left_mm_s, -80);
  EXPECT_EQ(s.right_mm_s, -80);
}

TEST(WallFollowerTest, RecoveringPhase2SpinLeft) {
  roomba_ros2::WallFollowerConfig cfg;
  cfg.recovery_backup_ms = 600;
  cfg.recovery_backup_speed_mm_s = 80;
  cfg.recovery_turn_ms = 800;
  cfg.recovery_turn_speed_mm_s = 80;

  auto wf{MakeFollower(cfg)};
  wf.Update(WithBump(), 50);

  wf.Update(NoSensors(), 650);  // into spin phase (elapsed > 600ms)
  auto s{wf.GetWheelSpeeds()};
  EXPECT_EQ(s.left_mm_s, -80);
  EXPECT_EQ(s.right_mm_s, 80);
}

TEST(WallFollowerTest, RecoveringCompletesAndTransitionsToFollowing) {
  roomba_ros2::WallFollowerConfig cfg;
  cfg.recovery_backup_ms = 100;
  cfg.recovery_turn_ms = 100;

  auto wf{MakeFollower(cfg)};
  wf.Update(WithBump(), 50);

  wf.Update(NoSensors(), 250);  // exceeds backup + turn
  EXPECT_EQ(wf.GetState(), roomba_ros2::WallFollower::State::kFollowing);

  auto s{wf.GetWheelSpeeds()};
  EXPECT_EQ(s.left_mm_s, 0);
  EXPECT_EQ(s.right_mm_s, 0);
}

TEST(WallFollowerTest, RecoveryElapsedResetsAfterCompletion) {
  roomba_ros2::WallFollowerConfig cfg;
  cfg.recovery_backup_ms = 100;
  cfg.recovery_turn_ms = 100;
  cfg.recovery_backup_speed_mm_s = 80;

  auto wf{MakeFollower(cfg)};

  // First recovery
  wf.Update(WithBump(), 50);
  wf.Update(NoSensors(), 250);
  EXPECT_EQ(wf.GetState(), roomba_ros2::WallFollower::State::kFollowing);

  // Second bump starts a fresh recovery
  wf.Update(WithBump(), 50);
  ASSERT_EQ(wf.GetState(), roomba_ros2::WallFollower::State::kRecovering);

  wf.Update(NoSensors(), 50);  // only 50 ms into new recovery → backup phase
  auto s{wf.GetWheelSpeeds()};
  EXPECT_EQ(s.left_mm_s, -80);
  EXPECT_EQ(s.right_mm_s, -80);
}

}  // namespace roomba_ros2::test
