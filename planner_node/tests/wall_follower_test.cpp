#include "planner_node/wall_follower.hpp"

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
  cfg.corner_detect_threshold = 1000;  // disable corner detection for this test

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
  cfg.corner_detect_threshold = 1000;  // disable corner detection for this test

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

// ---------------------------------------------------------------------------
// FOLLOWING state — integral (I) term
// ---------------------------------------------------------------------------

TEST(WallFollowerTest, IntegralTermAccumulatesWithSteadyError) {
  roomba_ros2::WallFollowerConfig cfg;
  cfg.base_speed_mm_s = 100;
  cfg.target_wall_signal = 100;
  cfg.kp = 0.0F;  // P=0, D=0: only I drives correction
  cfg.kd = 0.0F;
  cfg.ki = 1.0F;
  cfg.max_correction_mm_s = 200;

  auto wf{MakeFollower(cfg)};
  ASSERT_EQ(wf.GetState(), roomba_ros2::WallFollower::State::kFollowing);

  // wall_signal=80: error=+20 each cycle, correction grows each tick
  wf.Update(WithWall(80), 50);
  auto s1{wf.GetWheelSpeeds()};  // i_error=20, correction=20

  wf.Update(WithWall(80), 50);
  auto s2{wf.GetWheelSpeeds()};  // i_error=40, correction=40

  // Each cycle the I-correction should be larger than the previous
  EXPECT_GT(s1.left_mm_s, 100);  // positive correction pushes left wheel up
  EXPECT_GT(s2.left_mm_s, s1.left_mm_s);
}

TEST(WallFollowerTest, IntegralTermResetsOnFollowingReEntry) {
  roomba_ros2::WallFollowerConfig cfg;
  cfg.base_speed_mm_s = 100;
  cfg.target_wall_signal = 100;
  cfg.kp = 0.0F;
  cfg.kd = 0.0F;
  cfg.ki = 1.0F;
  cfg.max_correction_mm_s = 200;
  cfg.recovery_backup_ms = 100;
  cfg.recovery_turn_ms = 100;

  auto wf{MakeFollower(cfg)};

  // Accumulate I term over several cycles
  wf.Update(WithWall(80), 50);  // error=20, i=20
  wf.Update(WithWall(80), 50);  // error=20, i=40
  auto s_before{wf.GetWheelSpeeds()};
  ASSERT_GT(s_before.left_mm_s, 100);

  // Trigger recovery and complete it (resets I term)
  wf.Update(WithBump(), 50);
  ASSERT_EQ(wf.GetState(), roomba_ros2::WallFollower::State::kRecovering);
  wf.Update(roomba_ros2::WallFollowerSensors{}, 250);  // complete recovery → FOLLOWING
  ASSERT_EQ(wf.GetState(), roomba_ros2::WallFollower::State::kFollowing);

  // First tick after re-entry: I term restarted from 0, so correction = ki*error = 1*20 = 20
  wf.Update(WithWall(80), 50);
  auto s_after{wf.GetWheelSpeeds()};
  EXPECT_EQ(s_after.left_mm_s, 120);  // base(100) + ki(1)*i_error(20) = 120
}

TEST(WallFollowerTest, IntegralAntiWindupClampsAccumulator) {
  // Use a direct WallFollower (not MakeFollower) to control wall_detect_threshold.
  roomba_ros2::WallFollowerConfig cfg;
  cfg.base_speed_mm_s = 100;
  cfg.target_wall_signal = 100;
  cfg.kp = 0.0F;
  cfg.kd = 0.0F;
  cfg.ki = 1.0F;
  cfg.max_correction_mm_s = 50;  // windup_limit = 50/1.0 = 50
  cfg.wall_detect_threshold = 10;
  cfg.wall_too_close_threshold = 300;

  roomba_ros2::WallFollower wf{cfg};
  wf.Update(WithWall(50), 50);  // enter FOLLOWING (signal 50 >= threshold 10)
  ASSERT_EQ(wf.GetState(), roomba_ros2::WallFollower::State::kFollowing);

  // Feed constant large error (error=50 each cycle): i_error would reach 500
  // without anti-windup, but is clamped at windup_limit=50.
  for (int i{0}; i < 10; ++i) {
    wf.Update(WithWall(50), 50);
  }
  auto s{wf.GetWheelSpeeds()};
  // Correction clamped to max_correction_mm_s=50 → left=150, right=50
  EXPECT_EQ(s.left_mm_s, 150);
  EXPECT_EQ(s.right_mm_s, 50);
}

// ---------------------------------------------------------------------------
// FOLLOWING state — corner detection (speed adaptation)
// ---------------------------------------------------------------------------

TEST(WallFollowerTest, CornerDetectionReducesSpeed) {
  roomba_ros2::WallFollowerConfig cfg;
  cfg.base_speed_mm_s = 100;
  cfg.target_wall_signal = 100;
  cfg.kp = 0.0F;  // no P/D/I correction so speed change is isolated
  cfg.kd = 0.0F;
  cfg.ki = 0.0F;
  cfg.corner_detect_threshold = 20;
  cfg.corner_speed_ratio = 0.5F;  // 50% speed at corners

  auto wf{MakeFollower(cfg)};
  ASSERT_EQ(wf.GetState(), roomba_ros2::WallFollower::State::kFollowing);

  // First tick: error=0→0, d_error=0 → straight, full speed
  wf.Update(WithWall(100), 50);
  auto s_straight{wf.GetWheelSpeeds()};
  EXPECT_EQ(s_straight.left_mm_s, 100);

  // Second tick: wall_signal jumps by 30 → |d_error|=30 > threshold(20) → corner
  wf.Update(WithWall(70), 50);
  auto s_corner{wf.GetWheelSpeeds()};
  EXPECT_EQ(s_corner.left_mm_s, 50);   // 100 * 0.5 = 50
  EXPECT_EQ(s_corner.right_mm_s, 50);
}

TEST(WallFollowerTest, NoCornerDetectionBelowThreshold) {
  roomba_ros2::WallFollowerConfig cfg;
  cfg.base_speed_mm_s = 100;
  cfg.target_wall_signal = 100;
  cfg.kp = 0.0F;
  cfg.kd = 0.0F;
  cfg.ki = 0.0F;
  cfg.corner_detect_threshold = 50;
  cfg.corner_speed_ratio = 0.5F;

  auto wf{MakeFollower(cfg)};
  wf.Update(WithWall(100), 50);  // error=0, prev_error=0

  // Small delta: |d_error|=10 < threshold(50) → full speed
  wf.Update(WithWall(90), 50);
  auto s{wf.GetWheelSpeeds()};
  EXPECT_EQ(s.left_mm_s, 100);
  EXPECT_EQ(s.right_mm_s, 100);
}

}  // namespace roomba_ros2::test
