#ifndef LIBROOMBA_INCLUDE_WALL_FOLLOWER_HPP_
#define LIBROOMBA_INCLUDE_WALL_FOLLOWER_HPP_

#include <algorithm>
#include <cstdint>

// Wall-following state machine and P-controller for Roomba 600.
//
// The robot keeps the right-side wall sensor at a target signal value.
// It uses three states:
//   SEARCHING  — No wall detected; drive forward to search for wall.
//   FOLLOWING  — Wall detected; P-control maintains target wall distance.
//               If wall_signal exceeds wall_too_close_threshold, an emergency
//               left turn fires before contact occurs.
//   RECOVERING — Bumper hit; two-phase recovery:
//               Phase 1: straight reverse to gain clearance.
//               Phase 2: spin left to reorient away from the obstacle.
//               Then transition back to FOLLOWING.
//
// Cliff sensor debounce: a cliff reading must persist for cliff_debounce_count
// consecutive cycles before the robot reacts, filtering out floor/IR noise.
//
// Usage (call Update() at fixed intervals):
//   WallFollower wf{config};
//   wf.Update(sensors, elapsed_ms);
//   auto [left, right] = wf.GetWheelSpeeds();

namespace roomba {

// Configuration parameters for WallFollower (all tunable at runtime).
struct WallFollowerConfig {
  // Forward base speed [mm/s] used in SEARCHING and FOLLOWING states.
  int16_t base_speed_mm_s{100};

  // Target wall sensor signal.  Higher value = closer to wall.
  // Typical range: 50–200 (tune for actual wall distance).
  uint16_t target_wall_signal{100};

  // Proportional gain for wall distance P-controller.
  // correction = kp * (target - actual)
  // Applied as: left += correction, right -= correction.
  float kp{0.5F};

  // Emergency avoidance threshold [wall_signal units].
  // When wall_signal exceeds this value the P-controller is bypassed and a
  // full emergency left turn is applied to avoid contact.
  // Set higher than target_wall_signal (e.g. target * 2.5).
  uint16_t wall_too_close_threshold{250};

  // Wheel speed for emergency left turn [mm/s].
  // Left wheel stops while right wheel pushes forward.
  int16_t emergency_turn_speed_mm_s{80};

  // Phase 1 of RECOVERING: straight reverse duration [ms].
  // Provides clearance from the obstacle before spinning.
  int32_t recovery_backup_ms{600};

  // Reverse speed during Phase 1 [mm/s].
  int16_t recovery_backup_speed_mm_s{80};

  // Phase 2 of RECOVERING: spin-left duration [ms].
  // Larger values produce more rotation to clear corners.
  int32_t recovery_turn_ms{800};

  // Spin speed during Phase 2 [mm/s] (left = -speed, right = +speed).
  int16_t recovery_turn_speed_mm_s{80};

  // Speed bias applied to search: right wheel is slower by this amount [mm/s].
  // A small positive value drifts the robot rightward toward the wall.
  // Set to 0 to go straight (safer in open areas).
  int16_t search_turn_bias_mm_s{0};

  // wall_signal threshold above which wall is considered "found".
  uint16_t wall_detect_threshold{30};

  // Maximum correction magnitude [mm/s] to prevent overcorrection.
  int16_t max_correction_mm_s{80};

  // Number of consecutive cycles a cliff sensor must fire before the robot
  // reacts.  Filters transient false positives from dark floors or IR noise.
  // At 20 Hz: 3 = 150 ms, 5 = 250 ms.
  int32_t cliff_debounce_count{3};
};

// Sensor inputs consumed by WallFollower (subset of RoombaSensors).
struct WallFollowerSensors {
  bool bump_left{false};
  bool bump_right{false};
  bool cliff_left{false};
  bool cliff_front_left{false};
  bool cliff_front_right{false};
  bool cliff_right{false};
  uint16_t wall_signal{0};
};

// Wheel speed command produced by WallFollower.
struct WheelSpeeds {
  int16_t left_mm_s{0};
  int16_t right_mm_s{0};
};

class WallFollower {
 public:
  enum class State { kSearching, kFollowing, kRecovering };

  explicit WallFollower(WallFollowerConfig config = {}) : config_{config} {}

  // Update state machine with current sensor readings and elapsed time [ms].
  // Call this at a fixed rate (e.g., 20 Hz → elapsed_ms = 50).
  void Update(const WallFollowerSensors& sensors, int32_t elapsed_ms) {
    switch (state_) {
      case State::kSearching:
        UpdateSearching(sensors);
        break;
      case State::kFollowing:
        UpdateFollowing(sensors);
        break;
      case State::kRecovering:
        UpdateRecovering(elapsed_ms);
        break;
    }
  }

  WheelSpeeds GetWheelSpeeds() const { return speeds_; }
  State GetState() const { return state_; }

 private:
  // Returns true if cliff has been detected for cliff_debounce_count cycles.
  bool CliffConfirmed(const WallFollowerSensors& sensors) {
    bool any_cliff{sensors.cliff_left || sensors.cliff_front_left ||
                   sensors.cliff_front_right || sensors.cliff_right};
    if (any_cliff) {
      cliff_count_ = std::min(cliff_count_ + 1, config_.cliff_debounce_count);
    } else {
      cliff_count_ = 0;
    }
    return cliff_count_ >= config_.cliff_debounce_count;
  }

  void UpdateSearching(const WallFollowerSensors& sensors) {
    if (sensors.bump_left || sensors.bump_right) {
      EnterRecovering();
      return;
    }
    if (sensors.wall_signal >= config_.wall_detect_threshold) {
      cliff_count_ = 0;
      state_ = State::kFollowing;
      return;
    }
    // Go straight (or with slight right bias) to search for wall
    speeds_.left_mm_s = config_.base_speed_mm_s;
    speeds_.right_mm_s =
        static_cast<int16_t>(config_.base_speed_mm_s - config_.search_turn_bias_mm_s);
  }

  void UpdateFollowing(const WallFollowerSensors& sensors) {
    if (sensors.bump_left || sensors.bump_right) {
      cliff_count_ = 0;
      EnterRecovering();
      return;
    }
    // Cliff detected (debounced): stop and return to searching
    if (CliffConfirmed(sensors)) {
      speeds_ = {0, 0};
      cliff_count_ = 0;
      state_ = State::kSearching;
      return;
    }
    // Wall lost: return to searching
    if (sensors.wall_signal < config_.wall_detect_threshold) {
      state_ = State::kSearching;
      return;
    }
    // Emergency avoidance: wall too close → hard left turn before bumper triggers
    if (sensors.wall_signal >= config_.wall_too_close_threshold) {
      speeds_.left_mm_s = 0;
      speeds_.right_mm_s = config_.emergency_turn_speed_mm_s;
      return;
    }
    // Normal P-control
    float error{static_cast<float>(config_.target_wall_signal) -
                static_cast<float>(sensors.wall_signal)};
    auto correction{static_cast<int16_t>(std::clamp(
        config_.kp * error, static_cast<float>(-config_.max_correction_mm_s),
        static_cast<float>(config_.max_correction_mm_s)))};
    speeds_.left_mm_s =
        std::clamp(static_cast<int16_t>(config_.base_speed_mm_s + correction),
                   static_cast<int16_t>(-500), static_cast<int16_t>(500));
    speeds_.right_mm_s =
        std::clamp(static_cast<int16_t>(config_.base_speed_mm_s - correction),
                   static_cast<int16_t>(-500), static_cast<int16_t>(500));
  }

  void UpdateRecovering(int32_t elapsed_ms) {
    recovery_elapsed_ms_ += elapsed_ms;

    if (recovery_elapsed_ms_ < config_.recovery_backup_ms) {
      // Phase 1: straight reverse for clearance
      speeds_.left_mm_s = static_cast<int16_t>(-config_.recovery_backup_speed_mm_s);
      speeds_.right_mm_s = static_cast<int16_t>(-config_.recovery_backup_speed_mm_s);
    } else if (recovery_elapsed_ms_ <
               config_.recovery_backup_ms + config_.recovery_turn_ms) {
      // Phase 2: spin left to reorient
      speeds_.left_mm_s = static_cast<int16_t>(-config_.recovery_turn_speed_mm_s);
      speeds_.right_mm_s = config_.recovery_turn_speed_mm_s;
    } else {
      // Recovery complete: return to FOLLOWING so P-control can re-acquire wall
      speeds_ = {0, 0};
      recovery_elapsed_ms_ = 0;
      cliff_count_ = 0;
      state_ = State::kFollowing;
    }
  }

  void EnterRecovering() {
    state_ = State::kRecovering;
    recovery_elapsed_ms_ = 0;
    speeds_ = {0, 0};
  }

  WallFollowerConfig config_;
  State state_{State::kSearching};
  WheelSpeeds speeds_{};
  int32_t recovery_elapsed_ms_{0};
  int32_t cliff_count_{0};
};

}  // namespace roomba

#endif  // LIBROOMBA_INCLUDE_WALL_FOLLOWER_HPP_
