#ifndef LIBROOMBA_INCLUDE_STUB_VL53L1X_DRIVER_HPP_
#define LIBROOMBA_INCLUDE_STUB_VL53L1X_DRIVER_HPP_

#include <cstdint>

#include "libroomba/include/vl53l1x_driver.hpp"

// Stub VL53L1X driver for testing and simulation.
//
// Returns a fixed configurable distance without any hardware access.
// Useful for running tof_node with use_stub:=true in a lab/CI environment.
//
// Example:
//   StubVl53l1xDriver stub;
//   stub.SetDistance(150);          // inject 150 mm
//   int16_t mm = stub.ReadDistanceMm();  // returns 150

namespace roomba_ros2 {

class StubVl53l1xDriver : public Vl53l1xDriver {
 public:
  bool Init() override { return true; }

  int16_t ReadDistanceMm() override { return distance_mm_; }

  void Close() override {}

  // Inject a distance value for test scenarios.
  void SetDistance(int16_t mm) { distance_mm_ = mm; }

 private:
  int16_t distance_mm_{200};  // default stub distance [mm]
};

}  // namespace roomba_ros2

#endif  // LIBROOMBA_INCLUDE_STUB_VL53L1X_DRIVER_HPP_
