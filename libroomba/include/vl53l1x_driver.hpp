#ifndef LIBROOMBA_INCLUDE_VL53L1X_DRIVER_HPP_
#define LIBROOMBA_INCLUDE_VL53L1X_DRIVER_HPP_

#include <cstdint>

// Abstract HAL interface for the VL53L1X Time-of-Flight distance sensor.
//
// Implementations:
//   LinuxVl53l1xDriver — real I2C access via /dev/i2c-N (defined in tof_node.cpp)
//   StubVl53l1xDriver  — fixed-value stub for testing (stub_vl53l1x_driver.hpp)
//
// Usage:
//   driver.Init();                  // initialize sensor, start continuous ranging
//   int16_t mm = driver.ReadDistanceMm();  // < 0 on error
//   driver.Close();

namespace roomba {

class Vl53l1xDriver {
 public:
  Vl53l1xDriver() = default;
  virtual ~Vl53l1xDriver() = default;
  Vl53l1xDriver(const Vl53l1xDriver&) = delete;
  Vl53l1xDriver& operator=(const Vl53l1xDriver&) = delete;
  Vl53l1xDriver(Vl53l1xDriver&&) = delete;
  Vl53l1xDriver& operator=(Vl53l1xDriver&&) = delete;

  // Initialize the sensor and start continuous ranging.
  // Returns true on success.
  virtual bool Init() = 0;

  // Read the latest distance measurement in mm.
  // Returns a negative value if no valid measurement is available.
  virtual int16_t ReadDistanceMm() = 0;

  // Stop ranging and release I2C resources.
  virtual void Close() = 0;
};

}  // namespace roomba

#endif  // LIBROOMBA_INCLUDE_VL53L1X_DRIVER_HPP_
