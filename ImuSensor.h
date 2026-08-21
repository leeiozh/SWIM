#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "SwimTypes.h"

class ImuSensor {
 public:
  /// Initializes and validates the MPU-6500 on the supplied I2C bus.
  bool begin(TwoWire &wire);
  /// Reads one calibrated sample and computes level-frame acceleration.
  bool read(ImuSample &sample, uint64_t monotonicUs);
  /// Reads the MPU identity register.
  uint8_t whoAmI();
  /// Re-estimates gyro bias and installation attitude while stationary.
  bool calibrateStationary();

 private:
  /// Writes one MPU register over I2C.
  bool writeRegister(uint8_t reg, uint8_t value);
  /// Reads a contiguous MPU register block.
  bool readRegisters(uint8_t reg, uint8_t *buffer, size_t length);
  /// Updates roll and pitch with a complementary filter.
  void updateAttitude(ImuSample &sample, float dt);
  /// Rotates acceleration into the level frame and removes gravity.
  void calculateLevelAcceleration(ImuSample &sample);

  TwoWire *wire_ = nullptr;
  float roll_ = 0;
  float pitch_ = 0;
  float zeroRoll_ = 0;
  float zeroPitch_ = 0;
  bool attitudeInitialized_ = false;
  float gyroBiasX_ = 0;
  float gyroBiasY_ = 0;
  float gyroBiasZ_ = 0;
};
