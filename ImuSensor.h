#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "SwimTypes.h"

class ImuSensor {
 public:
  /// Scans the bus, identifies and initializes a supported IMU.
  bool begin(TwoWire &wire);
  /// Reads one calibrated sample and computes level-frame acceleration.
  bool read(ImuSample &sample, uint64_t monotonicUs);
  /// Reads the detected device identity register.
  uint8_t whoAmI();
  const char *modelName() const;
  uint8_t address() const { return address_; }
  /// Re-estimates gyro bias and installation attitude while stationary.
  bool calibrateStationary();

 private:
  /// Writes one MPU register over I2C.
  bool writeRegister(uint8_t reg, uint8_t value);
  /// Reads a contiguous MPU register block.
  bool readRegisters(uint8_t reg, uint8_t *buffer, size_t length);
  /// BMI323 uses 16-bit registers and returns two dummy bytes on I2C reads.
  bool readBmiRegisters(uint8_t reg, uint8_t *buffer, size_t wordCount);
  bool writeBmiRegister(uint8_t reg, uint16_t value);
  bool beginMpu6500();
  bool beginIcm42688();
  bool beginBmi323();
  /// Converts sensor axes to the enclosure frame: X bow, Y starboard, Z up.
  void mapAcceleration(float sensorX, float sensorY, float sensorZ,
                       float &instrumentX, float &instrumentY,
                       float &instrumentZ) const;
  void mapGyro(float sensorX, float sensorY, float sensorZ,
               float &instrumentX, float &instrumentY,
               float &instrumentZ) const;
  /// Updates roll and pitch with a complementary filter.
  void updateAttitude(ImuSample &sample, float dt);
  /// Rotates acceleration into the level frame and removes gravity.
  void calculateLevelAcceleration(ImuSample &sample);

  TwoWire *wire_ = nullptr;
  enum class Model : uint8_t { NONE, MPU6500, ICM42688, BMI323 };
  Model model_ = Model::NONE;
  uint8_t address_ = 0;
  float roll_ = 0;
  float pitch_ = 0;
  float zeroRoll_ = 0;
  float zeroPitch_ = 0;
  bool attitudeInitialized_ = false;
  float gyroBiasX_ = 0;
  float gyroBiasY_ = 0;
  float gyroBiasZ_ = 0;
};
