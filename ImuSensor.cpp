#include "ImuSensor.h"

#include <math.h>
#include "SWIMConfig.h"

namespace {
constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_CONFIG2 = 0x1D;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr uint8_t ICM_REG_DEVICE_CONFIG = 0x11;
constexpr uint8_t ICM_REG_ACCEL_DATA_X1 = 0x1F;
constexpr uint8_t ICM_REG_PWR_MGMT0 = 0x4E;
constexpr uint8_t ICM_REG_GYRO_CONFIG0 = 0x4F;
constexpr uint8_t ICM_REG_ACCEL_CONFIG0 = 0x50;
constexpr uint8_t BMI_REG_CHIP_ID = 0x00;
constexpr uint8_t BMI_REG_ACCEL_X = 0x03;
constexpr uint8_t BMI_REG_ACCEL_CONF = 0x20;
constexpr uint8_t BMI_REG_GYRO_CONF = 0x21;

constexpr float G0 = 9.80665f;
constexpr float ATTITUDE_TAU = 2.0f;
// Device-specific calibration. Gyro values are stationary zero offsets;
// accelerometer values use corrected=(raw-bias)/scale from the six-face run.
// SET ZERO is deliberately separate: it compensates installation attitude and
// never changes these sensor calibration coefficients.
constexpr float MPU_GYRO_BIAS_X = -4.089649f;
constexpr float MPU_GYRO_BIAS_Y = 1.033756f;
constexpr float MPU_GYRO_BIAS_Z = 0.389588f;
constexpr float MPU_ACC_BIAS_X = -0.015260f;
constexpr float MPU_ACC_BIAS_Y = -0.009653f;
constexpr float MPU_ACC_BIAS_Z = -0.023275f;
constexpr float MPU_ACC_SCALE_X = 1.000437f;
constexpr float MPU_ACC_SCALE_Y = 1.000363f;
constexpr float MPU_ACC_SCALE_Z = 1.004489f;

// Six-position calibration of the replacement ICM-42688-P in its final
// enclosure, 2026-09-19. Corrected acceleration is (raw_g-bias)/scale.
constexpr float ICM_GYRO_BIAS_X = -0.364869f;
constexpr float ICM_GYRO_BIAS_Y = 0.044432f;
constexpr float ICM_GYRO_BIAS_Z = -0.823456f;
constexpr float ICM_ACC_BIAS_X = 0.000628f;
constexpr float ICM_ACC_BIAS_Y = -0.001390f;
constexpr float ICM_ACC_BIAS_Z = 0.009199f;
constexpr float ICM_ACC_SCALE_X = 1.000903f;
constexpr float ICM_ACC_SCALE_Y = 0.999589f;
constexpr float ICM_ACC_SCALE_Z = 0.998392f;

// Six-position calibration of the BMI323 / GY-601N1 in prototype 2,
// 2026-09-19. Corrected acceleration is (raw_g-bias)/scale.
constexpr float BMI_GYRO_BIAS_X = 0.011613f;
constexpr float BMI_GYRO_BIAS_Y = 0.101537f;
constexpr float BMI_GYRO_BIAS_Z = 0.094809f;
constexpr float BMI_ACC_BIAS_X = 0.017527f;
constexpr float BMI_ACC_BIAS_Y = 0.008747f;
constexpr float BMI_ACC_BIAS_Z = -0.010387f;
constexpr float BMI_ACC_SCALE_X = 1.000850f;
constexpr float BMI_ACC_SCALE_Y = 1.002014f;
constexpr float BMI_ACC_SCALE_Z = 1.002354f;
}  // namespace

bool ImuSensor::writeRegister(uint8_t reg, uint8_t value) {
  wire_->beginTransmission(address_);
  wire_->write(reg);
  wire_->write(value);
  return wire_->endTransmission(true) == 0;
}

bool ImuSensor::readRegisters(uint8_t reg, uint8_t *buffer, size_t length) {
  wire_->beginTransmission(address_);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) return false;
  const size_t received = wire_->requestFrom(
      static_cast<uint16_t>(address_), length, true);
  if (received != length) return false;
  for (size_t i = 0; i < length; ++i) buffer[i] = wire_->read();
  return true;
}

bool ImuSensor::readBmiRegisters(uint8_t reg, uint8_t *buffer,
                                 size_t wordCount) {
  wire_->beginTransmission(address_);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) return false;
  const size_t payloadLength = wordCount * 2;
  const size_t requested = payloadLength + 2;  // BMI323 I2C dummy bytes.
  const size_t received = wire_->requestFrom(
      static_cast<uint16_t>(address_), requested, true);
  if (received != requested) return false;
  wire_->read();
  wire_->read();
  for (size_t i = 0; i < payloadLength; ++i) buffer[i] = wire_->read();
  return true;
}

bool ImuSensor::writeBmiRegister(uint8_t reg, uint16_t value) {
  wire_->beginTransmission(address_);
  wire_->write(reg);
  wire_->write(static_cast<uint8_t>(value));
  wire_->write(static_cast<uint8_t>(value >> 8));
  return wire_->endTransmission(true) == 0;
}

uint8_t ImuSensor::whoAmI() {
  uint8_t value = 0xFF;
  return readRegisters(REG_WHO_AM_I, &value, 1) ? value : 0xFF;
}

bool ImuSensor::begin(TwoWire &wire) {
  wire_ = &wire;
  model_ = Model::NONE;
  address_ = 0;

  Serial.println("IMU: scanning I2C bus");
  for (uint8_t candidate = 1; candidate < 0x7F; ++candidate) {
    wire_->beginTransmission(candidate);
    if (wire_->endTransmission(true) == 0) {
      Serial.printf("IMU: I2C device found at 0x%02X\n", candidate);
      if (!address_ && (candidate == SwimConfig::IMU_ADDRESS_LOW ||
                        candidate == SwimConfig::IMU_ADDRESS_HIGH))
        address_ = candidate;
    }
  }
  if (!address_) {
    Serial.println("IMU: no device at 0x68/0x69");
    return false;
  }

  const uint8_t id = whoAmI();
  Serial.printf("IMU: address=0x%02X WHO_AM_I=0x%02X\n", address_, id);
  if (id == 0x47) {
    model_ = Model::ICM42688;
    return beginIcm42688();
  }
  if (id == 0x70) {
    model_ = Model::MPU6500;
    return beginMpu6500();
  }
  // GY-601N1 is a carrier-board name, not a sensor model. It is sold with
  // ICM-42688-P, ICM-45686 or BMI323. Probe their read-only identity
  // registers so an unsupported population can be identified without making
  // configuration writes to an unknown chip.
  uint8_t icm45686Id = 0xFF;
  uint8_t bmiId[2] = {0xFF, 0xFF};
  const bool icm45686Readable = readRegisters(0x72, &icm45686Id, 1);
  const bool bmiReadable = readBmiRegisters(BMI_REG_CHIP_ID, bmiId, 1);
  Serial.printf("IMU: identity probes: reg75=0x%02X reg72=%s0x%02X reg00=%s0x%02X%02X\n",
                id, icm45686Readable ? "" : "read-error/", icm45686Id,
                bmiReadable ? "" : "read-error/", bmiId[0], bmiId[1]);
  if (icm45686Readable && icm45686Id == 0xE9) {
    Serial.println("IMU: detected ICM-45686 on GY-601N1; driver not enabled yet");
    return false;
  }
  if (bmiReadable && (bmiId[0] == 0x43 || bmiId[1] == 0x43)) {
    Serial.println("IMU: detected BMI323 on GY-601N1");
    model_ = Model::BMI323;
    return beginBmi323();
  }
  Serial.printf("IMU: unsupported WHO_AM_I=0x%02X; refusing to assume GY-601N1 chip\n", id);
  return false;
}

bool ImuSensor::beginMpu6500() {

  if (!writeRegister(REG_PWR_MGMT_1, 0x80)) return false;
  delay(150);
  if (!writeRegister(REG_PWR_MGMT_1, 0x01)) return false;
  delay(150);
  if (!writeRegister(REG_CONFIG, 0x03)) return false;
  if (!writeRegister(REG_SMPLRT_DIV, 9)) return false;       // 100 Hz from 1 kHz base
  if (!writeRegister(REG_GYRO_CONFIG, 0x08)) return false;   // +/-500 dps
  if (!writeRegister(REG_ACCEL_CONFIG, 0x08)) return false;  // +/-4 g
  if (!writeRegister(REG_ACCEL_CONFIG2, 0x03)) return false;
  gyroBiasX_ = MPU_GYRO_BIAS_X;
  gyroBiasY_ = MPU_GYRO_BIAS_Y;
  gyroBiasZ_ = MPU_GYRO_BIAS_Z;
  delay(100);
  return whoAmI() == 0x70;
}

bool ImuSensor::beginIcm42688() {
  if (!writeRegister(ICM_REG_DEVICE_CONFIG, 0x01)) return false;
  delay(10);
  // Low-noise accel + gyro, +/-4 g, +/-500 dps, both at 100 Hz.
  if (!writeRegister(ICM_REG_PWR_MGMT0, 0x0F)) return false;
  delay(50);
  // FS_SEL=2: +/-500 dps and +/-4 g. ODR=8: 100 Hz.
  if (!writeRegister(ICM_REG_GYRO_CONFIG0, 0x48)) return false;
  if (!writeRegister(ICM_REG_ACCEL_CONFIG0, 0x48)) return false;
  gyroBiasX_ = ICM_GYRO_BIAS_X;
  gyroBiasY_ = ICM_GYRO_BIAS_Y;
  gyroBiasZ_ = ICM_GYRO_BIAS_Z;
  delay(10);
  return whoAmI() == 0x47;
}

bool ImuSensor::beginBmi323() {
  // 100 Hz, normal-power mode, +/-4 g. BMI323 fields are mode[14:12],
  // range[6:4], ODR[3:0]. Gyro uses the same layout with +/-500 dps.
  if (!writeBmiRegister(BMI_REG_ACCEL_CONF, 0x4018)) return false;
  delay(2);
  if (!writeBmiRegister(BMI_REG_GYRO_CONF, 0x4028)) return false;
  delay(50);

  uint8_t id[2] = {};
  if (!readBmiRegisters(BMI_REG_CHIP_ID, id, 1) || id[0] != 0x43)
    return false;
  gyroBiasX_ = BMI_GYRO_BIAS_X;
  gyroBiasY_ = BMI_GYRO_BIAS_Y;
  gyroBiasZ_ = BMI_GYRO_BIAS_Z;
  return true;
}

void ImuSensor::mapAcceleration(float sensorX, float sensorY, float sensorZ,
                                float &instrumentX, float &instrumentY,
                                float &instrumentZ) const {
  if (model_ == Model::BMI323) {
    // Measured prototype-2 mapping: +sensor Y=bow, -sensor Z=starboard,
    // -sensor X=up.
    instrumentX = sensorY;
    instrumentY = -sensorZ;
    instrumentZ = -sensorX;
  } else {
    // Prototype 1: -sensor Y=bow, -sensor Z=starboard, +sensor X=up.
    instrumentX = -sensorY;
    instrumentY = -sensorZ;
    instrumentZ = sensorX;
  }
}

void ImuSensor::mapGyro(float sensorX, float sensorY, float sensorZ,
                        float &instrumentX, float &instrumentY,
                        float &instrumentZ) const {
  // Axial vectors transform with the same proper rotation as acceleration.
  mapAcceleration(sensorX, sensorY, sensorZ,
                  instrumentX, instrumentY, instrumentZ);
}

const char *ImuSensor::modelName() const {
  switch (model_) {
    case Model::MPU6500: return "MPU-6500";
    case Model::ICM42688: return "ICM-42688-P";
    case Model::BMI323: return "BMI323";
    default: return "unknown";
  }
}

bool ImuSensor::read(ImuSample &s, uint64_t monotonicUs) {
  uint8_t d[14];
  if (model_ == Model::BMI323) {
    // Six consecutive 16-bit little-endian words: accel XYZ, gyro XYZ.
    if (!readBmiRegisters(BMI_REG_ACCEL_X, d, 6)) return false;
    auto rawLe = [&](int p) {
      return static_cast<int16_t>((static_cast<uint16_t>(d[p + 1]) << 8) |
                                  d[p]);
    };
    s.monotonicUs = monotonicUs;
    s.axG = (rawLe(0) / 8192.0f - BMI_ACC_BIAS_X) / BMI_ACC_SCALE_X;
    s.ayG = (rawLe(2) / 8192.0f - BMI_ACC_BIAS_Y) / BMI_ACC_SCALE_Y;
    s.azG = (rawLe(4) / 8192.0f - BMI_ACC_BIAS_Z) / BMI_ACC_SCALE_Z;
    s.gxDps = rawLe(6) / 65.5f - gyroBiasX_;
    s.gyDps = rawLe(8) / 65.5f - gyroBiasY_;
    s.gzDps = rawLe(10) / 65.5f - gyroBiasZ_;
    updateAttitude(s, 1.0f / SwimConfig::IMU_RATE_HZ);
    calculateLevelAcceleration(s);
    return true;
  }

  const uint8_t dataReg = model_ == Model::ICM42688
                              ? ICM_REG_ACCEL_DATA_X1 : REG_ACCEL_XOUT_H;
  const size_t dataLength = model_ == Model::ICM42688 ? 12 : sizeof(d);
  if (!readRegisters(dataReg, d, dataLength)) return false;

  const int16_t axRaw = (int16_t)((d[0] << 8) | d[1]);
  const int16_t ayRaw = (int16_t)((d[2] << 8) | d[3]);
  const int16_t azRaw = (int16_t)((d[4] << 8) | d[5]);
  const int gyroOffset = model_ == Model::ICM42688 ? 6 : 8;
  const int16_t gxRaw = (int16_t)((d[gyroOffset] << 8) | d[gyroOffset + 1]);
  const int16_t gyRaw = (int16_t)((d[gyroOffset + 2] << 8) | d[gyroOffset + 3]);
  const int16_t gzRaw = (int16_t)((d[gyroOffset + 4] << 8) | d[gyroOffset + 5]);

  s.monotonicUs = monotonicUs;
  if (model_ == Model::MPU6500) {
    s.axG = (axRaw / 8192.0f - MPU_ACC_BIAS_X) / MPU_ACC_SCALE_X;
    s.ayG = (ayRaw / 8192.0f - MPU_ACC_BIAS_Y) / MPU_ACC_SCALE_Y;
    s.azG = (azRaw / 8192.0f - MPU_ACC_BIAS_Z) / MPU_ACC_SCALE_Z;
  } else {
    s.axG = (axRaw / 8192.0f - ICM_ACC_BIAS_X) / ICM_ACC_SCALE_X;
    s.ayG = (ayRaw / 8192.0f - ICM_ACC_BIAS_Y) / ICM_ACC_SCALE_Y;
    s.azG = (azRaw / 8192.0f - ICM_ACC_BIAS_Z) / ICM_ACC_SCALE_Z;
  }
  s.gxDps = gxRaw / 65.5f - gyroBiasX_;
  s.gyDps = gyRaw / 65.5f - gyroBiasY_;
  s.gzDps = gzRaw / 65.5f - gyroBiasZ_;

  updateAttitude(s, 1.0f / SwimConfig::IMU_RATE_HZ);
  calculateLevelAcceleration(s);
  return true;
}

bool ImuSensor::calibrateStationary() {
  constexpr int SAMPLE_COUNT = 200;
  double sumAx = 0, sumAy = 0, sumAz = 0;
  double sumGx = 0, sumGy = 0, sumGz = 0;
  double sumG2 = 0;
  int received = 0;
  for (int i = 0; i < SAMPLE_COUNT; ++i) {
    uint8_t d[14];
    if (model_ == Model::BMI323) {
      if (readBmiRegisters(BMI_REG_ACCEL_X, d, 6)) {
        auto raw = [&](int p) {
          return static_cast<int16_t>((static_cast<uint16_t>(d[p + 1]) << 8) |
                                      d[p]);
        };
        const float ax = (raw(0) / 8192.0f - BMI_ACC_BIAS_X) / BMI_ACC_SCALE_X;
        const float ay = (raw(2) / 8192.0f - BMI_ACC_BIAS_Y) / BMI_ACC_SCALE_Y;
        const float az = (raw(4) / 8192.0f - BMI_ACC_BIAS_Z) / BMI_ACC_SCALE_Z;
        const float gx = raw(6) / 65.5f;
        const float gy = raw(8) / 65.5f;
        const float gz = raw(10) / 65.5f;
        sumAx += ax; sumAy += ay; sumAz += az;
        sumGx += gx; sumGy += gy; sumGz += gz;
        sumG2 += gx * gx + gy * gy + gz * gz;
        received++;
      }
      delay(10);
      continue;
    }
    const uint8_t dataReg = model_ == Model::ICM42688
                                ? ICM_REG_ACCEL_DATA_X1 : REG_ACCEL_XOUT_H;
    const size_t dataLength = model_ == Model::ICM42688 ? 12 : sizeof(d);
    if (readRegisters(dataReg, d, dataLength)) {
      auto raw = [&](int p) { return (int16_t)((d[p] << 8) | d[p + 1]); };
      const bool mpu = model_ == Model::MPU6500;
      const float axBias = mpu ? MPU_ACC_BIAS_X : ICM_ACC_BIAS_X;
      const float ayBias = mpu ? MPU_ACC_BIAS_Y : ICM_ACC_BIAS_Y;
      const float azBias = mpu ? MPU_ACC_BIAS_Z : ICM_ACC_BIAS_Z;
      const float axScale = mpu ? MPU_ACC_SCALE_X : ICM_ACC_SCALE_X;
      const float ayScale = mpu ? MPU_ACC_SCALE_Y : ICM_ACC_SCALE_Y;
      const float azScale = mpu ? MPU_ACC_SCALE_Z : ICM_ACC_SCALE_Z;
      const float ax = (raw(0) / 8192.0f - axBias) / axScale;
      const float ay = (raw(2) / 8192.0f - ayBias) / ayScale;
      const float az = (raw(4) / 8192.0f - azBias) / azScale;
      const int gyroOffset = model_ == Model::ICM42688 ? 6 : 8;
      const float gx = raw(gyroOffset) / 65.5f;
      const float gy = raw(gyroOffset + 2) / 65.5f;
      const float gz = raw(gyroOffset + 4) / 65.5f;
      sumAx += ax; sumAy += ay; sumAz += az;
      sumGx += gx; sumGy += gy; sumGz += gz;
      sumG2 += gx * gx + gy * gy + gz * gz;
      received++;
    }
    delay(10);
  }
  if (received < SAMPLE_COUNT * 9 / 10) return false;
  const float ax = sumAx / received, ay = sumAy / received, az = sumAz / received;
  const float gx = sumGx / received, gy = sumGy / received, gz = sumGz / received;
  const float accelMagnitude = sqrtf(ax * ax + ay * ay + az * az);
  const float gyroVariance = fmaxf(0.0f, sumG2 / received - (gx * gx + gy * gy + gz * gz));
  if (fabsf(accelMagnitude - 1.0f) > 0.12f || sqrtf(gyroVariance) > 1.5f) {
    Serial.printf("IMU CAL rejected: |a|=%.3f gyroSD=%.3f dps (keep still)\n",
                  accelMagnitude, sqrtf(gyroVariance));
    return false;
  }
  gyroBiasX_ = gx; gyroBiasY_ = gy; gyroBiasZ_ = gz;
  float instrumentAx, instrumentAy, instrumentAz;
  mapAcceleration(ax, ay, az, instrumentAx, instrumentAy, instrumentAz);
  roll_ = atan2f(instrumentAy, instrumentAz);
  pitch_ = atan2f(-instrumentAx,
                  sqrtf(instrumentAy * instrumentAy + instrumentAz * instrumentAz));
  zeroRoll_ = roll_; zeroPitch_ = pitch_;
  attitudeInitialized_ = true;
  Serial.printf("IMU CAL OK: gyro bias=%+.4f,%+.4f,%+.4f dps orientation=%+.1f,%+.1f deg\n",
                gyroBiasX_, gyroBiasY_, gyroBiasZ_, roll_ * RAD_TO_DEG,
                pitch_ * RAD_TO_DEG);
  return true;
}

void ImuSensor::updateAttitude(ImuSample &s, float dt) {
  float ax, ay, az, gx, gy, gz;
  mapAcceleration(s.axG, s.ayG, s.azG, ax, ay, az);
  mapGyro(s.gxDps, s.gyDps, s.gzDps, gx, gy, gz);
  const float rollAcc = atan2f(ay, az);
  const float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az));
  if (!attitudeInitialized_) {
    roll_ = rollAcc;
    pitch_ = pitchAcc;
    attitudeInitialized_ = true;
  } else {
    const float alpha = ATTITUDE_TAU / (ATTITUDE_TAU + dt);
    roll_ = alpha * (roll_ + gx * DEG_TO_RAD * dt) + (1.0f - alpha) * rollAcc;
    pitch_ = alpha * (pitch_ + gy * DEG_TO_RAD * dt) + (1.0f - alpha) * pitchAcc;
  }
  s.rollRad = roll_ - zeroRoll_;
  s.pitchRad = pitch_ - zeroPitch_;
}

void ImuSensor::calculateLevelAcceleration(ImuSample &s) {
  // Gravity compensation must use the absolute attitude. SET ZERO changes
  // only the user-facing relative angles; using them here leaks gravity into
  // the wave channels whenever zero is set with the enclosure tilted.
  const float sr = sinf(roll_);
  const float cr = cosf(roll_);
  const float sp = sinf(pitch_);
  const float cp = cosf(pitch_);
  float ax, ay, az;
  mapAcceleration(s.axG, s.ayG, s.azG, ax, ay, az);
  const float fx = cp * ax + sr * sp * ay + cr * sp * az;
  const float fy = cr * ay - sr * az;
  const float fz = -sp * ax + sr * cp * ay + cr * cp * az;
  s.levelAx = fx * G0;
  s.levelAy = fy * G0;
  s.levelAz = (fz - 1.0f) * G0;
}
