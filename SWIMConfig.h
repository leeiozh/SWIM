#pragma once

#include <Arduino.h>

namespace SwimConfig {

// Runtime-selectable acquisition windows. TEST is ~82 s at 50 Hz; FIELD is
// 10 minutes. Buffers are allocated once for the larger mode in PSRAM.
constexpr bool DEFAULT_TEST_MODE = true;

// LilyGO T-Display-S3
constexpr int LCD_BL = 38;
constexpr int LCD_RST = 5;
constexpr int LCD_CS = 6;
constexpr int LCD_DC = 7;
constexpr int LCD_WR = 8;
constexpr int LCD_RD = 9;
constexpr int LCD_D0 = 39;
constexpr int LCD_D1 = 40;
constexpr int LCD_D2 = 41;
constexpr int LCD_D3 = 42;
constexpr int LCD_D4 = 45;
constexpr int LCD_D5 = 46;
constexpr int LCD_D6 = 47;
constexpr int LCD_D7 = 48;
constexpr int POWER_ON = 15;
constexpr int BUTTON_LEFT = 0;
constexpr int BUTTON_RIGHT = 14;
constexpr int BATTERY_ADC = 4;
// Empirical BAT_ADC correction against a multimeter on this assembled unit.
// The divider/ADC reads low, with additional droop under radio load.
constexpr float BATTERY_BASE_CORRECTION_V = 0.20f;
constexpr float BATTERY_GNSS_ACTIVE_CORRECTION_V = 0.04f;
constexpr float BATTERY_WIFI_ACTIVE_CORRECTION_V = 0.05f;

// GY-91 / MPU-6500
// The assembled instrument is operated and zeroed vertically: physical MPU
// +X reads approximately +1 g when stationary. ImuSensor maps that +X to the
// earth/instrument vertical before attitude and wave calculations.
constexpr bool VERTICAL_INSTALLATION = true;
// On-board 4-pin Qwiic/STEMMA connector (I2C, not SPI).
// Four-pin connector marked 43/44 beside USB:
// black = SDA -> GPIO43, red = SCL -> GPIO44,
// blue = GND, yellow = 3V3.
constexpr int I2C_SDA = 43;
constexpr int I2C_SCL = 44;
constexpr uint8_t MPU_ADDRESS = 0x68;
constexpr float IMU_RATE_HZ = 50.0f;
constexpr uint32_t IMU_PERIOD_US = 20000UL;

// GY-GPS6MU2 / u-blox M10 UART.
// GPS TX -> GPIO1 (ESP RX), GPS RX <- GPIO2 (ESP TX).
constexpr int GPS_RX_PIN = 1;
constexpr int GPS_TX_PIN = 2;
constexpr uint32_t GPS_BAUD = 9600;

// Wave processing retains the previous 100 -> 50 Hz decimation.
constexpr float WAVE_RATE_HZ = 50.0f;
constexpr int FFT_SIZE = 4096;
constexpr int FFT_STEP = FFT_SIZE / 2;
constexpr int TEST_WAVE_BUFFER_SIZE = 4096;
constexpr int FIELD_WAVE_BUFFER_SIZE = 30000;
constexpr int MAX_WAVE_BUFFER_SIZE = FIELD_WAVE_BUFFER_SIZE;
constexpr uint32_t TEST_WAVE_UPDATE_MS = 20000UL;
constexpr uint32_t FIELD_WAVE_UPDATE_MS = 60000UL;
constexpr float PARAM_FMIN_HZ = 0.04f;
constexpr float FMAX_HZ = 0.40f;

constexpr uint32_t DISPLAY_UPDATE_MS = 2000UL;
constexpr uint32_t DISPLAY_SLEEP_MS = 60000UL;
constexpr uint32_t LOG_FLUSH_MS = 1000UL;
constexpr uint32_t SUMMARY_LOG_INTERVAL_MS = 60000UL;
constexpr uint32_t BUTTON_MULTI_CLICK_MS = 900UL;
constexpr uint32_t MODE_SWITCH_HOLD_MS = 3000UL;
constexpr uint32_t CPU_FREQUENCY_MHZ = 80;

// Event codes stored in the binary journal and exported by swim_log_to_csv.py.
constexpr uint32_t EVENT_MARK = 1;
constexpr uint32_t EVENT_SET_ZERO = 2;
constexpr uint8_t STORAGE_WARNING_PERCENT = 10;
constexpr size_t STORAGE_STOP_FREE_BYTES = 16 * 1024;

}  // namespace SwimConfig
