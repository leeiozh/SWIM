#include <Arduino.h>
#include <Wire.h>
#include <esp32-hal-psram.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <esp_heap_caps.h>
#include <Preferences.h>

#include "SWIMConfig.h"
#include "DataLogger.h"
#include "DisplayUi.h"
#include "GnssReceiver.h"
#include "ImuSensor.h"
#include "WaveAnalyzer.h"
#include "WifiPortal.h"

TwoWire imuWire(0);
HardwareSerial gpsSerial(1);

ImuSensor imu;
GnssReceiver gnss;
WaveAnalyzer waves;
DataLogger logger;
DisplayUi display;
WifiPortal wifi;

ImuSample latestImu;
uint64_t nextImuUs = 0;
uint32_t lastWaveProcessMs = 0;
uint32_t lastDisplayMs = 0;
uint64_t lastSummaryUtcMinute = UINT64_MAX;
bool timeSyncWasLogged = false;
bool pendingSetZeroEvent = false;
uint64_t pendingSetZeroMonotonicUs = 0;
uint64_t pendingSetZeroUtcMs = 0;
uint32_t lastUtcNameAttemptMs = 0;

bool lastLeft = HIGH;
bool lastRight = HIGH;
uint32_t lastLeftChangeMs = 0;
uint32_t lastRightChangeMs = 0;
uint32_t rightPressedMs = 0;
uint32_t lastShortRightMs = 0;
uint8_t shortRightClicks = 0;
uint32_t lastDisplayInteractionMs = 0;
bool ignoreButtonsUntilReleased = false;
uint32_t imuHealthSamples = 0;
uint32_t imuMissedDeadlines = 0;
uint32_t imuReadErrors = 0;
uint32_t imuMaxLatenessUs = 0;
uint32_t lastImuHealthMs = 0;
float waveSumAx = 0, waveSumAy = 0, waveSumAz = 0;
float waveSumRoll = 0, waveSumPitch = 0;
uint8_t waveDecimationCount = 0;
int activeImuSda = SwimConfig::I2C_SDA;
int activeImuScl = SwimConfig::I2C_SCL;

/// Starts the IMU bus without changing the established prototype wiring. If
/// no device answers on GPIO1/2, also tests accidentally crossed SDA/SCL and
/// the older T-Display-S3 external-I2C pair used by early SWIM assemblies.
bool beginImuWithPinFallback() {
  struct PinPair { int sda; int scl; const char *label; };
  const PinPair candidates[] = {
      {SwimConfig::I2C_SDA, SwimConfig::I2C_SCL, "primary"},
      {SwimConfig::I2C_ALT_SDA, SwimConfig::I2C_ALT_SCL, "prototype-2"},
      {SwimConfig::I2C_SCL, SwimConfig::I2C_SDA, "primary-swapped"},
      {43, 44, "legacy"},
      {44, 43, "legacy-swapped"},
  };
  for (const PinPair &pins : candidates) {
    imuWire.end();
    delay(50);
    Serial.printf("IMU BUS: trying %s SDA=GPIO%d SCL=GPIO%d\n",
                  pins.label, pins.sda, pins.scl);
    if (!imuWire.begin(pins.sda, pins.scl, 100000)) {
      Serial.println("IMU BUS: Wire.begin failed");
      continue;
    }
    delay(150);
    if (imu.begin(imuWire)) {
      activeImuSda = pins.sda;
      activeImuScl = pins.scl;
      return true;
    }
    // A responding device with an unsupported identity is useful evidence.
    // Keep this bus active and do not hide it by probing unrelated pins.
    if (imu.address() != 0) return false;
  }
  return false;
}

const char *resetReasonText(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON: return "power-on";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic/exception";
    case ESP_RST_INT_WDT: return "interrupt watchdog";
    case ESP_RST_TASK_WDT: return "task watchdog";
    case ESP_RST_WDT: return "other watchdog";
    case ESP_RST_BROWNOUT: return "brownout / supply dip";
    default: return "other";
  }
}

/// Writes the current GNSS, wave and spectrum snapshot as one summary.
bool writeCurrentSummary(uint64_t monotonicUs, uint64_t utcMs) {
  GnssData snapshot = gnss.data();
  snapshot.utcMs = utcMs;
  uint8_t spectrum[32];
  int8_t a1[32], b1[32], a2[32], b2[32];
  waves.quantizedSpectrum(spectrum, a1, b1, a2, b2);
  return logger.writeSummary(snapshot, waves.results(), spectrum, a1, b1, a2, b2,
                             monotonicUs, snapshot.utcMs);
}

/// Handles the page-specific action of one short right-button press.
void saveMark(uint64_t monotonicUs) {
  if (display.page() == 1) {
    display.toggleSpectrumMode();
    display.draw(waves, gnss, logger, wifi);
    return;
  }
  if (display.page() == 3) {
    const bool enabled = wifi.toggle();
    display.notify(enabled ? "WIFI ON" : "WIFI OFF", enabled ? 0x07FF : 0xDEFB);
    display.draw(waves, gnss, logger, wifi);
    return;
  }
  if (logger.disabled()) {
    display.notify("NO STORAGE", 0xFFE0);
    display.draw(waves, gnss, logger, wifi);
    Serial.println("MARK skipped: no storage backend is available");
    return;
  }
  const bool saved = logger.writeEvent(SwimConfig::EVENT_MARK, monotonicUs,
                                       gnss.estimatedUtcMs(monotonicUs));
  display.notify(saved ? "MARK SAVED" : "MARK ERROR", saved ? 0x07E0 : 0xF800);
  display.draw(waves, gnss, logger, wifi);
  Serial.println(saved ? "MARK event written and flushed" : "MARK event write failed");
}

/// Shows a fatal startup error and stops normal execution.
void haltWithError(const char *title, const char *detail) {
  Serial.print(title); Serial.print(": "); Serial.println(detail);
  display.showFatal(title, detail);
  while (true) delay(1000);
}

/// Persists the opposite acquisition mode and restarts the controller.
void switchModeAndRestart() {
  const bool nextTestMode = !waves.testMode();
  Preferences preferences;
  preferences.begin("swim", false);
  preferences.putBool("test_mode", nextTestMode);
  preferences.end();
  logger.flush();
  wifi.stop();
  display.notify(nextTestMode ? "MODE TEST" : "MODE FIELD", 0x07FF, 1200);
  display.draw(waves, gnss, logger, wifi);
  Serial.printf("MODE: switching to %s and restarting\n",
                nextTestMode ? "TEST" : "FIELD");
  delay(1200);
  ESP.restart();
}

/// Initializes hardware, storage and all runtime subsystems.
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(SwimConfig::IMU_POWER_PIN, OUTPUT);
  digitalWrite(SwimConfig::IMU_POWER_PIN, HIGH);
  delay(SwimConfig::IMU_POWER_STABILIZE_MS);
  Serial.printf("IMU POWER: GPIO%d=HIGH (3.3 V logic supply)\n",
                SwimConfig::IMU_POWER_PIN);

  const esp_reset_reason_t resetReason = esp_reset_reason();
  Serial.printf("BOOT: reset reason=%d (%s), heap=%u internal=%u largest=%u\n",
                (int)resetReason, resetReasonText(resetReason), ESP.getFreeHeap(),
                heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));

  setCpuFrequencyMhz(SwimConfig::CPU_FREQUENCY_MHZ);

  pinMode(SwimConfig::BUTTON_LEFT, INPUT_PULLUP);
  pinMode(SwimConfig::BUTTON_RIGHT, INPUT_PULLUP);
  display.begin();
  display.showBoot("PSRAM / wave buffers");

  if (!psramFound()) psramInit();
  Preferences modePreferences;
  modePreferences.begin("swim", true);
  waves.setTestMode(modePreferences.getBool(
      "test_mode", SwimConfig::DEFAULT_TEST_MODE));
  modePreferences.end();
  if (!waves.begin()) haltWithError("PSRAM ERROR", "Wave buffers unavailable");
  Serial.printf("MEMORY: wave buffers in PSRAM, heap=%u internal=%u largest=%u, PSRAM free=%u\n",
                ESP.getFreeHeap(), heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                ESP.getFreePsram());

  display.showBoot("IMU");
  if (!beginImuWithPinFallback())
    haltWithError("IMU ERROR", "Check pin scan / IDs in Serial");

  display.showBoot("GNSS");
  gnss.begin(gpsSerial, SwimConfig::GPS_RX_PIN, SwimConfig::GPS_TX_PIN,
             SwimConfig::GPS_BAUD);
  gnss.configureFiveHz();

  // Prefer microSD, but preserve the original internal-FFat logger when the
  // second prototype has no card fitted.
  display.showBoot("LOGGER SD / FFat");
  logger.setWaveBufferSize(waves.bufferSize());
  if (!logger.begin())
    Serial.println("LOGGER: both storage backends failed; measurements continue without logging");
  display.showBoot("WIFI OFF");
  wifi.begin(logger);

  nextImuUs = esp_timer_get_time();
  lastDisplayInteractionMs = millis();
  display.draw(waves, gnss, logger, wifi);

  Serial.println("SWIM modular logger started");
  Serial.printf("IMU I2C: SDA=GPIO%d SCL=GPIO%d address=0x%02X model=%s\n",
                activeImuSda, activeImuScl,
                imu.address(), imu.modelName());
  Serial.printf("GPS UART: GPS TX -> GPIO%d, GPS RX <- GPIO%d, %lu baud\n",
                SwimConfig::GPS_RX_PIN, SwimConfig::GPS_TX_PIN,
                (unsigned long)SwimConfig::GPS_BAUD);
  Serial.printf("microSD SPI: CS=%d SCK=%d MISO=%d MOSI=%d\n",
                SwimConfig::SD_CS, SwimConfig::SD_SCK,
                SwimConfig::SD_MISO, SwimConfig::SD_MOSI);
  Serial.printf("Storage backend: %s\n", logger.backendName());
  Serial.printf("IMU %.0f Hz, wave %.0f Hz, buffer %d samples\n",
                SwimConfig::IMU_RATE_HZ, SwimConfig::WAVE_RATE_HZ,
                waves.bufferSize());
  Serial.printf("Mode: %s, wave update %lu s\n", waves.modeText(),
                (unsigned long)(waves.updateIntervalMs() / 1000UL));
  Serial.printf("CPU: %u MHz\n", getCpuFrequencyMhz());
  Serial.printf("Log: %s (%s)\n", logger.fileName(), logger.statusText());
}

/// Debounces buttons and dispatches click, hold and wake gestures.
void handleButtons(uint64_t monotonicUs) {
  const uint32_t now = millis();
  const bool left = digitalRead(SwimConfig::BUTTON_LEFT);
  const bool right = digitalRead(SwimConfig::BUTTON_RIGHT);

  // Count every physical interaction, including a held button and clicks
  // which later become part of a multi-click gesture.
  if (display.awake() && (left == LOW || right == LOW))
    lastDisplayInteractionMs = now;

  if (!display.awake()) {
    if (left == LOW || right == LOW) {
      display.wake();
      display.draw(waves, gnss, logger, wifi);
      lastDisplayInteractionMs = now;
      shortRightClicks = 0;
      ignoreButtonsUntilReleased = true;
      lastLeft = left;
      lastRight = right;
    }
    return;
  }

  // The button which woke the screen has no normal action. Wait for all
  // buttons to be released before accepting a new press.
  if (ignoreButtonsUntilReleased) {
    lastLeft = left;
    lastRight = right;
    if (left == HIGH && right == HIGH) {
      ignoreButtonsUntilReleased = false;
      lastLeftChangeMs = now;
      lastRightChangeMs = now;
    }
    return;
  }

  if (left != lastLeft && now - lastLeftChangeMs >= 40) {
    lastLeftChangeMs = now;
    lastLeft = left;
    if (left == LOW) {
      lastDisplayInteractionMs = now;
      display.nextPage();
      display.draw(waves, gnss, logger, wifi);
    }
  }

  if (right != lastRight && now - lastRightChangeMs >= 40) {
    lastRightChangeMs = now;
    lastRight = right;
    if (right == LOW) {
      lastDisplayInteractionMs = now;
      rightPressedMs = now;
    } else {
      const uint32_t heldMs = now - rightPressedMs;
      if (display.page() == 3 && heldMs >= SwimConfig::MODE_SWITCH_HOLD_MS) {
        shortRightClicks = 0;
        switchModeAndRestart();
        return;
      }
      // A press must be short enough to be an intentional click. MARK is
      // delayed until the multi-click window closes, otherwise the first two
      // clicks of SET ZERO would also create unwanted marks.
      if (heldMs < SwimConfig::BUTTON_MULTI_CLICK_MS) {
        if (now - lastShortRightMs > SwimConfig::BUTTON_MULTI_CLICK_MS)
          shortRightClicks = 0;
        lastShortRightMs = now;
        shortRightClicks++;
        if (shortRightClicks >= 3) {
          shortRightClicks = 0;
          display.notify(SwimConfig::VERTICAL_INSTALLATION ? "HOLD VERTICAL" : "HOLD STILL",
                         0xFFE0, 3000);
          display.draw(waves, gnss, logger, wifi);
          const bool calibrated = imu.calibrateStationary();
          nextImuUs = esp_timer_get_time() + SwimConfig::IMU_PERIOD_US;
          if (calibrated) {
            waves.reset();
            waveSumAx = waveSumAy = waveSumAz = 0;
            waveSumRoll = waveSumPitch = 0;
            waveDecimationCount = 0;
          }
          if (calibrated) {
            pendingSetZeroEvent = true;
            pendingSetZeroMonotonicUs = esp_timer_get_time();
            pendingSetZeroUtcMs = gnss.estimatedUtcMs(pendingSetZeroMonotonicUs);
            if (logger.ok() && logger.writeEvent(
                    SwimConfig::EVENT_SET_ZERO, pendingSetZeroMonotonicUs,
                    pendingSetZeroUtcMs)) pendingSetZeroEvent = false;
          }
          display.notify(calibrated ? "ZERO SET" : "CAL FAILED",
                         calibrated ? 0xFFE0 : 0xF800);
          display.draw(waves, gnss, logger, wifi);
          Serial.println(!calibrated ? "SET ZERO rejected: sensor moved during calibration"
                                     : (pendingSetZeroEvent
                                            ? "SET ZERO calibrated vertically; event queued until log opens"
                                            : "SET ZERO calibrated vertically and logged"));
        }
      }
    }
  }
  if (shortRightClicks && now - lastShortRightMs > SwimConfig::BUTTON_MULTI_CLICK_MS) {
    if (shortRightClicks == 1) saveMark(monotonicUs);
    shortRightClicks = 0;
  }
}

/// Runs acquisition, analysis, logging, UI and power scheduling.
void loop() {
  const uint64_t monotonicUs = esp_timer_get_time();
  gnss.update();

  if ((int64_t)(monotonicUs - nextImuUs) >= 0) {
    // Skip deadlines which are already irrecoverably late instead of taking a
    // burst of near-identical catch-up samples after FFT or flash activity.
    const uint64_t latenessUs = monotonicUs - nextImuUs;
    if (latenessUs > imuMaxLatenessUs)
      imuMaxLatenessUs = latenessUs > UINT32_MAX ? UINT32_MAX : (uint32_t)latenessUs;
    const uint32_t missed = (uint32_t)(latenessUs / SwimConfig::IMU_PERIOD_US);
    imuMissedDeadlines += missed;
    nextImuUs += (uint64_t)(missed + 1) * SwimConfig::IMU_PERIOD_US;
    if (imu.read(latestImu, monotonicUs)) {
      imuHealthSamples++;
      waveSumAx += latestImu.levelAx; waveSumAy += latestImu.levelAy;
      waveSumAz += latestImu.levelAz; waveSumRoll += latestImu.rollRad;
      waveSumPitch += latestImu.pitchRad;
      if (++waveDecimationCount == SwimConfig::WAVE_DECIMATION) {
        const float scale = 1.0f / SwimConfig::WAVE_DECIMATION;
        waves.addImuSample(waveSumAx * scale, waveSumAy * scale,
                           waveSumAz * scale, waveSumRoll * scale,
                           waveSumPitch * scale);
        waveSumAx = waveSumAy = waveSumAz = 0;
        waveSumRoll = waveSumPitch = 0;
        waveDecimationCount = 0;
      }
    } else imuReadErrors++;
  }

  const GnssData &g = gnss.data();
  if (logger.ok() && g.timeValid && !logger.utcNamed() &&
      millis() - lastUtcNameAttemptMs >= 5000) {
    lastUtcNameAttemptMs = millis();
    logger.applyUtcFileName(gnss.estimatedUtcMs(monotonicUs));
  }
  if (logger.ok() && g.timeValid && !timeSyncWasLogged) {
    timeSyncWasLogged = logger.writeTimeSync(
        monotonicUs, gnss.estimatedUtcMs(monotonicUs));
  }
  if (logger.ok() && pendingSetZeroEvent && logger.writeEvent(
          SwimConfig::EVENT_SET_ZERO, pendingSetZeroMonotonicUs,
          pendingSetZeroUtcMs)) {
    pendingSetZeroEvent = false;
    Serial.println("Queued SET ZERO event written");
  }

  // Store one summary at each UTC minute boundary. The record UTC timestamp
  // is exactly HH:MM:00.000; events and time-sync records retain full time.
  const uint64_t estimatedUtcMs = gnss.estimatedUtcMs(monotonicUs);
  if (logger.ok() && waves.results().ready && estimatedUtcMs) {
    const uint64_t utcMinute = estimatedUtcMs / SwimConfig::SUMMARY_LOG_INTERVAL_MS;
    const uint64_t withinMinuteMs = estimatedUtcMs % SwimConfig::SUMMARY_LOG_INTERVAL_MS;
    if (utcMinute != lastSummaryUtcMinute && withinMinuteMs < 2000ULL) {
      const uint64_t boundaryUtcMs = utcMinute * SwimConfig::SUMMARY_LOG_INTERVAL_MS;
      const uint64_t boundaryMonotonicUs = monotonicUs - withinMinuteMs * 1000ULL;
      if (writeCurrentSummary(boundaryMonotonicUs, boundaryUtcMs)) {
        lastSummaryUtcMinute = utcMinute;
        logger.flush();
        logger.checkStorage();
      }
    }
  }

  if (waves.readyToProcess() &&
      (!waves.results().ready || millis() - lastWaveProcessMs >= waves.updateIntervalMs())) {
    lastWaveProcessMs = millis();
    if (waves.process()) {
      Serial.printf("Waves: Hs=%.3f m Tp=%.2f s Tm02=%.2f s Dp=%.1f deg Dm=%.1f deg mean roll=%+.1f pitch=%+.1f deg\n",
                    waves.results().hsM, waves.results().peakPeriodS,
                    waves.results().tm02S, waves.results().directionFromDeg,
                    waves.results().meanDirectionFromDeg,
                    waves.results().meanRollDeg, waves.results().meanPitchDeg);
      if (logger.status() == DataLogger::Status::WAITING_DATA) {
        const uint64_t utcMs = gnss.estimatedUtcMs(monotonicUs);
        if (logger.startSession(utcMs)) {
          lastSummaryUtcMinute = UINT64_MAX;
        }
      }
    }
    gnss.update();
  }

  logger.update();
  wifi.update();
  handleButtons(monotonicUs);

  if (display.awake() &&
      millis() - lastDisplayInteractionMs >= SwimConfig::DISPLAY_SLEEP_MS) {
    display.sleep();
    shortRightClicks = 0;
    Serial.println("DISPLAY: asleep; measurements and logging continue");
  }

  if (millis() - lastDisplayMs >= SwimConfig::DISPLAY_UPDATE_MS) {
    lastDisplayMs = millis();
    display.draw(waves, gnss, logger, wifi);
  }

  // Report aggregate timing health once per minute. This replaces the removed
  // raw IMU diagnostics and is the acceptance test for light sleep.
  const uint32_t nowMs = millis();
  if (nowMs - lastImuHealthMs >= 60000UL) {
    const uint32_t elapsedMs = lastImuHealthMs ? nowMs - lastImuHealthMs : nowMs;
    const float actualHz = elapsedMs ? imuHealthSamples * 1000.0f / elapsedMs : 0.0f;
    Serial.printf("POWER/IMU: %.2f Hz, missed=%lu, read_err=%lu, max_late=%lu us\n",
                  actualHz, (unsigned long)imuMissedDeadlines,
                  (unsigned long)imuReadErrors, (unsigned long)imuMaxLatenessUs);
    lastImuHealthMs = nowMs;
    imuHealthSamples = 0;
    imuMissedDeadlines = 0;
    imuReadErrors = 0;
    imuMaxLatenessUs = 0;
  }

  // Yield to the RTOS while waiting for the next IMU sample. Real ESP32 light
  // sleep is deliberately disabled pending power and timing comparisons.
  const int64_t idleUs = (int64_t)(nextImuUs - esp_timer_get_time());
  if (idleUs >= 2000) delay(1);
}
