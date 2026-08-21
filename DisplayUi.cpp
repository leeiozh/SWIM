#include "DisplayUi.h"

#include <esp_timer.h>
#include <math.h>
#include "SWIMConfig.h"

namespace {
constexpr uint16_t C_BLACK = 0x0000;
constexpr uint16_t C_WHITE = 0xFFFF;
constexpr uint16_t C_GREEN = 0x07E0;
constexpr uint16_t C_YELLOW = 0xFFE0;
constexpr uint16_t C_RED = 0xF800;
constexpr uint16_t C_CYAN = 0x07FF;
// Secondary text/axes: light enough to remain readable in direct sunlight,
// while still visually distinct from primary white values.
constexpr uint16_t C_GREY = 0xDEFB;
}

DisplayUi::DisplayUi() {
  bus_ = new Arduino_ESP32PAR8Q(
      SwimConfig::LCD_DC, SwimConfig::LCD_CS, SwimConfig::LCD_WR, SwimConfig::LCD_RD,
      SwimConfig::LCD_D0, SwimConfig::LCD_D1, SwimConfig::LCD_D2, SwimConfig::LCD_D3,
      SwimConfig::LCD_D4, SwimConfig::LCD_D5, SwimConfig::LCD_D6, SwimConfig::LCD_D7);
  gfx_ = new Arduino_ST7789(bus_, SwimConfig::LCD_RST, 0, true, 170, 320, 35, 0, 35, 0);
}

void DisplayUi::begin() {
  pinMode(SwimConfig::POWER_ON, OUTPUT);
  digitalWrite(SwimConfig::POWER_ON, HIGH);
  pinMode(SwimConfig::LCD_BL, OUTPUT);
  digitalWrite(SwimConfig::LCD_BL, HIGH);
  pinMode(SwimConfig::BATTERY_ADC, INPUT);
  analogSetPinAttenuation(SwimConfig::BATTERY_ADC, ADC_11db);
  delay(200);
  gfx_->begin();
  gfx_->fillScreen(C_BLACK);
  awake_ = true;
}

void DisplayUi::sleep() {
  if (!awake_) return;
  digitalWrite(SwimConfig::LCD_BL, LOW);
  gfx_->displayOff();
  awake_ = false;
}

void DisplayUi::wake() {
  if (awake_) return;
  gfx_->displayOn();
  digitalWrite(SwimConfig::LCD_BL, HIGH);
  awake_ = true;
  directionalNeedsRedraw_ = true;
}

void DisplayUi::showFatal(const char *title, const char *detail) {
  gfx_->fillScreen(C_BLACK);
  gfx_->setTextSize(2); gfx_->setTextColor(C_RED); gfx_->setCursor(8, 70);
  gfx_->println(title);
  gfx_->setTextSize(1); gfx_->setTextColor(C_WHITE); gfx_->setCursor(8, 110);
  gfx_->println(detail);
}

void DisplayUi::showBoot(const char *stage) {
  gfx_->fillScreen(C_BLACK);
  gfx_->setTextSize(2); gfx_->setTextColor(C_CYAN); gfx_->setCursor(8, 70);
  gfx_->println("SWIM BOOT");
  gfx_->setTextSize(1); gfx_->setTextColor(C_WHITE); gfx_->setCursor(8, 108);
  gfx_->println(stage);
}

void DisplayUi::notify(const char *message, uint16_t color, uint32_t durationMs) {
  strncpy(notification_, message, sizeof(notification_) - 1);
  notification_[sizeof(notification_) - 1] = '\0';
  notificationColor_ = color;
  notificationUntilMs_ = millis() + durationMs;
  directionalNeedsRedraw_ = true;
}

void DisplayUi::nextPage() {
  if (page_ == 2) page_ = 0;
  else if (page_ == 0) page_ = 1;
  else if (page_ == 1) page_ = 3;
  else page_ = 2;
  directionalNeedsRedraw_ = true;
}

void DisplayUi::row(int y, const char *label, const char *value, uint16_t color) {
  gfx_->setTextSize(1);
  gfx_->setTextColor(C_GREY);
  gfx_->setCursor(7, y);
  gfx_->print(label);
  gfx_->setTextColor(color);
  gfx_->setCursor(72, y);
  gfx_->print(value);
}

void DisplayUi::draw(const WaveAnalyzer &waves, const GnssReceiver &gnss,
                     const DataLogger &logger, const WifiPortal &wifi) {
  if (!awake_) return;
  const bool notificationActive = notification_[0] &&
                                  (int32_t)(notificationUntilMs_ - millis()) > 0;
  const bool notificationPendingClear = notification_[0] && !notificationActive;
  if (page_ == 1 && directionalSpectrum_ && !directionalNeedsRedraw_ &&
      lastDirectionalGeneration_ == waves.resultGeneration() &&
      !notificationActive && !notificationPendingClear) return;
  if (page_ == 0) drawWavePage(waves, logger);
  else if (page_ == 1) drawSpectrumPage(waves);
  else if (page_ == 2) drawGnssPage(gnss);
  else drawDataPage(logger, wifi, waves);
  if (page_ == 1 && directionalSpectrum_) {
    lastDirectionalGeneration_ = waves.resultGeneration();
    directionalNeedsRedraw_ = false;
  }
  drawIndicators(logger, gnss, wifi);
  if (notification_[0] && (int32_t)(notificationUntilMs_ - millis()) > 0) {
    gfx_->fillRect(0, 282, 170, 38, C_BLACK);
    gfx_->drawRect(2, 284, 166, 32, notificationColor_);
    gfx_->setTextSize(2); gfx_->setTextColor(notificationColor_);
    gfx_->setCursor(10, 292); gfx_->print(notification_);
  } else if (notification_[0]) {
    notification_[0] = '\0';
  }
}

void DisplayUi::drawIndicators(const DataLogger &logger, const GnssReceiver &gnss,
                               const WifiPortal &wifi) {
  uint32_t millivolts = 0;
  for (int i = 0; i < 8; ++i) millivolts += analogReadMilliVolts(SwimConfig::BATTERY_ADC);
  const float measuredVoltage = millivolts * (2.0f / 8.0f) / 1000.0f;
  float voltage = measuredVoltage + SwimConfig::BATTERY_BASE_CORRECTION_V;
  if (gnss.data().uartAlive)
    voltage += SwimConfig::BATTERY_GNSS_ACTIVE_CORRECTION_V;
  if (wifi.active())
    voltage += SwimConfig::BATTERY_WIFI_ACTIVE_CORRECTION_V;
  const uint16_t batteryColor = voltage >= 3.70f ? C_GREEN :
                                (voltage >= 3.50f ? C_YELLOW : C_RED);
  gfx_->setTextSize(1); gfx_->setTextColor(batteryColor); gfx_->setCursor(138, 8);
  if (voltage > 1.0f) { gfx_->print(voltage, 2); gfx_->print("V"); }
  else gfx_->print("--V");
  if (logger.storageLow()) {
    gfx_->setTextColor(logger.storageFull() ? C_RED : C_YELLOW);
    gfx_->setCursor(146, 22); gfx_->print(logger.storageFull() ? "FULL" : "MEM!");
  }
}

void DisplayUi::drawWavePage(const WaveAnalyzer &waves, const DataLogger &logger) {
  const WaveResults &w = waves.results();
  char text[32];
  gfx_->fillScreen(C_BLACK);
  gfx_->setTextSize(2); gfx_->setTextColor(C_CYAN); gfx_->setCursor(8, 8);
  gfx_->println("SWIM WAVES");
  const bool waitingForData = logger.status() == DataLogger::Status::WAITING_DATA;
  gfx_->setTextSize(1);
  gfx_->setCursor(8, 34);
  if (logger.ok()) {
    gfx_->setTextColor(C_RED); gfx_->print("REC ");
    gfx_->setTextColor(C_CYAN); gfx_->print(logger.fileName());
  } else {
    gfx_->setTextColor(waitingForData ? C_YELLOW : C_RED);
    gfx_->print(waitingForData ? "WAIT DATA" : "NO LOG");
  }

  if (!w.ready) {
    gfx_->setTextSize(2); gfx_->setTextColor(C_YELLOW); gfx_->setCursor(8, 70);
    gfx_->println("COLLECTING");
    const float progress = waves.collectionProgress();
    snprintf(text, sizeof(text), "%.0f %%", progress);
    gfx_->setCursor(8, 105); gfx_->print(text);
    gfx_->drawRect(8, 140, 150, 16, C_WHITE);
    gfx_->fillRect(10, 142, (int)(146 * progress / 100.0f), 12, C_GREEN);
  } else {
    gfx_->setTextColor(C_GREEN); gfx_->setTextSize(2); gfx_->setCursor(8, 64);
    gfx_->print("Hs "); gfx_->print(w.hsM, 2); gfx_->println(" m");
    gfx_->setTextColor(C_GREEN); gfx_->setCursor(8, 96);
    gfx_->print("Tp "); gfx_->print(w.peakPeriodS, 1); gfx_->println(" s");
    gfx_->setTextSize(2); gfx_->setTextColor(C_WHITE); gfx_->setCursor(8, 128);
    gfx_->print("Tm02 "); gfx_->print(w.tm02S, 1); gfx_->println(" s");
    gfx_->setTextColor(C_GREEN); gfx_->setCursor(8, 160);
    gfx_->print("Dp "); gfx_->print(w.directionFromDeg, 0);
    const int degreeX = gfx_->getCursorX() + 3;
    gfx_->drawCircle(degreeX, 161, 3, C_GREEN);
    gfx_->setTextColor(C_WHITE); gfx_->setCursor(8, 190);
    gfx_->print("Dm "); gfx_->print(w.meanDirectionFromDeg, 0);
    const int meanDegreeX = gfx_->getCursorX() + 3;
    gfx_->drawCircle(meanDegreeX, 191, 3, C_WHITE);
    if (w.lowFrequencyEdgePeak) {
      gfx_->setTextSize(1); gfx_->setTextColor(C_YELLOW); gfx_->setCursor(8, 218);
      gfx_->print("LOW-F DRIFT / EDGE PEAK");
    }
  }

  snprintf(text, sizeof(text), "%lu KB", (unsigned long)(logger.fileSize() / 1024));
  row(270, "Log size", text, logger.ok() ? C_GREEN : C_RED);
  gfx_->setTextColor(C_GREY); gfx_->setCursor(7, 303); gfx_->print("LEFT -> spec");
  gfx_->setCursor(92, 303); gfx_->print("RIGHT -> mark");
}

void DisplayUi::drawSpectrumPage(const WaveAnalyzer &waves) {
  if (directionalSpectrum_) {
    drawDirectionalSpectrumPage(waves);
    return;
  }
  const WaveResults &r = waves.results();
  gfx_->fillScreen(C_BLACK);
  gfx_->setTextSize(2); gfx_->setTextColor(C_CYAN); gfx_->setCursor(8, 8);
  gfx_->println("SPECTRUM");
  if (!r.ready) {
    gfx_->setTextColor(C_YELLOW); gfx_->setCursor(8, 70); gfx_->println("NO DATA YET");
  } else {
    gfx_->setTextSize(2); gfx_->setTextColor(C_WHITE); gfx_->setCursor(6, 34);
    gfx_->print("Hs "); gfx_->print(r.hsM, 2); gfx_->print("m");
    gfx_->setCursor(6, 56); gfx_->print("Dp "); gfx_->print(r.directionFromDeg, 0);
    const int directionDegreeX = gfx_->getCursorX() + 3;
    gfx_->drawCircle(directionDegreeX, 57, 3, C_WHITE);
    const int x0 = 24, y0 = 258, width = 136, height = 158;
    constexpr float DISPLAY_FMIN_HZ = 1.0f / 30.0f;
    constexpr float DISPLAY_FMAX_HZ = 1.0f / 3.0f;
    gfx_->drawLine(x0, y0, x0 + width, y0, C_GREY);
    gfx_->drawLine(x0, y0 - height, x0, y0, C_GREY);
    float maxPsd = 0;
    const float *spectrum = waves.elevationSpectrum();
    for (int k = 1; k < waves.spectrumBins(); ++k) {
      const float f = waves.binFrequency(k);
      if (f >= DISPLAY_FMIN_HZ && f <= DISPLAY_FMAX_HZ && spectrum[k] > maxPsd)
        maxPsd = spectrum[k];
    }
    int px = -1, py = -1;
    if (maxPsd > 0) for (int k = 1; k < waves.spectrumBins(); ++k) {
      const float f = waves.binFrequency(k);
      if (f < DISPLAY_FMIN_HZ || f > DISPLAY_FMAX_HZ) continue;
      const int x = x0 + (int)(logf(f / DISPLAY_FMIN_HZ) /
                              logf(DISPLAY_FMAX_HZ / DISPLAY_FMIN_HZ) * width);
      const int y = y0 - (int)(spectrum[k] / maxPsd * height);
      if (px >= 0) gfx_->drawLine(px, py, x, y, C_CYAN);
      px = x; py = y;
    }
    auto frequencyX = [&](float frequency) {
      const float clipped = fmaxf(DISPLAY_FMIN_HZ,
                                   fminf(DISPLAY_FMAX_HZ, frequency));
      return x0 + (int)(logf(clipped / DISPLAY_FMIN_HZ) /
                        logf(DISPLAY_FMAX_HZ / DISPLAY_FMIN_HZ) * width);
    };
    const int peakX = frequencyX(r.peakFrequencyHz);
    const float meanFrequency = r.tm02S > 0 ? 1.0f / r.tm02S : NAN;
    gfx_->drawLine(peakX, y0 - height, peakX, y0, C_YELLOW);
    if (isfinite(meanFrequency)) {
      const int meanX = frequencyX(meanFrequency);
      gfx_->drawLine(meanX, y0 - height, meanX, y0, C_GREEN);
    }
    gfx_->setTextSize(1);
    gfx_->setTextColor(C_YELLOW); gfx_->setCursor(6, 78);
    gfx_->print("fp "); gfx_->print(r.peakPeriodS, 1); gfx_->print("s");
    gfx_->setTextColor(C_GREEN); gfx_->setCursor(82, 78);
    gfx_->print("fm02 "); gfx_->print(r.tm02S, 1); gfx_->print("s");
    gfx_->setTextColor(C_GREY);
    const int periods[] = {30, 20, 15, 10, 8, 6, 4, 3};
    for (int period : periods) {
      const int x = frequencyX(1.0f / period);
      gfx_->drawLine(x, y0, x, y0 + 3, C_GREY);
      gfx_->setCursor(x - (period >= 10 ? 6 : 3), 263);
      gfx_->print(period);
    }
    gfx_->setCursor(72, 276); gfx_->print("period, s");
    gfx_->setCursor(2, 92); gfx_->print(maxPsd, 2);
    gfx_->setCursor(8, 252); gfx_->print("0");
  }
  gfx_->setTextSize(1); gfx_->setTextColor(C_GREY); gfx_->setCursor(7, 303);
  gfx_->print("LEFT -> save");
  gfx_->setCursor(104, 303); gfx_->print("RIGHT -> 2D");
}

void DisplayUi::drawDirectionalSpectrumPage(const WaveAnalyzer &waves) {
  const WaveResults &r = waves.results();
  gfx_->fillScreen(C_BLACK);
  gfx_->setTextSize(2); gfx_->setTextColor(C_CYAN); gfx_->setCursor(8, 8);
  gfx_->println("SPECTRUM");
  if (!r.ready) {
    gfx_->setTextColor(C_YELLOW); gfx_->setCursor(8, 70);
    gfx_->println("NO DATA YET");
  } else {
    constexpr float FMIN = 1.0f / 30.0f;
    constexpr float FMAX = 1.0f / 3.0f;
    constexpr int ANGLES = 90;
    const int cx = 85, cy = 166, innerR = 10, outerR = 62;
    const float *spectrum = waves.elevationSpectrum();
    const float *a1 = waves.directionA1(), *b1 = waves.directionB1();
    const float *a2 = waves.directionA2(), *b2 = waves.directionB2();
    const float df = waves.binFrequency(1);
    auto interpolate = [&](const float *values, float f) {
      const float position = f / df;
      const int k = constrain((int)floorf(position), 1, waves.spectrumBins() - 2);
      const float fraction = position - k;
      return values[k] + fraction * (values[k + 1] - values[k]);
    };
    auto frequencyAtRadius = [&](int radius) {
      const float fraction = (float)(radius - innerR) / (outerR - innerR);
      return FMIN * powf(FMAX / FMIN, fraction);
    };
    float maximum = 0;
    for (int radius = innerR; radius <= outerR; ++radius) {
      const float f = frequencyAtRadius(radius);
      const float sf = interpolate(spectrum, f);
      const float ca1 = interpolate(a1, f), cb1 = interpolate(b1, f);
      const float ca2 = interpolate(a2, f), cb2 = interpolate(b2, f);
      for (int j = 0; j < ANGLES; ++j) {
        const float from = 2.0f * PI * j / ANGLES;
        const float toward = from - PI;
        const float spreading = fmaxf(0.0f, 1.0f + 2.0f *
            (ca1 * cosf(toward) + cb1 * sinf(toward) +
             ca2 * cosf(2.0f * toward) + cb2 * sinf(2.0f * toward)));
        maximum = fmaxf(maximum, sf * spreading);
      }
    }
    if (maximum > 0) {
      for (int radius = innerR; radius <= outerR; ++radius) {
        const float f = frequencyAtRadius(radius);
        const float sf = interpolate(spectrum, f);
        const float ca1 = interpolate(a1, f), cb1 = interpolate(b1, f);
        const float ca2 = interpolate(a2, f), cb2 = interpolate(b2, f);
        for (int j = 0; j < ANGLES; ++j) {
          const float from = 2.0f * PI * j / ANGLES;
          const float toward = from - PI;
          const float spreading = fmaxf(0.0f, 1.0f + 2.0f *
              (ca1 * cosf(toward) + cb1 * sinf(toward) +
               ca2 * cosf(2.0f * toward) + cb2 * sinf(2.0f * toward)));
          const float level = sf * spreading / maximum;
          if (level < 0.025f) continue;
          const uint16_t color = level > 0.70f ? C_RED :
                                 (level > 0.35f ? C_YELLOW :
                                  (level > 0.12f ? C_GREEN : C_CYAN));
          const int x = cx + (int)lroundf(radius * sinf(from));
          const int y = cy - (int)lroundf(radius * cosf(from));
          gfx_->drawPixel(x, y, color);
        }
      }
    }
    for (int period : {30, 15, 8, 4, 3}) {
      const float f = 1.0f / period;
      const int radius = innerR + (int)(logf(f / FMIN) / logf(FMAX / FMIN) *
                                       (outerR - innerR));
      gfx_->drawCircle(cx, cy, radius, C_GREY);
      if (period != 3) {
        gfx_->setTextSize(1); gfx_->setTextColor(C_GREY);
        gfx_->setCursor(cx + 3, cy - radius - 1); gfx_->print(period);
      }
    }
    gfx_->drawLine(cx, cy - outerR - 5, cx, cy + outerR + 5, C_GREY);
    gfx_->drawLine(cx - outerR - 5, cy, cx + outerR + 5, cy, C_GREY);
    gfx_->setTextSize(1); gfx_->setTextColor(C_WHITE);
    gfx_->setCursor(67, 74); gfx_->print("BOW 0");
    gfx_->setCursor(151, 162); gfx_->print("90");
    gfx_->setCursor(70, 232); gfx_->print("180");
    gfx_->setCursor(2, 162); gfx_->print("270");
    gfx_->setTextColor(C_GREY); gfx_->setCursor(cx + 15, cy - 8);
    gfx_->print("T,s");
    gfx_->setCursor(cx + 3, cy - outerR - 10); gfx_->print("3s");
    auto directionMarker = [&](float degrees, int radius, uint16_t color,
                               const char *label) {
      if (!isfinite(degrees)) return;
      const float angle = degrees * DEG_TO_RAD;
      const int x1 = cx + (int)lroundf((outerR + 1) * sinf(angle));
      const int y1 = cy - (int)lroundf((outerR + 1) * cosf(angle));
      const int x2 = cx + (int)lroundf(radius * sinf(angle));
      const int y2 = cy - (int)lroundf(radius * cosf(angle));
      gfx_->drawLine(x1, y1, x2, y2, color);
      gfx_->setTextColor(color);
      gfx_->setCursor(constrain(x2 - 3, 1, 163), constrain(y2 - 4, 42, 270));
      gfx_->print(label);
    };
    directionMarker(r.directionFromDeg, outerR + 8, C_GREEN, "P");
    directionMarker(r.meanDirectionFromDeg, outerR + 14, C_WHITE, "M");
  }
  gfx_->setTextSize(1); gfx_->setTextColor(C_GREY); gfx_->setCursor(7, 303);
  gfx_->print("LEFT -> save");
  gfx_->setCursor(104, 303); gfx_->print("RIGHT -> 1D");
}

void DisplayUi::drawGnssPage(const GnssReceiver &gnss) {
  const GnssData &g = gnss.data();
  char text[32];
  gfx_->fillScreen(C_BLACK);
  gfx_->setTextSize(2); gfx_->setTextColor(C_CYAN); gfx_->setCursor(8, 8);
  gfx_->println("GNSS");
  if (gnss.powerSaveEnabled()) {
    gfx_->setTextSize(1); gfx_->setTextColor(g.uartAlive ? C_GREEN : C_YELLOW);
    gfx_->setCursor(7, 29); gfx_->print(g.uartAlive ? "GNSS ACTIVE" : "GNSS SLEEP");
  }
  if (!g.uartAlive && !gnss.powerSaveEnabled()) {
    const bool neverReceivedData = g.totalBytes == 0;
    const bool startupGraceExpired = millis() >= 15000UL;
    gfx_->setTextSize(2);
    gfx_->setTextColor(neverReceivedData && startupGraceExpired ? C_RED : C_YELLOW);
    gfx_->setCursor(8, 82);
    gfx_->println(neverReceivedData && startupGraceExpired ? "GPS ERROR"
                                                           : "GNSS LOADING");
  } else if (!g.fixValid) {
    gfx_->setTextSize(2); gfx_->setTextColor(C_YELLOW); gfx_->setCursor(8, 82);
    gfx_->println("WAITING FIX");
    snprintf(text, sizeof(text), "%u / %u", g.satellitesUsed, g.signalsVisible);
    row(126, "Sat used/max", text, C_YELLOW);
    snprintf(text, sizeof(text), "%.2f", g.hdop); row(146, "HDOP", text, C_YELLOW);
  } else {
    gfx_->setTextSize(1); gfx_->setTextColor(C_GREY); gfx_->setCursor(7, 40); gfx_->print("UTC TIME");
    if (!g.timeValid) {
      gfx_->setTextColor(C_YELLOW); gfx_->setCursor(92, 40); gfx_->print("DATE WAIT");
    }
    gfx_->setTextSize(2); gfx_->setTextColor(g.timeValid ? C_GREEN : C_YELLOW);
    const uint64_t currentUtcMs = gnss.estimatedUtcMs(esp_timer_get_time());
    char currentUtc[16] = "--:--:--";
    if (currentUtcMs) {
      const time_t currentSeconds = (time_t)(currentUtcMs / 1000ULL);
      struct tm current = {};
      if (gmtime_r(&currentSeconds, &current))
        strftime(currentUtc, sizeof(currentUtc), "%H:%M:%S", &current);
    }
    gfx_->setCursor(7, 54); gfx_->print(currentUtc);
    if (g.timeValid) {
      const time_t seconds = (time_t)(g.utcMs / 1000ULL);
      struct tm utc = {};
      if (gmtime_r(&seconds, &utc)) {
        char date[16];
        strftime(date, sizeof(date), "%d.%m.%Y", &utc);
        gfx_->setTextSize(1); gfx_->setTextColor(C_GREEN);
        gfx_->setCursor(7, 74); gfx_->print(date);
      }
    }
    snprintf(text, sizeof(text), "%u/%u", g.satellitesUsed, g.signalsVisible);
    gfx_->setTextSize(1); gfx_->setTextColor(C_GREY); gfx_->setCursor(7, 90);
    gfx_->print("Sat used/max");
    gfx_->setTextColor(C_GREEN); gfx_->setCursor(90, 90); gfx_->print(text);
    snprintf(text, sizeof(text), "%.2f", g.hdop); row(106, "HDOP", text, C_GREEN);
    gfx_->setTextSize(1); gfx_->setTextColor(C_GREY); gfx_->setCursor(7, 126); gfx_->print("LAT");
    gfx_->setTextSize(2); gfx_->setTextColor(C_WHITE); gfx_->setCursor(7, 140); gfx_->print(g.latitudeDeg, 6);
    gfx_->setTextSize(1); gfx_->setTextColor(C_GREY); gfx_->setCursor(7, 170); gfx_->print("LON");
    gfx_->setTextSize(2); gfx_->setTextColor(C_WHITE); gfx_->setCursor(7, 184); gfx_->print(g.longitudeDeg, 6);
    gfx_->setTextSize(1); gfx_->setTextColor(C_GREY); gfx_->setCursor(7, 214); gfx_->print("SOG / COG");
    gfx_->setTextSize(2); gfx_->setTextColor(C_WHITE); gfx_->setCursor(7, 228);
    gfx_->print(g.sogMps * 1.943844f, 1); gfx_->print("kn "); gfx_->print(g.cogDeg, 0);
    const int degreeX = gfx_->getCursorX() + 3;
    gfx_->drawCircle(degreeX, 221, 3, C_WHITE);
    gfx_->setTextSize(1); gfx_->setTextColor(C_GREY); gfx_->setCursor(7, 258); gfx_->print("ALTITUDE");
    gfx_->setTextSize(2); gfx_->setTextColor(C_GREEN); gfx_->setCursor(7, 272);
    gfx_->print(g.altitudeM, 1); gfx_->print(" m");
  }
  gfx_->setTextSize(1); gfx_->setTextColor(C_GREY); gfx_->setCursor(7, 303);
  gfx_->print("LEFT -> waves");
  gfx_->setCursor(92, 303); gfx_->print("RIGHT -> mark");
}

void DisplayUi::drawDataPage(const DataLogger &logger, const WifiPortal &wifi,
                             const WaveAnalyzer &waves) {
  char text[32];
  gfx_->fillScreen(C_BLACK);
  gfx_->setTextSize(2); gfx_->setTextColor(C_CYAN); gfx_->setCursor(8, 8);
  gfx_->println("SAVE");
  auto saveRow = [&](int y, const char *label, const char *value, uint16_t color) {
    gfx_->setTextSize(1); gfx_->setTextColor(C_GREY); gfx_->setCursor(7, y);
    gfx_->print(label); gfx_->setTextColor(color); gfx_->setCursor(66, y);
    gfx_->print(value);
  };
  saveRow(52, "Logger", logger.statusText(), logger.ok() ? C_GREEN : C_RED);
  const char *shownFile = logger.fileName();
  if (shownFile[0] == '/') ++shownFile;
  saveRow(74, "File", shownFile, C_WHITE);
  snprintf(text, sizeof(text), "%lu KB", (unsigned long)(logger.fileSize() / 1024));
  saveRow(96, "Size", text, C_WHITE);
  if (logger.totalBytes()) snprintf(text, sizeof(text), "%u%% free", logger.freePercent());
  else snprintf(text, sizeof(text), "--");
  saveRow(112, "Storage", text, !logger.totalBytes() || logger.storageFull() ? C_RED :
                                      (logger.storageLow() ? C_YELLOW : C_GREEN));
  saveRow(134, "Wi-Fi", wifi.statusText(), wifi.active() ? C_CYAN :
                                          (wifi.connected() ? C_GREEN : C_YELLOW));
  const String network = wifi.networkName();
  saveRow(156, "Network", network.c_str(), C_WHITE);
  const String ip = wifi.ipText();
  saveRow(178, "IP", ip.c_str(), C_WHITE);
  saveRow(196, "Mode", waves.modeText(), waves.testMode() ? C_YELLOW : C_GREEN);
  if (wifi.active()) {
    gfx_->setTextSize(1); gfx_->setTextColor(C_GREY); gfx_->setCursor(8, 214);
    gfx_->print("Password: "); gfx_->setTextColor(C_WHITE); gfx_->print(WifiPortal::setupPassword());
    gfx_->setTextSize(2); gfx_->setTextColor(C_CYAN); gfx_->setCursor(8, 234);
    gfx_->println("OPEN IP");
    gfx_->setCursor(8, 258); gfx_->println("IN BROWSER");
    gfx_->setTextSize(1); gfx_->setTextColor(C_GREY); gfx_->setCursor(8, 280);
    gfx_->print("RIGHT -> WIFI OFF");
  } else {
    gfx_->setTextSize(1); gfx_->setTextColor(C_CYAN); gfx_->setCursor(8, 220);
    gfx_->println("RIGHT BUTTON turn on wifi");
  }
  gfx_->setTextSize(1); gfx_->setTextColor(C_GREY); gfx_->setCursor(7, 303);
  gfx_->print("LEFT -> GNSS");
}
