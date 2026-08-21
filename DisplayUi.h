#pragma once

#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include "DataLogger.h"
#include "GnssReceiver.h"
#include "WaveAnalyzer.h"
#include "WifiPortal.h"

class DisplayUi {
 public:
  /// Creates the display bus and panel objects.
  DisplayUi();
  /// Powers and initializes the display hardware.
  void begin();
  /// Shows a non-recoverable startup error.
  void showFatal(const char *title, const char *detail);
  /// Shows the current startup stage.
  void showBoot(const char *stage);
  /// Displays a temporary status notification.
  void notify(const char *message, uint16_t color, uint32_t durationMs = 2500);
  /// Advances to the next main page.
  void nextPage();
  /// Switches between one- and two-dimensional spectrum views.
  void toggleSpectrumMode() {
    directionalSpectrum_ = !directionalSpectrum_;
    directionalNeedsRedraw_ = true;
  }
  /// Renders the active page and status indicators.
  void draw(const WaveAnalyzer &waves, const GnssReceiver &gnss,
            const DataLogger &logger, const WifiPortal &wifi);
  /// Turns off the panel and backlight.
  void sleep();
  /// Turns the panel and backlight back on.
  void wake();
  /// Reports whether the panel is awake.
  bool awake() const { return awake_; }
  /// Returns the active page index.
  uint8_t page() const { return page_; }

 private:
  /// Draws the primary wave-parameter page.
  void drawWavePage(const WaveAnalyzer &waves, const DataLogger &logger);
  /// Draws the active spectrum representation.
  void drawSpectrumPage(const WaveAnalyzer &waves);
  /// Draws the polar directional spectrum.
  void drawDirectionalSpectrumPage(const WaveAnalyzer &waves);
  /// Draws GNSS time, fix and navigation data.
  void drawGnssPage(const GnssReceiver &gnss);
  /// Draws logger, mode and Wi-Fi controls.
  void drawDataPage(const DataLogger &logger, const WifiPortal &wifi,
                    const WaveAnalyzer &waves);
  /// Draws battery and storage indicators shared by all pages.
  void drawIndicators(const DataLogger &logger, const GnssReceiver &gnss,
                      const WifiPortal &wifi);
  /// Draws one compact label/value row.
  void row(int y, const char *label, const char *value, uint16_t color);

  Arduino_DataBus *bus_;
  Arduino_GFX *gfx_;
  // During combined field testing, start on GNSS; LEFT cycles all pages.
  uint8_t page_ = 2;
  char notification_[24] = {};
  uint16_t notificationColor_ = 0xFFFF;
  uint32_t notificationUntilMs_ = 0;
  bool awake_ = true;
  bool directionalSpectrum_ = false;
  bool directionalNeedsRedraw_ = true;
  uint32_t lastDirectionalGeneration_ = UINT32_MAX;
};
