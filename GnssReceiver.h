#pragma once

#include <Arduino.h>
#include "SwimTypes.h"

class GnssReceiver {
 public:
  /// Opens the GNSS UART with the configured pins and baud rate.
  void begin(HardwareSerial &serial, int rxPin, int txPin, uint32_t baud);
  /// Consumes pending NMEA bytes and refreshes receiver state.
  void update();
  /// Returns the latest parsed GNSS snapshot.
  const GnssData &data() const { return data_; }
  /// Estimates UTC for a monotonic ESP timer timestamp.
  uint64_t estimatedUtcMs(uint64_t monotonicUs) const;
  /// Requests a 200 ms navigation period (approximately 5 Hz).
  bool configureFiveHz();

 private:
  /// Validates the checksum of one NMEA sentence.
  bool checksumValid(const char *line) const;
  /// Splits and dispatches one validated NMEA sentence.
  void parseLine(char *line);
  /// Parses position and fix-quality fields from GGA.
  void parseGga(char **field, int count);
  /// Parses navigation and UTC fields from RMC.
  void parseRmc(char **field, int count);
  /// Aggregates visible-satellite signal data from GSV.
  void parseGsv(char **field, int count);
  /// Updates the monotonic-to-UTC anchor from NMEA time and date.
  void setUtcAnchor(const char *time, const char *date);
  /// Sends one UBX command frame to the receiver.
  bool sendUbx(uint8_t messageClass, uint8_t messageId,
               const uint8_t *payload, uint16_t payloadLength);

  HardwareSerial *serial_ = nullptr;
  GnssData data_;
  char line_[128] = {};
  size_t length_ = 0;
  bool overflow_ = false;
  uint64_t utcAnchorMs_ = 0;
  uint64_t monotonicAnchorUs_ = 0;
  uint32_t gsvWindowStartedMs_ = 0;
  uint8_t gsvWindowVisible_ = 0;
  uint8_t gsvWindowBestCn0_ = 0;
  bool fiveHzConfigured_ = false;
};
