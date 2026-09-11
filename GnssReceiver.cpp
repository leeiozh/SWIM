#include "GnssReceiver.h"

#include <esp_timer.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace {
/// Converts one hexadecimal character to its numeric value.
int hexDigit(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

/// Converts an NMEA degrees/minutes coordinate to signed decimal degrees.
double coordinate(const char *value, const char *hemisphere) {
  if (!value || !*value || !hemisphere || !*hemisphere) return NAN;
  const double raw = atof(value);
  const int degrees = (int)(raw / 100.0);
  double result = degrees + (raw - degrees * 100.0) / 60.0;
  if (*hemisphere == 'S' || *hemisphere == 'W') result = -result;
  return result;
}

// Days since 1970-01-01. Howard Hinnant's civil-calendar transform.
int64_t daysFromCivil(int year, unsigned month, unsigned day) {
  year -= month <= 2;
  const int era = (year >= 0 ? year : year - 399) / 400;
  const unsigned yoe = (unsigned)(year - era * 400);
  const unsigned doy = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097LL + (int)doe - 719468LL;
}
}  // namespace

void GnssReceiver::begin(HardwareSerial &serial, int rxPin, int txPin, uint32_t baud) {
  serial_ = &serial;
  serial_->begin(baud, SERIAL_8N1, rxPin, txPin);
  gsvWindowStartedMs_ = millis();
}

bool GnssReceiver::sendUbx(uint8_t messageClass, uint8_t messageId,
                           const uint8_t *payload, uint16_t payloadLength) {
  if (!serial_) return false;
  uint8_t ckA = 0, ckB = 0;
  auto addChecksum = [&](uint8_t value) { ckA += value; ckB += ckA; };
  const uint8_t header[] = {0xB5, 0x62, messageClass, messageId,
                            (uint8_t)(payloadLength & 0xFF),
                            (uint8_t)(payloadLength >> 8)};
  for (size_t i = 2; i < sizeof(header); ++i) addChecksum(header[i]);
  for (uint16_t i = 0; i < payloadLength; ++i) addChecksum(payload[i]);
  size_t written = serial_->write(header, sizeof(header));
  written += serial_->write(payload, payloadLength);
  const uint8_t checksum[] = {ckA, ckB};
  written += serial_->write(checksum, sizeof(checksum));
  serial_->flush();
  return written == sizeof(header) + payloadLength + sizeof(checksum);
}

bool GnssReceiver::configureFiveHz() {
  if (fiveHzConfigured_) return true;
  // UBX-CFG-RATE: 200 ms measurement period, one navigation solution per
  // measurement, UTC time reference. This packet is supported by NEO-M8N.
  const uint8_t payload[] = {0xC8, 0x00, 0x01, 0x00, 0x00, 0x00};
  fiveHzConfigured_ = sendUbx(0x06, 0x08, payload, sizeof(payload));
  Serial.println(fiveHzConfigured_ ? "GNSS: 5 Hz navigation rate requested"
                                   : "GNSS: failed to request 5 Hz rate");
  return fiveHzConfigured_;
}

bool GnssReceiver::checksumValid(const char *line) const {
  if (!line || line[0] != '$') return false;
  const char *star = strchr(line, '*');
  if (!star || strlen(star) < 3) return false;
  uint8_t sum = 0;
  for (const char *p = line + 1; p < star; ++p) sum ^= (uint8_t)*p;
  const int hi = hexDigit(star[1]);
  const int lo = hexDigit(star[2]);
  return hi >= 0 && lo >= 0 && sum == (uint8_t)((hi << 4) | lo);
}

void GnssReceiver::update() {
  if (!serial_) return;
  while (serial_->available()) {
    const char c = (char)serial_->read();
    data_.totalBytes++;
    data_.lastByteMs = millis();
    data_.uartAlive = true;
    if (c == '\n') {
      if (length_ && line_[length_ - 1] == '\r') line_[--length_] = '\0';
      if (!overflow_ && checksumValid(line_)) {
        data_.goodSentences++;
        parseLine(line_);
      } else {
        data_.badSentences++;
      }
      length_ = 0;
      line_[0] = '\0';
      overflow_ = false;
    } else if (!overflow_) {
      if (length_ < sizeof(line_) - 1) {
        line_[length_++] = c;
        line_[length_] = '\0';
      } else {
        overflow_ = true;
      }
    }
  }

  const uint32_t now = millis();
  data_.uartAlive = data_.lastByteMs && now - data_.lastByteMs < 2000;
  if (now - gsvWindowStartedMs_ >= 5000) {
    data_.signalsVisible = gsvWindowVisible_;
    data_.bestCn0DbHz = gsvWindowBestCn0_;
    gsvWindowVisible_ = 0;
    gsvWindowBestCn0_ = 0;
    gsvWindowStartedMs_ = now;
  }
}

void GnssReceiver::parseLine(char *line) {
  char *star = strchr(line, '*');
  if (star) *star = '\0';
  char *field[24] = {};
  int count = 0;
  char *cursor = line;
  while (count < 24) {
    field[count++] = cursor;
    char *comma = strchr(cursor, ',');
    if (!comma) break;
    *comma = '\0';
    cursor = comma + 1;
  }
  if (!count || strlen(field[0]) < 6) return;
  const char *type = field[0] + 3;
  if (!strcmp(type, "GGA")) parseGga(field, count);
  else if (!strcmp(type, "RMC")) parseRmc(field, count);
  else if (!strcmp(type, "GSV")) parseGsv(field, count);
}

void GnssReceiver::parseGga(char **f, int n) {
  if (n < 10) return;
  if (strlen(f[1]) >= 6) {
    snprintf(data_.utcText, sizeof(data_.utcText), "%c%c:%c%c:%c%c",
             f[1][0], f[1][1], f[1][2], f[1][3], f[1][4], f[1][5]);
  }
  data_.fixQuality = atoi(f[6]);
  data_.satellitesUsed = atoi(f[7]);
  data_.hdop = *f[8] ? atof(f[8]) : 99.99f;
  data_.fixValid = data_.fixQuality > 0;
  if (data_.fixValid) {
    data_.latitudeDeg = coordinate(f[2], f[3]);
    data_.longitudeDeg = coordinate(f[4], f[5]);
    data_.altitudeM = *f[9] ? atof(f[9]) : NAN;
  } else {
    data_.latitudeDeg = data_.longitudeDeg = data_.altitudeM = NAN;
  }
}

void GnssReceiver::parseRmc(char **f, int n) {
  if (n < 10) return;
  const bool navigationValid = f[2][0] == 'A';
  data_.sogMps = navigationValid && *f[7] ? atof(f[7]) * 0.514444f : NAN;
  data_.cogDeg = navigationValid && *f[8] ? atof(f[8]) : NAN;
  if (strlen(f[1]) >= 6) {
    snprintf(data_.utcText, sizeof(data_.utcText), "%c%c:%c%c:%c%c",
             f[1][0], f[1][1], f[1][2], f[1][3], f[1][4], f[1][5]);
  }
  if (strlen(f[1]) >= 6 && strlen(f[9]) == 6) setUtcAnchor(f[1], f[9]);
}

void GnssReceiver::parseGsv(char **f, int n) {
  if (n < 4) return;
  const int visible = atoi(f[3]);
  if (visible > gsvWindowVisible_) gsvWindowVisible_ = visible;
  for (int i = 7; i < n; i += 4) {
    const int cn0 = atoi(f[i]);
    if (cn0 > gsvWindowBestCn0_) gsvWindowBestCn0_ = cn0;
  }
}

void GnssReceiver::setUtcAnchor(const char *time, const char *date) {
  const int hour = (time[0] - '0') * 10 + time[1] - '0';
  const int minute = (time[2] - '0') * 10 + time[3] - '0';
  const int second = (time[4] - '0') * 10 + time[5] - '0';
  const int day = (date[0] - '0') * 10 + date[1] - '0';
  const int month = (date[2] - '0') * 10 + date[3] - '0';
  const int yy = (date[4] - '0') * 10 + date[5] - '0';
  const int year = yy >= 80 ? 1900 + yy : 2000 + yy;
  const int64_t days = daysFromCivil(year, month, day);
  utcAnchorMs_ = (uint64_t)(days * 86400LL + hour * 3600 + minute * 60 + second) * 1000ULL;
  const char *dot = strchr(time, '.');
  if (dot) {
    int factor = 100;
    for (++dot; *dot && factor > 0; ++dot, factor /= 10) {
      if (*dot >= '0' && *dot <= '9') utcAnchorMs_ += (*dot - '0') * factor;
    }
  }
  monotonicAnchorUs_ = esp_timer_get_time();
  data_.utcMs = utcAnchorMs_;
  data_.timeValid = true;
}

uint64_t GnssReceiver::estimatedUtcMs(uint64_t monotonicUs) const {
  if (!data_.timeValid || monotonicUs < monotonicAnchorUs_) return 0;
  return utcAnchorMs_ + (monotonicUs - monotonicAnchorUs_) / 1000ULL;
}
