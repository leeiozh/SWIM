#pragma once

#include <Arduino.h>

struct ImuSample {
  uint64_t monotonicUs = 0;
  float axG = 0;
  float ayG = 0;
  float azG = 0;
  float gxDps = 0;
  float gyDps = 0;
  float gzDps = 0;
  float rollRad = 0;
  float pitchRad = 0;
  float levelAx = 0;
  float levelAy = 0;
  float levelAz = 0;
};

struct GnssData {
  bool uartAlive = false;
  bool timeValid = false;
  bool fixValid = false;
  uint8_t fixQuality = 0;
  uint8_t satellitesUsed = 0;
  uint8_t signalsVisible = 0;
  uint8_t bestCn0DbHz = 0;
  float hdop = 99.99f;
  double latitudeDeg = NAN;
  double longitudeDeg = NAN;
  float altitudeM = NAN;
  float sogMps = NAN;
  float cogDeg = NAN;
  uint64_t utcMs = 0;
  char utcText[20] = "--:--:--";
  uint32_t goodSentences = 0;
  uint32_t badSentences = 0;
  uint32_t totalBytes = 0;
  uint32_t lastByteMs = 0;
};

struct WaveResults {
  bool ready = false;
  bool lowFrequencyEdgePeak = false;
  float hsM = NAN;
  float peakFrequencyHz = NAN;
  float peakPeriodS = NAN;
  float tm02S = NAN;
  float directionFromDeg = NAN;
  float meanDirectionFromDeg = NAN;
  uint32_t sampleCount = 0;

};
