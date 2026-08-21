#pragma once

#include <Arduino.h>
#include "SwimTypes.h"

class WaveAnalyzer {
 public:
  /// Allocates the wave buffers in PSRAM.
  bool begin();
  /// Selects the short test window or the full field window.
  void setTestMode(bool testMode) { testMode_ = testMode; }
  /// Reports whether the short test window is active.
  bool testMode() const { return testMode_; }
  /// Returns the active mode label for the display.
  const char *modeText() const { return testMode_ ? "TEST" : "FIELD"; }
  /// Returns the active sample-window size.
  int bufferSize() const;
  /// Returns the processing interval for the active mode.
  uint32_t updateIntervalMs() const;
  /// Adds one level-frame acceleration sample.
  void addImuSample(float levelAx, float levelAy, float levelAz);
  /// Reports whether the active window contains enough samples.
  bool readyToProcess() const;
  /// Computes spectra and wave parameters for the current window.
  bool process();
  /// Clears collected samples and calculated results.
  void reset();
  /// Returns the latest calculated wave parameters.
  const WaveResults &results() const { return results_; }
  /// Returns the latest elevation spectrum.
  const float *elevationSpectrum() const { return elevationSpectrum_; }
  /// Returns the first cosine directional coefficient array.
  const float *directionA1() const { return a1_; }
  /// Returns the first sine directional coefficient array.
  const float *directionB1() const { return b1_; }
  /// Returns the second cosine directional coefficient array.
  const float *directionA2() const { return a2_; }
  /// Returns the second sine directional coefficient array.
  const float *directionB2() const { return b2_; }
  /// Returns the number of positive-frequency FFT bins.
  int spectrumBins() const;
  /// Returns a counter incremented after each successful calculation.
  uint32_t resultGeneration() const { return resultGeneration_; }
  /// Converts an FFT-bin index to frequency in hertz.
  float binFrequency(int bin) const;
  /// Returns collection progress from 0 to 100 percent.
  float collectionProgress() const;
  /// Resamples and quantizes spectra for compact log storage.
  void quantizedSpectrum(uint8_t spectrum[32], int8_t a1[32], int8_t b1[32],
                         int8_t a2[32], int8_t b2[32]) const;

 private:
  /// Performs an in-place radix-2 FFT.
  void fft(float *real, float *imag, int n);
  /// Copies a ring-buffer segment into the FFT workspace.
  void loadChannel(float *buffer, int oldest, int start);

  float *waveAx_ = nullptr;
  float *waveAy_ = nullptr;
  float *waveAz_ = nullptr;
  int writeIndex_ = 0;
  int samplesCollected_ = 0;
  uint32_t resultGeneration_ = 0;
  bool testMode_ = true;
  WaveResults results_;

  static float fftReal_[];
  static float fftImag_[];
  static float elevationSpectrum_[];
  static float a1_[];
  static float b1_[];
  float *a2_ = nullptr;
  float *b2_ = nullptr;
};
