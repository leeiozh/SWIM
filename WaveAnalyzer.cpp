#include "WaveAnalyzer.h"

#include <esp32-hal-psram.h>
#include <math.h>
#include "SWIMConfig.h"

namespace {
constexpr int N = SwimConfig::FFT_SIZE;
constexpr int HALF = N / 2;
constexpr float DF = SwimConfig::WAVE_RATE_HZ / N;
}

float WaveAnalyzer::fftReal_[N];
float WaveAnalyzer::fftImag_[N];
float WaveAnalyzer::elevationSpectrum_[HALF];
float WaveAnalyzer::a1_[HALF];
float WaveAnalyzer::b1_[HALF];

bool WaveAnalyzer::begin() {
  waveAx_ = (float *)ps_malloc(SwimConfig::MAX_WAVE_BUFFER_SIZE * sizeof(float));
  waveAy_ = (float *)ps_malloc(SwimConfig::MAX_WAVE_BUFFER_SIZE * sizeof(float));
  waveAz_ = (float *)ps_malloc(SwimConfig::MAX_WAVE_BUFFER_SIZE * sizeof(float));
  a2_ = (float *)ps_malloc(HALF * sizeof(float));
  b2_ = (float *)ps_malloc(HALF * sizeof(float));
  return waveAx_ && waveAy_ && waveAz_ && a2_ && b2_;
}

int WaveAnalyzer::bufferSize() const {
  return testMode_ ? SwimConfig::TEST_WAVE_BUFFER_SIZE
                   : SwimConfig::FIELD_WAVE_BUFFER_SIZE;
}

uint32_t WaveAnalyzer::updateIntervalMs() const {
  return testMode_ ? SwimConfig::TEST_WAVE_UPDATE_MS
                   : SwimConfig::FIELD_WAVE_UPDATE_MS;
}

void WaveAnalyzer::addImuSample(float x, float y, float z) {
  if (!waveAx_) return;
  waveAx_[writeIndex_] = x;
  waveAy_[writeIndex_] = y;
  waveAz_[writeIndex_] = z;
  writeIndex_ = (writeIndex_ + 1) % bufferSize();
  if (samplesCollected_ < bufferSize()) samplesCollected_++;
  results_.sampleCount = samplesCollected_;
}

bool WaveAnalyzer::readyToProcess() const {
  return samplesCollected_ >= bufferSize();
}

void WaveAnalyzer::reset() {
  writeIndex_ = 0;
  samplesCollected_ = 0;
  results_ = WaveResults();
  resultGeneration_++;
  memset(elevationSpectrum_, 0, sizeof(elevationSpectrum_));
  memset(a1_, 0, sizeof(a1_)); memset(b1_, 0, sizeof(b1_));
  if (a2_) memset(a2_, 0, HALF * sizeof(float));
  if (b2_) memset(b2_, 0, HALF * sizeof(float));
}

float WaveAnalyzer::collectionProgress() const {
  return 100.0f * samplesCollected_ / bufferSize();
}

int WaveAnalyzer::spectrumBins() const { return HALF; }
float WaveAnalyzer::binFrequency(int bin) const { return bin * DF; }

void WaveAnalyzer::quantizedSpectrum(uint8_t spectrum[32], int8_t qa1[32],
                                     int8_t qb1[32], int8_t qa2[32],
                                     int8_t qb2[32]) const {
  float maximum = 0;
  const int lastBin = (int)floorf(SwimConfig::FMAX_HZ / DF);
  for (int i = 0; i < 32; ++i) {
    const float f = SwimConfig::FMAX_HZ * i / 31.0f;
    const int k = constrain((int)lroundf(f / DF), 0, lastBin);
    maximum = fmaxf(maximum, elevationSpectrum_[k]);
  }
  for (int i = 0; i < 32; ++i) {
    const float f = SwimConfig::FMAX_HZ * i / 31.0f;
    const int k = constrain((int)lroundf(f / DF), 0, lastBin);
    spectrum[i] = maximum > 0 ? (uint8_t)constrain(
        (int)lroundf(elevationSpectrum_[k] / maximum * 255.0f), 0, 255) : 0;
    qa1[i] = (int8_t)constrain((int)lroundf(a1_[k] * 127.0f), -127, 127);
    qb1[i] = (int8_t)constrain((int)lroundf(b1_[k] * 127.0f), -127, 127);
    qa2[i] = (int8_t)constrain((int)lroundf(a2_[k] * 127.0f), -127, 127);
    qb2[i] = (int8_t)constrain((int)lroundf(b2_[k] * 127.0f), -127, 127);
  }
}

void WaveAnalyzer::fft(float *real, float *imag, int n) {
  int j = 0;
  for (int i = 1; i < n; ++i) {
    int bit = n >> 1;
    while (j & bit) { j ^= bit; bit >>= 1; }
    j ^= bit;
    if (i < j) {
      float t = real[i]; real[i] = real[j]; real[j] = t;
      t = imag[i]; imag[i] = imag[j]; imag[j] = t;
    }
  }
  for (int len = 2; len <= n; len <<= 1) {
    const float angle = -2.0f * PI / len;
    const float wlenR = cosf(angle);
    const float wlenI = sinf(angle);
    for (int i = 0; i < n; i += len) {
      float wr = 1.0f, wi = 0.0f;
      for (int k = 0; k < len / 2; ++k) {
        const int u = i + k;
        const int v = u + len / 2;
        const float vr = real[v] * wr - imag[v] * wi;
        const float vi = real[v] * wi + imag[v] * wr;
        const float ur = real[u], ui = imag[u];
        real[u] = ur + vr; imag[u] = ui + vi;
        real[v] = ur - vr; imag[v] = ui - vi;
        const float newWr = wr * wlenR - wi * wlenI;
        wi = wr * wlenI + wi * wlenR;
        wr = newWr;
      }
    }
  }
}

void WaveAnalyzer::loadChannel(float *buffer, int oldest, int start) {
  // Remove a least-squares straight line before the Hann window. A constant
  // mean alone leaves slow bias/tilt drift which becomes enormous after the
  // omega^-4 conversion from acceleration to elevation.
  double sumY = 0;
  double sumIY = 0;
  const double sumI = (double)N * (N - 1) / 2.0;
  const double sumII = (double)(N - 1) * N * (2.0 * N - 1) / 6.0;
  for (int i = 0; i < N; ++i) {
    const float y = buffer[(oldest + start + i) % bufferSize()];
    sumY += y;
    sumIY += (double)i * y;
  }
  const double denom = N * sumII - sumI * sumI;
  const double slope = denom != 0 ? (N * sumIY - sumI * sumY) / denom : 0;
  const double intercept = (sumY - slope * sumI) / N;
  for (int i = 0; i < N; ++i) {
    const int idx = (oldest + start + i) % bufferSize();
    const float w = 0.5f - 0.5f * cosf(2.0f * PI * i / (N - 1));
    fftReal_[i] = (buffer[idx] - (intercept + slope * i)) * w;
    fftImag_[i] = 0;
  }
  fft(fftReal_, fftImag_, N);
}

bool WaveAnalyzer::process() {
  if (!readyToProcess()) return false;

  static float Sxx[HALF], Syy[HALF], Szz[HALF];
  static float Cxy[HALF], Qzx[HALF], Qzy[HALF];
  static float Xr[HALF], Xi[HALF], Yr[HALF], Yi[HALF], Zr[HALF], Zi[HALF];
  memset(Sxx, 0, sizeof(Sxx)); memset(Syy, 0, sizeof(Syy));
  memset(Szz, 0, sizeof(Szz)); memset(Cxy, 0, sizeof(Cxy));
  memset(Qzx, 0, sizeof(Qzx)); memset(Qzy, 0, sizeof(Qzy));

  double windowPower = 0;
  for (int i = 0; i < N; ++i) {
    const float w = 0.5f - 0.5f * cosf(2.0f * PI * i / (N - 1));
    windowPower += w * w;
  }

  const int oldest = writeIndex_;
  int segments = 0;
  for (int start = 0; start + N <= bufferSize();
       start += SwimConfig::FFT_STEP) {
    loadChannel(waveAx_, oldest, start);
    for (int k = 0; k < HALF; ++k) { Xr[k] = fftReal_[k]; Xi[k] = fftImag_[k]; }
    loadChannel(waveAy_, oldest, start);
    for (int k = 0; k < HALF; ++k) { Yr[k] = fftReal_[k]; Yi[k] = fftImag_[k]; }
    loadChannel(waveAz_, oldest, start);
    for (int k = 0; k < HALF; ++k) { Zr[k] = fftReal_[k]; Zi[k] = fftImag_[k]; }

    const float norm = 2.0f / (SwimConfig::WAVE_RATE_HZ * windowPower);
    for (int k = 1; k < HALF; ++k) {
      Sxx[k] += norm * (Xr[k] * Xr[k] + Xi[k] * Xi[k]);
      Syy[k] += norm * (Yr[k] * Yr[k] + Yi[k] * Yi[k]);
      Szz[k] += norm * (Zr[k] * Zr[k] + Zi[k] * Zi[k]);
      Cxy[k] += norm * (Xr[k] * Yr[k] + Xi[k] * Yi[k]);
      Qzx[k] += norm * (Zi[k] * Xr[k] - Zr[k] * Xi[k]);
      Qzy[k] += norm * (Zi[k] * Yr[k] - Zr[k] * Yi[k]);
    }
    segments++;
  }
  if (!segments) return false;

  double m0 = 0, m2 = 0;
  double meanDirectionA = 0, meanDirectionB = 0;
  float peakEnergy = 0;
  int peakBin = -1;
  for (int k = 1; k < HALF; ++k) {
    Sxx[k] /= segments; Syy[k] /= segments; Szz[k] /= segments;
    Cxy[k] /= segments; Qzx[k] /= segments; Qzy[k] /= segments;
    const float f = k * DF;
    const float omega2 = sq(2.0f * PI * f);
    elevationSpectrum_[k] = Szz[k] / (omega2 * omega2);
    const float horizontal = Sxx[k] + Syy[k];
    const float firstDenom = sqrtf(Szz[k] * horizontal);
    a1_[k] = firstDenom > 1e-12f ? Qzx[k] / firstDenom : 0;
    b1_[k] = firstDenom > 1e-12f ? Qzy[k] / firstDenom : 0;
    a2_[k] = horizontal > 1e-12f ? (Sxx[k] - Syy[k]) / horizontal : 0;
    b2_[k] = horizontal > 1e-12f ? 2.0f * Cxy[k] / horizontal : 0;
    if (f >= SwimConfig::PARAM_FMIN_HZ && f <= SwimConfig::FMAX_HZ) {
      const float s = elevationSpectrum_[k];
      m0 += s * DF;
      m2 += f * f * s * DF;
      meanDirectionA += a1_[k] * s * DF;
      meanDirectionB += b1_[k] * s * DF;
      if (s > peakEnergy) { peakEnergy = s; peakBin = k; }
    }
  }

  results_.hsM = m0 > 0 ? 4.0f * sqrtf(m0) : NAN;
  results_.tm02S = m2 > 0 ? sqrtf(m0 / m2) : NAN;
  if (m0 > 0 && (fabs(meanDirectionA) > 1e-12 || fabs(meanDirectionB) > 1e-12)) {
    float meanFrom = atan2f(meanDirectionB, meanDirectionA) * RAD_TO_DEG + 180.0f;
    while (meanFrom < 0) meanFrom += 360;
    while (meanFrom >= 360) meanFrom -= 360;
    results_.meanDirectionFromDeg = meanFrom;
  } else {
    results_.meanDirectionFromDeg = NAN;
  }
  const int firstParameterBin = (int)ceilf(SwimConfig::PARAM_FMIN_HZ / DF);
  results_.lowFrequencyEdgePeak = peakBin > 0 && peakBin <= firstParameterBin + 1;
  if (peakBin > 0) {
    results_.peakFrequencyHz = peakBin * DF;
    results_.peakPeriodS = 1.0f / results_.peakFrequencyHz;
    float from = atan2f(b1_[peakBin], a1_[peakBin]) * RAD_TO_DEG + 180.0f;
    while (from < 0) from += 360;
    while (from >= 360) from -= 360;
    results_.directionFromDeg = from;
  }
  results_.ready = true;
  resultGeneration_++;
  return true;
}
