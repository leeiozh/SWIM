#include "WaveAnalyzer.h"

#include <esp32-hal-psram.h>
#include <math.h>
#include "SWIMConfig.h"

namespace {
constexpr int N = SwimConfig::FFT_SIZE;
constexpr int HALF = N / 2;
constexpr float DF = SwimConfig::WAVE_RATE_HZ / N;
}

bool WaveAnalyzer::begin() {
  waveAx_ = (float *)ps_malloc(SwimConfig::MAX_WAVE_BUFFER_SIZE * sizeof(float));
  waveAy_ = (float *)ps_malloc(SwimConfig::MAX_WAVE_BUFFER_SIZE * sizeof(float));
  waveAz_ = (float *)ps_malloc(SwimConfig::MAX_WAVE_BUFFER_SIZE * sizeof(float));
  roll_ = (float *)ps_malloc(SwimConfig::MAX_WAVE_BUFFER_SIZE * sizeof(float));
  pitch_ = (float *)ps_malloc(SwimConfig::MAX_WAVE_BUFFER_SIZE * sizeof(float));
  fftReal_ = (float *)ps_malloc(N * sizeof(float));
  fftImag_ = (float *)ps_malloc(N * sizeof(float));
  elevationSpectrum_ = (float *)ps_malloc(HALF * sizeof(float));
  a1_ = (float *)ps_malloc(HALF * sizeof(float));
  b1_ = (float *)ps_malloc(HALF * sizeof(float));
  a2_ = (float *)ps_malloc(HALF * sizeof(float));
  b2_ = (float *)ps_malloc(HALF * sizeof(float));
  float **workspaces[] = {&sxx_, &syy_, &szz_, &cxy_, &qzx_, &qzy_,
                          &xr_, &xi_, &yr_, &yi_, &zr_, &zi_};
  for (float **workspace : workspaces)
    *workspace = (float *)ps_malloc(HALF * sizeof(float));
  const bool allocated = waveAx_ && waveAy_ && waveAz_ && roll_ && pitch_ &&
                         fftReal_ && fftImag_ && elevationSpectrum_ && a1_ &&
                         b1_ && a2_ && b2_ && sxx_ && syy_ && szz_ && cxy_ &&
                         qzx_ && qzy_ && xr_ && xi_ && yr_ && yi_ && zr_ && zi_;
  if (allocated) {
    memset(elevationSpectrum_, 0, HALF * sizeof(float));
    memset(a1_, 0, HALF * sizeof(float)); memset(b1_, 0, HALF * sizeof(float));
    memset(a2_, 0, HALF * sizeof(float)); memset(b2_, 0, HALF * sizeof(float));
  }
  return allocated;
}

int WaveAnalyzer::bufferSize() const {
  return testMode_ ? SwimConfig::TEST_WAVE_BUFFER_SIZE
                   : SwimConfig::FIELD_WAVE_BUFFER_SIZE;
}

uint32_t WaveAnalyzer::updateIntervalMs() const {
  return testMode_ ? SwimConfig::TEST_WAVE_UPDATE_MS
                   : SwimConfig::FIELD_WAVE_UPDATE_MS;
}

void WaveAnalyzer::addImuSample(float x, float y, float z,
                                float rollRad, float pitchRad) {
  if (!waveAx_) return;
  waveAx_[writeIndex_] = x;
  waveAy_[writeIndex_] = y;
  waveAz_[writeIndex_] = z;
  roll_[writeIndex_] = rollRad;
  pitch_[writeIndex_] = pitchRad;
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
  memset(elevationSpectrum_, 0, HALF * sizeof(float));
  memset(a1_, 0, HALF * sizeof(float)); memset(b1_, 0, HALF * sizeof(float));
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

  memset(sxx_, 0, HALF * sizeof(float)); memset(syy_, 0, HALF * sizeof(float));
  memset(szz_, 0, HALF * sizeof(float)); memset(cxy_, 0, HALF * sizeof(float));
  memset(qzx_, 0, HALF * sizeof(float)); memset(qzy_, 0, HALF * sizeof(float));

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
    for (int k = 0; k < HALF; ++k) { xr_[k] = fftReal_[k]; xi_[k] = fftImag_[k]; }
    loadChannel(waveAy_, oldest, start);
    for (int k = 0; k < HALF; ++k) { yr_[k] = fftReal_[k]; yi_[k] = fftImag_[k]; }
    loadChannel(waveAz_, oldest, start);
    for (int k = 0; k < HALF; ++k) { zr_[k] = fftReal_[k]; zi_[k] = fftImag_[k]; }

    const float norm = 2.0f / (SwimConfig::WAVE_RATE_HZ * windowPower);
    for (int k = 1; k < HALF; ++k) {
      sxx_[k] += norm * (xr_[k] * xr_[k] + xi_[k] * xi_[k]);
      syy_[k] += norm * (yr_[k] * yr_[k] + yi_[k] * yi_[k]);
      szz_[k] += norm * (zr_[k] * zr_[k] + zi_[k] * zi_[k]);
      cxy_[k] += norm * (xr_[k] * yr_[k] + xi_[k] * yi_[k]);
      qzx_[k] += norm * (zi_[k] * xr_[k] - zr_[k] * xi_[k]);
      qzy_[k] += norm * (zi_[k] * yr_[k] - zr_[k] * yi_[k]);
    }
    segments++;
  }
  if (!segments) return false;

  // Circular mean avoids a false zero when roll crosses -180/+180 degrees.
  double sumRollSin = 0, sumRollCos = 0, sumPitch = 0;
  for (int i = 0; i < bufferSize(); ++i) {
    sumRollSin += sinf(roll_[i]);
    sumRollCos += cosf(roll_[i]);
    sumPitch += pitch_[i];
  }
  results_.meanRollDeg = atan2(sumRollSin, sumRollCos) * RAD_TO_DEG;
  results_.meanPitchDeg = (sumPitch / bufferSize()) * RAD_TO_DEG;

  double m0 = 0, m2 = 0;
  double meanDirectionA = 0, meanDirectionB = 0;
  float rawPeakEnergy = 0;
  int rawPeakBin = -1;
  for (int k = 1; k < HALF; ++k) {
    sxx_[k] /= segments; syy_[k] /= segments; szz_[k] /= segments;
    cxy_[k] /= segments; qzx_[k] /= segments; qzy_[k] /= segments;
    const float f = k * DF;
    const float omega2 = sq(2.0f * PI * f);
    // Fourth-order high-pass power response suppresses attitude/bias drift
    // which otherwise explodes after the acceleration-to-elevation f^-4 step.
    const float ratio = f / SwimConfig::DRIFT_HIGHPASS_HZ;
    const float ratio4 = ratio * ratio * ratio * ratio;
    const float highPassPower = (ratio4 * ratio4) / (1.0f + ratio4 * ratio4);
    elevationSpectrum_[k] = szz_[k] / (omega2 * omega2) * highPassPower;
    const float horizontal = sxx_[k] + syy_[k];
    const float firstDenom = sqrtf(szz_[k] * horizontal);
    a1_[k] = firstDenom > 1e-12f ? qzx_[k] / firstDenom : 0;
    b1_[k] = firstDenom > 1e-12f ? qzy_[k] / firstDenom : 0;
    a2_[k] = horizontal > 1e-12f ? (sxx_[k] - syy_[k]) / horizontal : 0;
    b2_[k] = horizontal > 1e-12f ? 2.0f * cxy_[k] / horizontal : 0;
    if (f >= SwimConfig::PARAM_FMIN_HZ && f <= SwimConfig::FMAX_HZ) {
      const float s = elevationSpectrum_[k];
      m0 += s * DF;
      m2 += f * f * s * DF;
      meanDirectionA += a1_[k] * s * DF;
      meanDirectionB += b1_[k] * s * DF;
      if (s > rawPeakEnergy) { rawPeakEnergy = s; rawPeakBin = k; }
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
  // Tp uses a slightly safer band and a three-bin smoother. Hs/Tm02 still use
  // the wider parameter band above. This prevents the old 20.48 s edge bin
  // from masking a real wave peak while retaining an explicit QC flag.
  const int firstPeakBin = (int)ceilf(SwimConfig::PEAK_FMIN_HZ / DF);
  const int lastPeakBin = min(HALF - 2, (int)floorf(SwimConfig::FMAX_HZ / DF));
  auto smoothed = [&](int k) {
    return 0.25f * elevationSpectrum_[k - 1] +
           0.50f * elevationSpectrum_[k] +
           0.25f * elevationSpectrum_[k + 1];
  };
  float peakEnergy = 0;
  int peakBin = -1;
  for (int k = max(1, firstPeakBin); k <= lastPeakBin; ++k) {
    const float energy = smoothed(k);
    if (energy > peakEnergy) { peakEnergy = energy; peakBin = k; }
  }
  results_.lowFrequencyEdgePeak = rawPeakBin > 0 && rawPeakBin < firstPeakBin;
  if (peakBin > 0) {
    const float left = smoothed(peakBin - 1);
    const float center = smoothed(peakBin);
    const float right = smoothed(peakBin + 1);
    const float denominator = left - 2.0f * center + right;
    float offset = fabsf(denominator) > 1e-20f
                       ? constrain(0.5f * (left - right) / denominator,
                                   -0.5f, 0.5f)
                       : 0.0f;
    if (peakBin == firstPeakBin && offset < 0) offset = 0;
    results_.peakFrequencyHz = (peakBin + offset) * DF;
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
