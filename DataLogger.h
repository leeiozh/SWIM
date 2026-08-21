#pragma once

#include <Arduino.h>
#include <FS.h>
#include "SwimTypes.h"
#include "SWIMConfig.h"

class DataLogger {
 public:
  enum class Status : uint8_t {
    NOT_STARTED,
    WAITING_DATA,
    OK,
    MOUNT_FAILED,
    OPEN_FAILED,
    WRITE_FAILED,
    STORAGE_FULL,
  };

  /// Mounts FFat and prepares the logger without creating a session file.
  bool begin();
  /// Stores the active wave-window size for the next file header.
  void setWaveBufferSize(uint32_t samples) { waveBufferSize_ = samples; }
  /// Opens a new log after the first wave result is available.
  bool startSession(uint64_t utcMs);
  /// Flushes buffered records at the configured interval.
  void update();
  /// Writes one combined minute summary and directional spectrum.
  bool writeSummary(const GnssData &gnss, const WaveResults &waves,
                    const uint8_t spectrum[32], const int8_t a1[32],
                    const int8_t b1[32], const int8_t a2[32],
                    const int8_t b2[32], uint64_t monotonicUs, uint64_t utcMs);
  /// Writes the current monotonic-to-UTC time anchor.
  bool writeTimeSync(uint64_t monotonicUs, uint64_t utcMs);
  /// Writes and immediately flushes a user event.
  bool writeEvent(uint32_t code, uint64_t monotonicUs, uint64_t utcMs);
  /// Reports whether the session file is ready for writes.
  bool ok() const { return ready_ && file_; }
  /// Returns the active file name or the current placeholder text.
  const char *fileName() const { return fileName_; }
  /// Returns the current session size in bytes.
  size_t fileSize() const { return file_ ? file_.size() : 0; }
  /// Returns the logger state.
  Status status() const { return status_; }
  /// Returns a compact display label for the logger state.
  const char *statusText() const;
  /// Returns the FFat capacity captured at mount time.
  size_t totalBytes() const { return totalBytes_; }
  /// Returns the latest measured free-space percentage.
  uint8_t freePercent() const;
  /// Reports whether the low-space warning threshold was reached.
  bool storageLow() const;
  /// Reports whether logging stopped because storage is full.
  bool storageFull() const { return status_ == Status::STORAGE_FULL; }
  /// Flushes buffered data and the active file.
  bool flush();
  /// Renames a numeric session file once valid UTC becomes available.
  bool applyUtcFileName(uint64_t utcMs);
  /// Refreshes free-space state and stops writes at the safety limit.
  void checkStorage();
  /// Reports whether the active file already has a UTC-based name.
  bool utcNamed() const { return utcNamed_; }

 private:
  /// Appends one typed binary record to the write buffer.
  bool writeRecord(uint8_t type, uint64_t monotonicUs, uint64_t utcMs,
                   const void *payload, uint16_t payloadSize);
  /// Flushes the in-memory write buffer to the active file.
  bool flushBuffer();
  /// Writes the fixed binary header for a new session.
  bool writeFileHeader();

  File file_;
  bool ready_ = false;
  Status status_ = Status::NOT_STARTED;
  uint32_t lastFlushMs_ = 0;
  char fileName_[40] = "not-open";
  static constexpr size_t WRITE_BUFFER_SIZE = 8192;
  uint8_t writeBuffer_[WRITE_BUFFER_SIZE] = {};
  size_t bufferedBytes_ = 0;
  size_t totalBytes_ = 0;
  size_t freeBytes_ = 0;
  bool utcNamed_ = false;
  uint32_t waveBufferSize_ = SwimConfig::TEST_WAVE_BUFFER_SIZE;
};
