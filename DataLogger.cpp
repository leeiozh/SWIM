#include "DataLogger.h"

#include <FFat.h>
#include <Preferences.h>
#include <string.h>
#include <time.h>
#include "SWIMConfig.h"

namespace {
enum RecordType : uint8_t {
  RECORD_TIME_SYNC = 4,
  RECORD_EVENT = 5,
  RECORD_SUMMARY = 6,
};

struct __attribute__((packed)) FileHeader {
  char magic[8];
  uint16_t formatVersion;
  uint16_t headerSize;
  uint32_t imuRateMilliHz;
  uint32_t waveRateMilliHz;
  uint32_t fftSize;
  uint32_t waveBufferSize;
  char firmware[16];
};

struct __attribute__((packed)) RecordHeader {
  uint8_t type;
  uint8_t version;
  uint16_t payloadSize;
  uint64_t monotonicUs;
  uint64_t utcMs;  // Zero until GNSS date/time synchronization is available.
};

struct __attribute__((packed)) SummaryPayload {
  uint8_t flags;
  uint8_t reserved[3];
  double latitudeDeg, longitudeDeg;
  float altitudeM, sogMps, cogDeg;
  float hsM, peakFrequencyHz, peakPeriodS, tm02S, directionFromDeg;
  uint8_t spectrum[32];
  int8_t a1[32], b1[32], a2[32], b2[32];
};

static_assert(sizeof(FileHeader) == 44, "Unexpected SWIM file header layout");
static_assert(sizeof(RecordHeader) == 20, "Unexpected SWIM record header layout");
static_assert(sizeof(SummaryPayload) == 212, "Unexpected summary payload layout");

/// Normalizes an FFat entry name to an absolute path.
String normalizedPath(const char *name) {
  String path = name ? String(name) : String();
  if (!path.startsWith("/")) path = "/" + path;
  return path;
}

/// Checks for an exact path without creating a placeholder file.
bool listedPathExists(const String &wanted) {
  File root = FFat.open("/");
  for (File entry = root ? root.openNextFile() : File(); entry;
       entry = root.openNextFile()) {
    if (normalizedPath(entry.name()) == wanted) return true;
  }
  return false;
}
}  // namespace

bool DataLogger::begin() {
  // Format a new FAT partition once, but never auto-format a previously used
  // partition after corruption: preserving field logs is more important.
  Preferences preferences;
  preferences.begin("swim", false);
  const bool wasInitialized = preferences.getBool("ffat_init", false);
  bool mounted = FFat.begin(false);
  if (!mounted && !wasInitialized) {
    mounted = FFat.format(false) && FFat.begin(false);
  }
  if (!mounted) {
    preferences.end();
    status_ = Status::MOUNT_FAILED;
    Serial.println("LOGGER: FFat mount failed. Check 16 MB flash + 3MB APP/9.9MB FATFS; if the partition scheme changed, erase flash once to clear stale NVS/format state.");
    return false;
  }
  preferences.putBool("ffat_init", true);
  preferences.end();

  // A previous experimental allocator could leave thousands of zero-byte
  // placeholders while probing numeric names. They contain no header or data
  // but can exhaust FAT directory entries, causing every later open to fail.
  int emptyLogsRemoved = 0;
  while (true) {
    String stale[16];
    int staleCount = 0;
    File cleanupRoot = FFat.open("/");
    for (File entry = cleanupRoot ? cleanupRoot.openNextFile() : File();
         entry && staleCount < 16; entry = cleanupRoot.openNextFile()) {
      const String path = normalizedPath(entry.name());
      if (!entry.isDirectory() && entry.size() == 0 && path.endsWith(".bin"))
        stale[staleCount++] = path;
    }
    cleanupRoot.close();
    if (!staleCount) break;
    for (int i = 0; i < staleCount; ++i) {
      if (FFat.remove(stale[i])) emptyLogsRemoved++;
      delay(0);
    }
  }
  if (emptyLogsRemoved)
    Serial.printf("LOGGER: removed %d empty stale log files\n", emptyLogsRemoved);

  totalBytes_ = FFat.totalBytes();
  freeBytes_ = FFat.freeBytes();
  status_ = Status::WAITING_DATA;
  strncpy(fileName_, "waiting", sizeof(fileName_) - 1);
  return true;
}

bool DataLogger::startSession(uint64_t utcMs) {
  if (ok()) return true;
  if (status_ != Status::WAITING_DATA) return false;

  if (utcMs) {
    const time_t seconds = (time_t)(utcMs / 1000ULL);
    struct tm utc = {};
    char stamp[20];
    if (gmtime_r(&seconds, &utc) && utc.tm_year >= 120 &&
        strftime(stamp, sizeof(stamp), "%y%m%d_%H%M", &utc)) {
      String target = String("/") + stamp + ".bin";
      if (listedPathExists(target)) {
        bool found = false;
        for (int suffix = 1; suffix <= 99; ++suffix) {
          target = String("/") + stamp + (suffix < 10 ? "_0" : "_") +
                   String(suffix) + ".bin";
          if (!listedPathExists(target)) { found = true; break; }
        }
        if (!found) target = "";
      }
      if (target.length()) {
        strncpy(fileName_, target.c_str(), sizeof(fileName_) - 1);
        fileName_[sizeof(fileName_) - 1] = '\0';
        utcNamed_ = true;
      }
    }
  }

  if (!utcNamed_) {
    int highestNumericIndex = -1;
    uint8_t usedNumeric[1250] = {};
    File root = FFat.open("/");
    for (File entry = root ? root.openNextFile() : File(); entry;
         entry = root.openNextFile()) {
      String name = normalizedPath(entry.name());
      const bool legacyNumeric = name.length() == 14 &&
                                 name.startsWith("/swim_") &&
                                 name.endsWith(".bin");
      const bool shortNumeric = name.length() == 9 && name.endsWith(".bin");
      if (!legacyNumeric && !shortNumeric) continue;
      bool digits = true; int index = 0;
      const int firstDigit = legacyNumeric ? 6 : 1;
      for (int pos = firstDigit; pos < firstDigit + 4; ++pos) {
        if (!isDigit(name[pos])) { digits = false; break; }
        index = index * 10 + (name[pos] - '0');
      }
      if (digits && entry.size() > 0) {
        usedNumeric[index >> 3] |= (uint8_t)(1U << (index & 7));
        if (index > highestNumericIndex) highestNumericIndex = index;
      }
    }
    int nextIndex = highestNumericIndex + 1;
    if (nextIndex > 9999) {
      nextIndex = 0;
      while (nextIndex <= 9999 &&
             (usedNumeric[nextIndex >> 3] & (1U << (nextIndex & 7)))) nextIndex++;
    }
    if (nextIndex > 9999) {
      status_ = Status::OPEN_FAILED;
      Serial.println("LOGGER: numeric filename range exhausted");
      return false;
    }
    snprintf(fileName_, sizeof(fileName_), "/%04d.bin", nextIndex);
  }

  file_ = FFat.open(fileName_, FILE_WRITE);
  if (!file_) {
    status_ = Status::OPEN_FAILED;
    Serial.printf("LOGGER: cannot open %s for writing\n", fileName_);
    return false;
  }

  if (!writeFileHeader()) return false;
  file_.flush();
  ready_ = true;
  status_ = Status::OK;
  lastFlushMs_ = millis();
  Serial.printf("LOGGER: session started as %s\n", fileName_);
  return true;
}

bool DataLogger::writeFileHeader() {
  FileHeader header = {};
  memcpy(header.magic, "SWIMLOG", 7);
  header.formatVersion = 2;
  header.headerSize = sizeof(header);
  header.imuRateMilliHz = (uint32_t)(SwimConfig::IMU_RATE_HZ * 1000);
  header.waveRateMilliHz = (uint32_t)(SwimConfig::WAVE_RATE_HZ * 1000);
  header.fftSize = SwimConfig::FFT_SIZE;
  header.waveBufferSize = waveBufferSize_;
  // Mounting/direction metadata: sensor +X up, +Z toward bow; reported
  // direction is wave FROM, clockwise from bow.
  strncpy(header.firmware, "SWIM-XU-ZB-CW", sizeof(header.firmware) - 1);
  if (file_.write((const uint8_t *)&header, sizeof(header)) != sizeof(header)) {
    status_ = Status::WRITE_FAILED;
    file_.close();
    Serial.println("LOGGER: failed to write the file header");
    return false;
  }
  return true;
}

const char *DataLogger::statusText() const {
  switch (status_) {
    case Status::OK: return "REC";
    case Status::WAITING_DATA: return "WAIT DATA";
    case Status::MOUNT_FAILED: return "MOUNT FAIL";
    case Status::OPEN_FAILED: return "OPEN FAIL";
    case Status::WRITE_FAILED: return "WRITE FAIL";
    case Status::STORAGE_FULL: return "MEM FULL";
    default: return "NOT READY";
  }
}

uint8_t DataLogger::freePercent() const {
  return totalBytes_ ? (uint8_t)((uint64_t)freeBytes_ * 100ULL / totalBytes_) : 0;
}

bool DataLogger::storageLow() const {
  return totalBytes_ && freePercent() <= SwimConfig::STORAGE_WARNING_PERCENT;
}

bool DataLogger::flush() {
  if (!ok()) return false;
  if (!flushBuffer()) return false;
  file_.flush();
  lastFlushMs_ = millis();
  return true;
}

bool DataLogger::applyUtcFileName(uint64_t utcMs) {
  if (utcNamed_ || !ok() || !utcMs) return utcNamed_;
  const time_t seconds = (time_t)(utcMs / 1000ULL);
  struct tm utc = {};
  if (!gmtime_r(&seconds, &utc) || utc.tm_year < 120) return false;

  char stamp[20];
  if (!strftime(stamp, sizeof(stamp), "%y%m%d_%H%M", &utc)) return false;
  String target = String("/") + stamp + ".bin";
  if (listedPathExists(target)) {
    bool uniqueNameFound = false;
    for (int suffix = 1; suffix <= 99; ++suffix) {
      target = String("/") + stamp + (suffix < 10 ? "_0" : "_") +
               String(suffix) + ".bin";
      if (!listedPathExists(target)) { uniqueNameFound = true; break; }
    }
    if (!uniqueNameFound) {
      Serial.println("LOGGER: all UTC filename suffixes are occupied");
      return false;
    }
  }

  if (!flush()) return false;
  const String oldName(fileName_);
  file_.close();
  if (!FFat.rename(oldName, target)) {
    Serial.printf("LOGGER: rename %s failed; starting new UTC file\n", oldName.c_str());
    file_ = FFat.open(target, FILE_WRITE);
    if (file_ && writeFileHeader()) {
      file_.flush();
      strncpy(fileName_, target.c_str(), sizeof(fileName_) - 1);
      fileName_[sizeof(fileName_) - 1] = '\0';
      utcNamed_ = true;
      Serial.printf("LOGGER: UTC rollover filename %s\n", fileName_);
      return true;
    }
    file_.close();
    file_ = FFat.open(oldName, FILE_APPEND);
    status_ = file_ ? Status::OK : Status::OPEN_FAILED;
    ready_ = (bool)file_;
    return false;
  }
  file_ = FFat.open(target, FILE_APPEND);
  if (!file_) {
    status_ = Status::OPEN_FAILED;
    ready_ = false;
    Serial.printf("LOGGER: renamed to %s but failed to reopen\n", target.c_str());
    return false;
  }
  strncpy(fileName_, target.c_str(), sizeof(fileName_) - 1);
  fileName_[sizeof(fileName_) - 1] = '\0';
  utcNamed_ = true;
  Serial.printf("LOGGER: UTC filename %s\n", fileName_);
  return true;
}

bool DataLogger::writeRecord(uint8_t type, uint64_t monotonicUs, uint64_t utcMs,
                             const void *payload, uint16_t payloadSize) {
  if (!ok()) return false;
  const RecordHeader header = {type, 1, payloadSize, monotonicUs, utcMs};
  const size_t needed = sizeof(header) + payloadSize;
  if (needed > WRITE_BUFFER_SIZE) return false;
  if (bufferedBytes_ + needed > WRITE_BUFFER_SIZE && !flushBuffer()) return false;
  memcpy(writeBuffer_ + bufferedBytes_, &header, sizeof(header));
  bufferedBytes_ += sizeof(header);
  memcpy(writeBuffer_ + bufferedBytes_, payload, payloadSize);
  bufferedBytes_ += payloadSize;
  return true;
}

bool DataLogger::flushBuffer() {
  if (!ok()) return false;
  if (!bufferedBytes_) return true;
  if (file_.write(writeBuffer_, bufferedBytes_) != bufferedBytes_) {
    ready_ = false;
    freeBytes_ = FFat.freeBytes();
    status_ = freeBytes_ <= SwimConfig::STORAGE_STOP_FREE_BYTES
                  ? Status::STORAGE_FULL : Status::WRITE_FAILED;
    Serial.println("LOGGER: buffered write failed");
    return false;
  }
  bufferedBytes_ = 0;
  return true;
}

bool DataLogger::writeSummary(const GnssData &g, const WaveResults &w,
                              const uint8_t spectrum[32], const int8_t a1[32],
                              const int8_t b1[32], const int8_t a2[32],
                              const int8_t b2[32], uint64_t monotonicUs,
                              uint64_t utcMs) {
  SummaryPayload p = {};
  p.flags = (g.uartAlive ? 1 : 0) | (g.timeValid ? 2 : 0) |
            (g.fixValid ? 4 : 0) | (w.ready ? 8 : 0) |
            (w.lowFrequencyEdgePeak ? 16 : 0);
  p.latitudeDeg = g.latitudeDeg;
  p.longitudeDeg = g.longitudeDeg;
  p.altitudeM = g.altitudeM;
  p.sogMps = g.sogMps;
  p.cogDeg = g.cogDeg;
  p.hsM = w.hsM;
  p.peakFrequencyHz = w.peakFrequencyHz;
  p.peakPeriodS = w.peakPeriodS;
  p.tm02S = w.tm02S;
  p.directionFromDeg = w.directionFromDeg;
  memcpy(p.spectrum, spectrum, 32);
  memcpy(p.a1, a1, 32); memcpy(p.b1, b1, 32);
  memcpy(p.a2, a2, 32); memcpy(p.b2, b2, 32);
  return writeRecord(RECORD_SUMMARY, monotonicUs, utcMs, &p, sizeof(p));
}

bool DataLogger::writeTimeSync(uint64_t monotonicUs, uint64_t utcMs) {
  return writeRecord(RECORD_TIME_SYNC, monotonicUs, utcMs, &utcMs, sizeof(utcMs));
}

bool DataLogger::writeEvent(uint32_t code, uint64_t monotonicUs, uint64_t utcMs) {
  if (!writeRecord(RECORD_EVENT, monotonicUs, utcMs, &code, sizeof(code))) return false;
  // A manual mark is valuable precisely when something unusual happens. Commit
  // it immediately so a power loss shortly after the click does not erase it.
  if (!flushBuffer()) return false;
  file_.flush();
  lastFlushMs_ = millis();
  return true;
}

void DataLogger::update() {
  if (ok() && millis() - lastFlushMs_ >= SwimConfig::LOG_FLUSH_MS) {
    flushBuffer();
    file_.flush();
    lastFlushMs_ = millis();
  }
}

void DataLogger::checkStorage() {
  if (!ok()) return;
  freeBytes_ = FFat.freeBytes();
  if (freeBytes_ <= SwimConfig::STORAGE_STOP_FREE_BYTES) {
    flushBuffer();
    file_.flush();
    ready_ = false;
    status_ = Status::STORAGE_FULL;
    Serial.println("LOGGER: storage full; recording stopped, measurements continue");
  }
}
