#pragma once
#include "Arduino.h"
#include "SPI.h"
#include <algorithm>
#include <cassert>
#include <utility>
#include <vector>
constexpr int O_RDWR = 1, O_CREAT = 2, O_APPEND = 4, DEDICATED_SPI = 1;
struct SdSpiConfig {
  uint8_t cs; int mode; uint32_t hz; SPIClass* bus;
  SdSpiConfig(uint8_t cs_, int mode_, uint32_t hz_, SPIClass* bus_) : cs(cs_), mode(mode_), hz(hz_), bus(bus_) {}
};
inline void (*timestamp_callback)(uint16_t*, uint16_t*) = nullptr;
struct SdFile { static void dateTimeCallback(void (*callback)(uint16_t*, uint16_t*)) { timestamp_callback = callback; } };
struct FileFaults { bool write = false, sync = false, close = false; };
inline FileFaults file_faults;
inline unsigned handles = 0;
struct FsFile {
  bool open = false, write_error = false;
  size_t bytes = 0;
  explicit FsFile(bool ready = false) : open(ready) { if (open) ++handles; }
  FsFile(FsFile&& other) : open(other.open), bytes(other.bytes) { other.open = false; }
  FsFile(const FsFile&) = delete;
  ~FsFile() { close(); }
  explicit operator bool() const { return open; }
  bool isOpen() const { return open; }
  uint64_t size() const { return bytes; }
  size_t write(const uint8_t*, size_t length) {
    if (!open || file_faults.write) { write_error = true; return 0; }
    bytes += length; return length;
  }
  bool getWriteError() const { return !open || write_error; }
  bool sync() { return open && !file_faults.sync; }
  bool close() { if (!open) return true; --handles; open = false; return !file_faults.close; }
};
struct SdFat {
  std::vector<SdSpiConfig> attempts;
  uint32_t good_hz = 0;
  bool open_fail = false;
  bool begin(SdSpiConfig config) { attempts.push_back(config); return config.hz == good_hz; }
  bool exists(const char*) { return false; }
  FsFile open(const char*, int flags) { assert(flags == (O_RDWR | O_CREAT | O_APPEND)); return FsFile(!open_fail); }
  int sdErrorCode() { return 1; }
  int sdErrorData() { return 2; }
};
inline void printSdErrorSymbol(Stream*, int) {}
