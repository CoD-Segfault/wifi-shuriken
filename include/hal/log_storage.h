#pragma once
#include "hal/time.h"
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>

namespace controller::logging {
// A failed or short write, size query, flush or close must latch hasError().
// Opening a new handle starts with a clear error state. Handles own their files.
class LogFile {
 public:
  virtual ~LogFile() = default;
  virtual bool isOpen() const = 0;
  explicit operator bool() const { return isOpen(); }
  virtual uint64_t size() const = 0;
  virtual size_t write(const uint8_t* data, size_t length) = 0;
  virtual bool flush() = 0;
  virtual bool close() = 0;
  virtual bool hasError() const = 0;
  size_t print(const char* text) { return write(reinterpret_cast<const uint8_t*>(text), std::strlen(text)); }
  size_t println(const char* text) { const size_t count = print(text); return count + print("\r\n"); }
};
using UtcTimestamp = hal::UtcDateTime;
using TimestampReader = bool (*)(void* context, UtcTimestamp& timestamp);
class LogStorage {
 public:
  virtual ~LogStorage() = default;
  // Backends own mounting, device/bus configuration and fallback mechanisms.
  virtual bool mount() = 0;
  virtual bool recover() { return mount(); }
  virtual bool exists(const char* path) const = 0;
  // Paths are rooted within this storage. Existing contents must be preserved.
  virtual std::unique_ptr<LogFile> openAppend(const char* path) = 0;
  virtual void setTimestampSource(void*, TimestampReader) {}
};
}
