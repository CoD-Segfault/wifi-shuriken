#pragma once
#include "hal/controller.h"
#include <memory>
#include <stddef.h>
#include <stdint.h>

namespace controller {
class Output;
class ScannerUpdateFile {
 public:
  virtual ~ScannerUpdateFile() = default;
  virtual uint64_t size() const = 0;
  virtual bool seekSet(uint32_t offset) = 0;
  virtual int read(void* data, size_t capacity) = 0;
  virtual void close() = 0;
};
class ScannerUpdateStorage {
 public:
  virtual ~ScannerUpdateStorage() = default;
  virtual bool exists(const char* path) const = 0;
  virtual std::unique_ptr<ScannerUpdateFile> open(const char* path) = 0;
  // Must fail if the destination already exists; never replace an applied image.
  virtual bool rename(const char* source, const char* destination) = 0;
};
class ScannerUpdateObserver {
 public:
  virtual ~ScannerUpdateObserver() = default;
  virtual void started() = 0;
};
struct ScannerUpdateConfig {
  bool enabled = true;
  const char* image_path = "/scanner.bin";
  const char* applied_path = "/scanner.applied.bin";
  uint32_t normal_hz = 20000000, update_hz = 20000000;
  uint32_t first_pull_us = 500, identify_first_pull_us = 250;
  uint32_t interframe_us = 25, normal_interframe_us = 10;
  uint16_t response_pulls = 500;
  uint8_t packet_retries = 5;
  uint32_t hash_chunk_bytes = 512, flash_sector_bytes = 0;
};
// Exclusive boot-time operation, before the scanner worker starts. Owns the
// transport and image handles for its duration. Returns true if any scanner was
// updated; preserves the staged file unless every eligible scanner succeeds.
bool updateScannerFirmware(ScannerUpdateStorage& storage, hal::ScannerTransport& transport,
                           hal::Clock& clock, Output& output, const ScannerUpdateConfig& config,
                           ScannerUpdateObserver* observer = nullptr);
}
