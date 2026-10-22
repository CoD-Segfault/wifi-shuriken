#pragma once
#include "hal/log_storage.h"
#include <Arduino.h>
#include <SdFat.h>
#include <SPI.h>
#include <new>

namespace controller::rp2350 {
class LogFile final : public logging::LogFile {
 public:
  explicit LogFile(FsFile&& file) : file_(std::move(file)) {}
  ~LogFile() override { close(); }
  bool isOpen() const override { return file_.isOpen(); }
  uint64_t size() const override { if (!file_.isOpen()) { error_ = true; return 0; } return file_.size(); }
  size_t write(const uint8_t* data, size_t length) override {
    const size_t count = file_.write(data, length);
    if (count != length) error_ = true;
    return count;
  }
  bool flush() override { if (!file_.sync()) error_ = true; return !hasError(); }
  bool close() override {
    if (file_.isOpen()) {
      error_ = error_ || file_.getWriteError();
      if (!file_.close()) error_ = true;
    }
    return !error_;
  }
  bool hasError() const override { return error_ || (file_.isOpen() && file_.getWriteError()); }
 private:
  mutable FsFile file_;
  mutable bool error_ = false;
};
class LogStorage final : public logging::LogStorage {
 public:
  LogStorage(SdFat& filesystem, SPIClass& bus, uint8_t cs, const uint32_t* clocks,
             size_t count, Stream& console)
      : filesystem_(filesystem), bus_(bus), cs_(cs), clocks_(clocks), count_(count), console_(console) {}
  bool mount() override {
    if (!clocks_) return false;
    for (size_t i = 0; i < count_; ++i) if (beginAt(clocks_[i])) return true;
    return false;
  }
  bool recover() override {
    if (!clocks_ || !count_) return false;
    const uint32_t hz = clocks_[retry_index_];
    retry_index_ = (retry_index_ + 1) % count_;
    return beginAt(hz);
  }
  bool exists(const char* path) const override { return filesystem_.exists(path); }
  std::unique_ptr<logging::LogFile> openAppend(const char* path) override {
    FsFile file = filesystem_.open(path, O_RDWR | O_CREAT | O_APPEND);
    if (!file) return {};
    return std::unique_ptr<logging::LogFile>(new (std::nothrow) LogFile(std::move(file)));
  }
  void setTimestampSource(void* context, logging::TimestampReader reader) override {
    timestamp_context_ = context; timestamp_reader_ = reader;
    SdFile::dateTimeCallback(timestamp);
  }
 private:
  bool beginAt(uint32_t hz) {
    if (filesystem_.begin(SdSpiConfig(cs_, DEDICATED_SPI, hz, &bus_))) {
      console_.print("SD initialized at "); console_.print(hz); console_.println(" Hz"); return true;
    }
    console_.print("SD init failed @"); console_.print(hz); console_.print("Hz err=");
    printSdErrorSymbol(&console_, filesystem_.sdErrorCode());
    console_.print(" (0x"); console_.print(filesystem_.sdErrorCode(), HEX);
    console_.print(") data=0x"); console_.println(filesystem_.sdErrorData(), HEX);
    delay(20);
    return false;
  }
  static void timestamp(uint16_t* date, uint16_t* time) {
    logging::UtcTimestamp utc;
    if (timestamp_reader_ && !timestamp_reader_(timestamp_context_, utc)) utc = {};
    if (utc.year < 1980 || utc.year > 2107) utc = {};
    *date = static_cast<uint16_t>(((utc.year - 1980) << 9) | (utc.month << 5) | utc.day);
    *time = static_cast<uint16_t>((utc.hour << 11) | (utc.minute << 5) | (utc.second / 2));
  }
  SdFat& filesystem_;
  SPIClass& bus_;
  uint8_t cs_;
  const uint32_t* clocks_;
  size_t count_, retry_index_ = 0;
  Stream& console_;
  inline static void* timestamp_context_ = nullptr;
  inline static logging::TimestampReader timestamp_reader_ = nullptr;
};
}
