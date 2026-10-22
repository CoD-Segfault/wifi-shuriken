#pragma once
#include "controller/scanner_update.h"
#include <SdFat.h>
#include <new>
#include <utility>
namespace controller { namespace rp2350 {
class UpdateStorage final : public ScannerUpdateStorage {
 public:
  explicit UpdateStorage(SdFat& sd) : sd_(sd) {}
  bool exists(const char* path) const override { return sd_.exists(path); }
  std::unique_ptr<ScannerUpdateFile> open(const char* path) override {
    FsFile handle = sd_.open(path, O_RDONLY);
    if (!handle) return {};
    return std::unique_ptr<ScannerUpdateFile>(new (std::nothrow) File(std::move(handle)));
  }
  bool rename(const char* from, const char* to) override { return !sd_.exists(to) && sd_.rename(from, to); }
 private:
  class File final : public ScannerUpdateFile {
   public:
    explicit File(FsFile&& file) : file_(std::move(file)) {}
    ~File() override { close(); }
    uint64_t size() const override { return const_cast<FsFile&>(file_).size(); }
    bool seekSet(uint32_t offset) override { return file_.seekSet(offset); }
    int read(void* data, size_t capacity) override { return file_.read(data, capacity); }
    void close() override { file_.close(); }
   private:
    FsFile file_;
  };
  SdFat& sd_;
};
}}
