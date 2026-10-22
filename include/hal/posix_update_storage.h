#pragma once
#include "controller/scanner_update.h"
#include <cstdio>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <new>
#if defined(SHURIKEN_PC_CONTROLLER)
#include <sys/syscall.h>
#endif

namespace controller { namespace hal {
// Checked read-only image handles on a mounted VFS or a native directory.
class PosixUpdateStorage final : public ScannerUpdateStorage {
 public:
  explicit PosixUpdateStorage(const char* root) : root_(root) {}
  bool exists(const char* path) const override {
    char full[512]; struct stat info = {};
    return fullPath(path, full, sizeof(full)) && stat(full, &info) == 0;
  }
  std::unique_ptr<ScannerUpdateFile> open(const char* path) override {
    char full[512];
    if (!fullPath(path, full, sizeof(full))) return {};
    const int fd = ::open(full, O_RDONLY);
    if (fd < 0) return {};
    std::unique_ptr<ScannerUpdateFile> file(new (std::nothrow) File(fd));
    if (!file) ::close(fd);
    return file;
  }
  bool rename(const char* source, const char* destination) override {
    char from[512], to[512];
    if (!fullPath(source, from, sizeof(from)) || !fullPath(destination, to, sizeof(to))) return false;
#if defined(SHURIKEN_PC_CONTROLLER)
    // Linux renameat2 works on mounted FAT/exFAT too and never replaces a marker.
    return ::syscall(SYS_renameat2, AT_FDCWD, from, AT_FDCWD, to, 1 /* RENAME_NOREPLACE */) == 0;
#else
    // FatFs rename fails with FR_EXIST when the destination already exists.
    if (exists(destination)) return false;
    return ::rename(from, to) == 0;
#endif
  }
 private:
  class File final : public ScannerUpdateFile {
   public:
    explicit File(int fd) : fd_(fd) {}
    ~File() override { close(); }
    uint64_t size() const override {
      struct stat info = {};
      return fd_ >= 0 && fstat(fd_, &info) == 0 && info.st_size >= 0 ? static_cast<uint64_t>(info.st_size) : 0;
    }
    bool seekSet(uint32_t offset) override { return lseek(fd_, offset, SEEK_SET) == static_cast<off_t>(offset); }
    int read(void* data, size_t capacity) override { return static_cast<int>(::read(fd_, data, capacity)); }
    void close() override { if (fd_ >= 0) ::close(fd_); fd_ = -1; }
   private:
    int fd_;
  };
  bool fullPath(const char* path, char* output, size_t capacity) const {
    if (!root_ || !path || path[0] != '/') return false;
    const int length = std::snprintf(output, capacity, "%s%s", root_, path);
    return length > 0 && static_cast<size_t>(length) < capacity;
  }
  const char* root_;
};
}}
