#pragma once
#include "hal/log_storage.h"
#include <cerrno>
#include <cstdio>
#include <fcntl.h>
#include <new>
#include <sys/stat.h>
#include <unistd.h>

namespace controller::storage {
class PosixLogFile final : public logging::LogFile {
 public:
  explicit PosixLogFile(int fd = -1) : fd_(fd) {}
  ~PosixLogFile() override { close(); }
  PosixLogFile(const PosixLogFile&) = delete;
  PosixLogFile& operator=(const PosixLogFile&) = delete;
  bool isOpen() const override { return fd_ >= 0; }
  uint64_t size() const override {
    struct stat info = {};
    if (fd_ < 0 || fstat(fd_, &info) != 0 || info.st_size < 0) { error_ = true; return 0; }
    return static_cast<uint64_t>(info.st_size);
  }
  size_t write(const uint8_t* data, size_t length) override {
    const ssize_t written = fd_ >= 0 ? ::write(fd_, data, length) : -1;
    if (written < 0 || static_cast<size_t>(written) != length) error_ = true;
    return written < 0 ? 0 : static_cast<size_t>(written);
  }
  bool flush() override {
    if (fd_ < 0 || fsync(fd_) != 0) error_ = true;
    return !error_;
  }
  bool close() override {
    if (fd_ >= 0 && ::close(fd_) != 0) error_ = true;
    fd_ = -1;
    return !error_;
  }
  bool hasError() const override { return error_; }
 private:
  int fd_;
  mutable bool error_ = false;
};
class PosixLogStorage : public logging::LogStorage {
 public:
  explicit PosixLogStorage(const char* directory) : directory_(directory) {}
  bool mount() override {
    struct stat info = {};
    return directory_ && stat(directory_, &info) == 0 && S_ISDIR(info.st_mode);
  }
  bool exists(const char* path) const override {
    char full[1024]; struct stat info = {};
    if (!fullPath(path, full, sizeof(full))) return true;
    // An unknown path is never considered available for a new session name.
    return stat(full, &info) == 0 || errno != ENOENT;
  }
  std::unique_ptr<logging::LogFile> openAppend(const char* path) override {
    char full[1024];
    if (!fullPath(path, full, sizeof(full))) return {};
    const int fd = ::open(full, O_WRONLY | O_CREAT | O_APPEND, 0666);
    if (fd < 0) return {};
    auto* handle = new (std::nothrow) PosixLogFile(fd);
    if (!handle) ::close(fd);
    return std::unique_ptr<logging::LogFile>(handle);
  }
 private:
  bool fullPath(const char* path, char* out, size_t length) const {
    if (!directory_ || !path || path[0] != '/') return false;
    // Reject lexical parent traversal and dot components in virtual paths.
    for (const char* part = path + 1; *part;) {
      const char* end = std::strchr(part, '/');
      const size_t size = end ? static_cast<size_t>(end - part) : std::strlen(part);
      if ((size == 1 && part[0] == '.') || (size == 2 && part[0] == '.' && part[1] == '.')) return false;
      if (!end) break;
      part = end + 1;
    }
    const int size = std::snprintf(out, length, "%s%s", directory_, path);
    return size > 0 && static_cast<size_t>(size) < length;
  }
  const char* directory_;
};
}
