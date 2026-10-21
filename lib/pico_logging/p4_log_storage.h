#pragma once

#include <Arduino.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstdio>

// A narrow logger backend on the mounted FatFs VFS. POSIX fsync/write return
// errors that Arduino File::flush() cannot report. No second SD driver is used.
namespace pico_logging {
class LogFile : public Print {
 public:
  explicit LogFile(int fd = -1) : fd_(fd) {}
  ~LogFile() { close(); }
  LogFile(const LogFile&) = delete;
  LogFile& operator=(const LogFile&) = delete;
  LogFile(LogFile&& other) noexcept : fd_(other.fd_) {
    if (other.getWriteError()) setWriteError();
    other.fd_ = -1;
  }
  LogFile& operator=(LogFile&& other) noexcept {
    if (this != &other) {
      close();
      clearWriteError();
      if (other.getWriteError()) setWriteError();
      fd_ = other.fd_;
      other.fd_ = -1;
    }
    return *this;
  }
  explicit operator bool() const { return fd_ >= 0; }
  size_t size() const {
    struct stat info = {};
    return fd_ >= 0 && fstat(fd_, &info) == 0 ? static_cast<size_t>(info.st_size) : 0;
  }
  size_t write(uint8_t byte) override { return write(&byte, 1); }
  size_t write(const uint8_t* data, size_t length) override {
    const ssize_t written = fd_ >= 0 ? ::write(fd_, data, length) : -1;
    if (written < 0 || static_cast<size_t>(written) != length) setWriteError();
    return written < 0 ? 0 : static_cast<size_t>(written);
  }
  void flush() { if (fd_ >= 0 && fsync(fd_) != 0) setWriteError(); }
  void close() {
    if (fd_ >= 0 && ::close(fd_) != 0) setWriteError();
    fd_ = -1;
  }
 private:
  int fd_;
};

class LogStorage {
 public:
  explicit LogStorage(const char* mount = "/sdcard") : mount_(mount) {}
  bool exists(const char* path) const {
    char full[256];
    struct stat info = {};
    return fullPath(path, full, sizeof(full)) && stat(full, &info) == 0;
  }
  LogFile openAppend(const char* path) const {
    char full[256];
    if (!fullPath(path, full, sizeof(full))) return LogFile();
    return LogFile(::open(full, O_RDWR | O_CREAT | O_APPEND, 0666));
  }
 private:
  bool fullPath(const char* path, char* out, size_t length) const {
    if (!path || path[0] != '/') return false;
    const int size = snprintf(out, length, "%s%s", mount_, path);
    return size > 0 && static_cast<size_t>(size) < length;
  }
  const char* mount_;
};
inline LogFile openLogAppend(LogStorage& sd, const char* path) { return sd.openAppend(path); }
}  // namespace pico_logging
