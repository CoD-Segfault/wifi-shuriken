#pragma once
// Framework compatibility for the unchanged TinyGPS++/Print-based CSV library.
// The controller application itself depends only on controller/application.h.
#include "hal/platform.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <thread>
#include <chrono>
#include <ctime>
using byte = uint8_t;
constexpr double TWO_PI = 6.28318530717958647692;
inline double radians(double value) { return value * TWO_PI / 360.0; }
inline double degrees(double value) { return value * 360.0 / TWO_PI; }
template<class T> inline T sq(T value) { return value * value; }
inline unsigned long millis() { return controller::hal::clock().milliseconds(); }
inline void delay(uint32_t ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
class Print {
 public:
  virtual ~Print() = default;
  virtual size_t write(uint8_t byte) = 0;
  virtual size_t write(const uint8_t* data, size_t length) {
    size_t count = 0;
    while (count < length && write(data[count]) == 1) ++count;
    return count;
  }
  size_t print(const char* text) { return write(reinterpret_cast<const uint8_t*>(text), std::strlen(text)); }
  size_t print(char* text) { return print(static_cast<const char*>(text)); }
  template<class T> size_t print(T value) { return print(std::to_string(value).c_str()); }
  size_t println() { return print("\r\n"); }
  template<class T> size_t println(T value) { return print(value) + println(); }
  int getWriteError() const { return error_; }
  void clearWriteError() { error_ = 0; }
 protected:
  void setWriteError(int error = 1) { error_ = error; }
 private:
  int error_ = 0;
};
class Stream : public Print {
 public:
  virtual int available() = 0;
  virtual int read() = 0;
  virtual int peek() = 0;
  virtual void flush() = 0;
  virtual int availableForWrite() { return 0; }
};
class HardwareSerial final : public Stream {
 public:
  controller::hal::ByteStream* stream = nullptr;
  int available() override { return peek() >= 0; }
  int peek() override {
    if (cached_ < 0 && stream) {
      uint8_t byte;
      if (stream->read(&byte, 1) == 1) cached_ = byte;
    }
    return cached_;
  }
  int read() override { const int byte = peek(); cached_ = -1; return byte; }
  void flush() override {}
  size_t write(uint8_t byte) override {
    return stream && stream->write(&byte, 1) == 1 ? 1 : 0;
  }
 private:
  int cached_ = -1;
};
