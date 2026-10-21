#pragma once
#include <cstdint>
#include <cstddef>
#include <cmath>
#include <string>
#include <cstring>
using byte = uint8_t;
constexpr double TWO_PI = 6.28318530717958647692;
inline double radians(double value) { return value * TWO_PI / 360.0; }
inline double degrees(double value) { return value * 360.0 / TWO_PI; }
template<class T> inline T sq(T value) { return value * value; }
inline uint32_t fakeMillis = 0;
inline uint32_t millis() { return fakeMillis; }
inline void delay(uint32_t ms) { fakeMillis += ms; }
class Print {
  int error_ = 0;
 public:
  virtual ~Print() = default;
  virtual size_t write(uint8_t) = 0;
  virtual size_t write(const uint8_t* data, size_t length) {
    size_t count = 0;
    while (count < length && write(data[count]) == 1) ++count;
    return count;
  }
  size_t print(const char* text) { return write(reinterpret_cast<const uint8_t*>(text), strlen(text)); }
  size_t print(char* text) { return print(static_cast<const char*>(text)); }
  template<class T> size_t print(T value) { return print(std::to_string(value).c_str()); }
  template<class T> size_t println(T value) { return print(value) + print("\r\n"); }
  int getWriteError() const { return error_; }
  void clearWriteError() { error_ = 0; }
 protected:
  void setWriteError(int value = 1) { error_ = value; }
};
class Stream : public Print {
 public:
  std::string output;
  virtual int available() { return 0; }
  virtual int read() { return -1; }
  virtual int peek() { return -1; }
  virtual void flush() {}
  size_t write(uint8_t value) override { output.push_back(value); return 1; }
};
class HardwareSerial : public Stream {};
struct EspFixture { uint64_t getEfuseMac() { return 0x010203040506ULL; } };
inline EspFixture ESP;
