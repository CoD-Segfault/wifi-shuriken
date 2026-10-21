#pragma once
#include <cstdint>
#include <cstddef>
#include <deque>
#include <string>
struct Stream {
  virtual ~Stream() = default;
  virtual int available() = 0;
  virtual int read() = 0;
  virtual int peek() = 0;
  virtual void flush() = 0;
  virtual int availableForWrite() { return 0; }
  virtual size_t write(uint8_t) = 0;
  virtual size_t write(const uint8_t*, size_t) = 0;
};
inline uint32_t fakeMillis = 0;
inline uint32_t millis() { return fakeMillis; }
inline void delay(uint32_t ms) { fakeMillis += ms; }
struct FakeSerial {
  std::deque<char> input;
  std::string output;
  int writable = 64;
  void begin(unsigned) {}
  int availableForWrite() { return writable; }
  int available() { return static_cast<int>(input.size()); }
  int read() {
    if (input.empty()) return -1;
    const unsigned char c = input.front();
    input.pop_front();
    return c;
  }
  size_t write(const uint8_t* data, size_t length) {
    output.append(reinterpret_cast<const char*>(data), length);
    return length;
  }
  void command(const std::string& text) { input.insert(input.end(), text.begin(), text.end()); }
};
inline FakeSerial Serial0;
