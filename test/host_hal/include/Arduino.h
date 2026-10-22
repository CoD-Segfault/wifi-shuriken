#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>
#include <cassert>
#define HIGH 1
#define LOW 0
#define OUTPUT 1
namespace fake {
struct Frame { uint32_t time; uint32_t hz; uint8_t cs; std::vector<uint8_t> data; };
inline uint32_t now = 0;
inline bool watchdog_ok = true;
inline int watchdog_adds = 0, watchdog_feeds = 0, yields = 0;
inline bool begin_ok = true, pullup_ok = true, enabled = false, direct = false;
inline uint8_t cs = 0xff, pending_cs = 0xff;
inline int pin_values[64] = {};
inline std::vector<Frame> frames;
inline std::vector<int> events;
}
inline uint32_t millis() { return fake::now / 1000; }
inline uint32_t micros() { return fake::now; }
inline void delay(uint32_t ms) { fake::now += ms * 1000; }
inline void delayMicroseconds(uint32_t us) { fake::now += us; }
inline void pinMode(int pin, int mode) { fake::events.push_back(1000 + pin * 10 + mode); }
inline void digitalWrite(int pin, int value) {
  fake::events.push_back(pin * 10 + value);
  if (pin == 27) fake::enabled = !value;
  if (pin == 13) fake::direct = !value;
  if (pin == 26 && value) { assert(!fake::enabled); fake::cs = fake::pending_cs; }
  fake::pin_values[pin] = value;
}
struct FakeUart {
  std::deque<uint8_t> rx;
  std::vector<uint8_t> tx;
  size_t space = 2;
  size_t available() const { return rx.size(); }
  int read() { if (rx.empty()) return -1; int c = rx.front(); rx.pop_front(); return c; }
  size_t availableForWrite() const { return space; }
  size_t write(const uint8_t* data, size_t size) { tx.insert(tx.end(), data, data + size); return size; }
};
inline FakeUart fake_uart;
