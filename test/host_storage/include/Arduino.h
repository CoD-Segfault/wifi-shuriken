#pragma once
#include <cstdint>
#include <string>
inline constexpr int HEX = 16;
inline uint32_t delay_total = 0;
inline void delay(uint32_t ms) { delay_total += ms; }
struct Stream {
  template<class T> void print(T) {}
  template<class T> void print(T, int) {}
  template<class T> void println(T) {}
  template<class T> void println(T, int) {}
};
