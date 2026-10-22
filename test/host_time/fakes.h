#pragma once
#include "hal/time.h"
struct TestMonotonic final : controller::hal::Clock {
  uint64_t us = 0;
  uint64_t slept_us = 0;
  uint32_t milliseconds() const override { return static_cast<uint32_t>(us / 1000); }
  uint32_t microseconds() const override { return static_cast<uint32_t>(us); }
  uint64_t microseconds64() const override { return us; }
  void sleepMicroseconds(uint32_t duration) override { us += duration; slept_us += duration; }
};
struct TestUtc final : controller::hal::UtcClock {
  TestMonotonic& monotonic;
  controller::hal::EpochSeconds epoch = 0;
  uint64_t at = 0;
  bool read_ok = true, set_ok = true;
  mutable unsigned reads = 0;
  unsigned sets = 0;
  explicit TestUtc(TestMonotonic& time) : monotonic(time) {}
  bool read(controller::hal::EpochSeconds& out) const override {
    ++reads;
    if (!read_ok) return false;
    out = epoch + static_cast<controller::hal::EpochSeconds>((monotonic.us - at) / 1000000);
    return true;
  }
  bool synchronize(controller::hal::EpochSeconds value) override {
    ++sets;
    if (!set_ok) return false;
    epoch = value; at = monotonic.us; return true;
  }
};
