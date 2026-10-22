#pragma once
#include "hal/controller.h"
#include <cstddef>
#include <cstdint>
#include <limits>
namespace controller::hal {
using EpochSeconds = int64_t;
struct UtcDateTime {
  uint16_t year = 1980;
  uint8_t month = 1, day = 1, hour = 0, minute = 0, second = 0;
};
// Reads UTC from the platform; synchronization must report whether it succeeded.
class UtcClock {
 public:
  virtual ~UtcClock() = default;
  virtual bool read(EpochSeconds& epoch) const = 0;
  virtual bool synchronize(EpochSeconds epoch) = 0;
};
bool epochFromUtc(const UtcDateTime& utc, EpochSeconds& epoch);
bool utcFromEpoch(EpochSeconds epoch, UtcDateTime& utc);
bool formatUtc(EpochSeconds epoch, char* output, size_t capacity, bool compact = false);

// Uses the system clock until GNSS synchronization, then advances a local UTC
// anchor with monotonic time. Never writes to the supplied system clock.
class LocalUtcClock final : public UtcClock {
 public:
  LocalUtcClock(UtcClock& system, Clock& monotonic) : system_(system), monotonic_(monotonic) {}
  bool read(EpochSeconds& epoch) const override {
    if (!synchronized_) return system_.read(epoch);
    const uint64_t now = monotonic_.microseconds64();
    if (now < anchor_us_) return false;
    const uint64_t seconds = (now - anchor_us_) / 1000000;
    if (seconds > static_cast<uint64_t>(std::numeric_limits<EpochSeconds>::max() - anchor_)) return false;
    epoch = anchor_ + static_cast<EpochSeconds>(seconds);
    return true;
  }
  bool synchronize(EpochSeconds epoch) override {
    UtcDateTime utc;
    if (!utcFromEpoch(epoch, utc)) return false;
    anchor_ = epoch; anchor_us_ = monotonic_.microseconds64(); synchronized_ = true;
    return true;
  }
 private:
  UtcClock& system_;
  Clock& monotonic_;
  bool synchronized_ = false;
  EpochSeconds anchor_ = 0;
  uint64_t anchor_us_ = 0;
};
}
