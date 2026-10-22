#pragma once
#include "hal/time.h"
#include <limits>
#include <sys/time.h>
namespace controller::hal {
class SystemUtcClock final : public UtcClock {
 public:
  // Backend hooks permit tests without reading or changing the real host clock.
  using Reader = int (*)(timeval*);
  using Writer = int (*)(const timeval*);
  explicit SystemUtcClock(Reader reader = readSystem, Writer writer = setSystem)
      : reader_(reader), writer_(writer) {}
  bool read(EpochSeconds& epoch) const override {
    timeval now = {};
    if (reader_(&now) != 0) return false;
    epoch = static_cast<EpochSeconds>(now.tv_sec); return true;
  }
  bool synchronize(EpochSeconds epoch) override {
    if (epoch < 0 || epoch > std::numeric_limits<time_t>::max()) return false;
    timeval now = {}; now.tv_sec = static_cast<time_t>(epoch);
    return writer_(&now) == 0;
  }
 private:
  static int readSystem(timeval* value) { return ::gettimeofday(value, nullptr); }
  static int setSystem(const timeval* value) { return ::settimeofday(value, nullptr); }
  Reader reader_;
  Writer writer_;
};
}
