#include "rp2350/log_storage.h"
#include <cassert>
#include <cstdio>
int main() {
  SdFat filesystem; SPIClass bus; Stream console;
  const uint32_t clocks[] = {20000000, 4000000, 1000000, 400000};
  controller::rp2350::LogStorage storage(filesystem, bus, 17, clocks, 4, console);
  filesystem.good_hz = 1000000;
  assert(storage.mount() && filesystem.attempts.size() == 3 && delay_total == 40);
  for (const auto& attempt : filesystem.attempts)
    assert(attempt.cs == 17 && attempt.bus == &bus && attempt.mode == DEDICATED_SPI);
  filesystem.attempts.clear();
  assert(!storage.recover() && !storage.recover() && storage.recover() && !storage.recover() && !storage.recover());
  const uint32_t expected[] = {20000000, 4000000, 1000000, 400000, 20000000};
  assert(filesystem.attempts.size() == 5);
  for (unsigned i = 0; i < 5; ++i) assert(filesystem.attempts[i].hz == expected[i]);
  auto file = storage.openAppend("/test.csv"); assert(file && *file && handles == 1);
  assert(file->println("abc") == 5 && file->size() == 5 && file->flush());
  assert(file->close() && !*file && !file->hasError() && handles == 0);
  file.reset(); assert(handles == 0);
  for (unsigned fault = 0; fault < 3; ++fault) {
    file_faults = {}; auto failed = storage.openAppend("/failure.csv");
    if (fault == 0) { file_faults.write = true; assert(failed->print("x") == 0); }
    if (fault == 1) { file_faults.sync = true; assert(!failed->flush()); }
    if (fault == 2) { file_faults.close = true; assert(!failed->close()); }
    assert(failed->hasError());
  }
  assert(handles == 0); file_faults = {};
  filesystem.open_fail = true; assert(!storage.openAppend("/failure.csv") && handles == 0);
  storage.setTimestampSource(nullptr, nullptr);
  uint16_t date, time; assert(timestamp_callback); timestamp_callback(&date, &time);
  assert(date == 33 && time == 0);
  int context = 123;
  storage.setTimestampSource(&context, [](void* value, controller::logging::UtcTimestamp& out) {
    assert(*static_cast<int*>(value) == 123); out = {2026, 10, 21, 20, 9, 59}; return true;
  });
  timestamp_callback(&date, &time);
  assert(date == ((2026 - 1980) << 9 | 10 << 5 | 21));
  assert(time == (20 << 11 | 9 << 5 | 59 / 2));
  storage.setTimestampSource(nullptr, [](void*, controller::logging::UtcTimestamp& out) {
    out = {1970, 0, 0, 0, 0, 0}; return false;
  });
  timestamp_callback(&date, &time); assert(date == 33 && time == 0);
  controller::rp2350::LogStorage no_clocks(filesystem, bus, 17, nullptr, 4, console);
  assert(!no_clocks.mount() && !no_clocks.recover());
  puts("PASS: RP2350 SD configuration/fallback/recovery/FAT timestamps/file errors");
}
