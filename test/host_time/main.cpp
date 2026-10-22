#include "fakes.h"
#include "posix/utc_clock.h"
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <cstdio>
#include <initializer_list>
using namespace controller::hal;
static int reads = 0, writes = 0;
static timeval system_value = {};
static bool read_ok = true, write_ok = true;
static int readSystem(timeval* value) { ++reads; *value = system_value; return read_ok ? 0 : -1; }
static int writeSystem(const timeval* value) { ++writes; if (!write_ok) return -1; system_value = *value; return 0; }
int main() {
  EpochSeconds epoch = -1;
  assert(epochFromUtc({1970,1,1,0,0,0}, epoch) && epoch == 0);
  assert(epochFromUtc({2000,1,1,0,0,0}, epoch) && epoch == 946684800);
  assert(epochFromUtc({2038,1,19,3,14,7}, epoch) && epoch == 2147483647);
  assert(epochFromUtc({2040,1,1,0,0,0}, epoch) && epoch == 2208988800LL);
  assert(epochFromUtc({9999,12,31,23,59,59}, epoch) && epoch == 253402300799LL);
  assert(!epochFromUtc({2100,2,29,0,0,0}, epoch));
  assert(epochFromUtc({2000,2,29,0,0,0}, epoch));
  for (const auto& bad : {UtcDateTime{1969,1,1,0,0,0}, UtcDateTime{2026,0,1,0,0,0},
       UtcDateTime{2026,13,1,0,0,0}, UtcDateTime{2026,2,30,0,0,0}, UtcDateTime{2026,1,0,0,0,0},
       UtcDateTime{2026,1,1,24,0,0}, UtcDateTime{2026,1,1,0,60,0}, UtcDateTime{2026,1,1,0,0,60}})
    assert(!epochFromUtc(bad, epoch));
  UtcDateTime utc;
  assert(!utcFromEpoch(-1, utc) && !utcFromEpoch(253402300800LL, utc));
  // Independent libc reference over a full Gregorian leap-year cycle.
  setenv("TZ", "America/Chicago", 1); tzset();
  for (EpochSeconds value = 0; value < 146097LL * 86400; value += 86400 + 17) {
    assert(utcFromEpoch(value, utc));
    const time_t native = value; tm reference = {}; assert(gmtime_r(&native, &reference));
    assert(utc.year == reference.tm_year + 1900 && utc.month == reference.tm_mon + 1 && utc.day == reference.tm_mday);
    assert(utc.hour == reference.tm_hour && utc.minute == reference.tm_min && utc.second == reference.tm_sec);
    assert(epochFromUtc(utc, epoch) && epoch == value);
  }
  char text[24];
  assert(formatUtc(0, text, sizeof(text)) && std::strcmp(text, "1970-01-01 00:00:00") == 0);
  assert(formatUtc(0, text, sizeof(text), true) && std::strcmp(text, "19700101000000") == 0);
  assert(!formatUtc(0, text, 5) && text[0] == '\0');
  assert(!formatUtc(0, nullptr, 0));
  assert(std::strcmp(getenv("TZ"), "America/Chicago") == 0);
  SystemUtcClock native(readSystem, writeSystem);
  system_value.tv_sec = 1792600000;
  assert(native.read(epoch) && epoch == system_value.tv_sec && reads == 1);
  read_ok = false; assert(!native.read(epoch)); read_ok = true;
  write_ok = false; assert(!native.synchronize(1792600010) && writes == 1);
  write_ok = true; assert(native.synchronize(1792600010) && writes == 2);
  assert(system_value.tv_sec == 1792600010 && system_value.tv_usec == 0);
  assert(!native.synchronize(-1) && writes == 2);
  TestMonotonic monotonic; TestUtc host(monotonic); host.epoch = 1792600000;
  LocalUtcClock local(host, monotonic);
  assert(local.read(epoch) && epoch == host.epoch);
  host.epoch += 90; assert(local.read(epoch) && epoch == host.epoch);
  monotonic.us = (uint64_t(UINT32_MAX) - 100) * 1000;
  assert(local.synchronize(1792613399));
  monotonic.us += 2500000;
  assert(monotonic.milliseconds() < 3000); // Cross the 49-day millisecond rollover.
  assert(local.read(epoch) && epoch == 1792613401);
  monotonic.us += 60ULL * 86400 * 1000000;
  host.read_ok = false;
  assert(local.read(epoch) && epoch == 1792613401 + 60LL * 86400 && host.sets == 0);
  assert(!local.synchronize(-1));
  monotonic.us = 0; assert(!local.read(epoch));
  TestMonotonic sleep; sleep.sleepMilliseconds(5000);
  assert(sleep.slept_us == 5000000);
  sleep.sleepMilliseconds(4294968);
  assert(sleep.slept_us == 5000000ULL + 4294968000ULL);
  puts("PASS: UTC/system-clock failures/Gregorian dates/timezones/64-bit monotonic/PC clock isolation");
}
