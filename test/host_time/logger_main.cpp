#include "fakes.h"
#include "controller/runtime.h"
#include "controller_hal.h"
#include "posix/log_storage.h"
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
using namespace controller;
struct Empty : hal::ByteStream {
  int read(uint8_t*, size_t) override { return 0; }
  int write(const uint8_t*, size_t) override { return 0; }
};
struct NoScanner : hal::ScannerTransport {
  bool begin() override { return true; }
  uint8_t slotCount() const override { return 6; }
  bool transfer(uint8_t, uint32_t, const uint8_t*, uint8_t*) override { return false; }
};
struct Capture : Output {
  std::string text;
  void write(const uint8_t* data, size_t size, bool = true) override { text.append(reinterpret_cast<const char*>(data), size); }
};
struct Store : storage::PosixLogStorage {
  using PosixLogStorage::PosixLogStorage;
  void* context = nullptr;
  logging::TimestampReader timestamp = nullptr;
  void setTimestampSource(void* value, logging::TimestampReader reader) override { context = value; timestamp = reader; }
};
static std::string sentence(const std::string& body) {
  unsigned crc = 0; for (const auto byte : body) crc ^= static_cast<uint8_t>(byte);
  char tail[8]; std::snprintf(tail, sizeof(tail), "*%02X\r\n", crc); return "$" + body + tail;
}
static void feed(TinyGPSPlus& gps, const char* time = "200959.00", const char* date = "211026") {
  // A clock-bearing RMC sentence without a position fix.
  const auto text = sentence(std::string("GPRMC,") + time + ",V,,,,,,," + date + ",,,N");
  for (const auto byte : text) gps.encode(byte);
  assert(gps.date.isValid() && gps.time.isValid() && !gps.location.isValid());
}
static logging::Config config() { return {7,30000,100,2,3000,1000,5000,2020,true}; }
int main() {
  char root[] = "/tmp/shuriken-time-logger-XXXXXX"; assert(mkdtemp(root));
  TestMonotonic monotonic; Empty bytes; NoScanner scanner;
  pc::bindPlatform(scanner, monotonic, bytes);
  HardwareSerial uart; OutputStream console; Capture output; console.output = &output;
  setenv("TZ", "America/Chicago", 1); tzset();
  // A usable system/RTC clock permits immediate startup without GNSS or sleeps.
  {
    Store store(root); TestUtc utc(monotonic); utc.epoch = 1792613399;
    TinyGPSPlus gps; logging::State state; const auto policy = config();
    logging::Logger logger(store, monotonic, utc, gps, uart, console, policy, state);
    logger.initializeClock(); logger.captureBootTimestamp(); logger.registerStorageTimestamp();
    assert(state.master_clock_valid && monotonic.slept_us == 0 && utc.sets == 0);
    assert(std::strcmp(state.csv_boot_timestamp, "20261021200959") == 0);
    assert(std::strcmp(getenv("TZ"), "America/Chicago") == 0);
    logging::UtcTimestamp timestamp;
    assert(store.timestamp && store.timestamp(store.context, timestamp) && timestamp.second == 59);
    monotonic.us += 2000000;
    assert(store.timestamp(store.context, timestamp) && timestamp.minute == 10 && timestamp.second == 1);
    utc.read_ok = false; logger.initializeClock(); assert(!state.master_clock_valid);
    assert(!store.timestamp(store.context, timestamp));
    utc.read_ok = true; logger.initializeClock(); assert(state.master_clock_valid);
  }
  // Invalid system time can be repaired by fresh GNSS time even without a fix.
  monotonic.us = monotonic.slept_us = 0;
  {
    Store store(root); TestUtc utc(monotonic); TinyGPSPlus gps; logging::State state; const auto policy = config();
    logging::Logger logger(store, monotonic, utc, gps, uart, console, policy, state);
    logger.initializeClock(); assert(!state.master_clock_valid && !logger.ensureBootTimestampFromClock());
    logger.captureBootTimestamp(); assert(monotonic.slept_us == 100000);
    feed(gps); utc.set_ok = false;
    assert(!logger.syncMasterClockFromGnss() && !state.master_clock_valid && utc.sets == 1);
    monotonic.us += 500000; assert(!logger.syncMasterClockFromGnss() && utc.sets == 1);
    monotonic.us += 500000; utc.set_ok = true;
    assert(logger.syncMasterClockFromGnss() && state.master_clock_valid && utc.sets == 2);
    for (unsigned i = 0; i < 20; ++i) { monotonic.us += 10000; assert(logger.syncMasterClockFromGnss()); }
    assert(utc.sets == 2); // Polling the same sentence must not repeatedly reset seconds.
    hal::EpochSeconds epoch; assert(utc.read(epoch) && epoch == 1792613399);
    monotonic.us += 1000000; assert(utc.read(epoch) && epoch == 1792613400);
    monotonic.us += 3000000;
    assert(!logger.syncMasterClockFromGnss() && utc.sets == 2); // Stale data cannot reset the clock.
    logger.initializeClock(); assert(state.master_clock_valid && logger.ensureBootTimestampFromClock());
    feed(gps, "201010.00"); assert(logger.syncMasterClockFromGnss() && utc.sets == 3);
    // Reject impossible/placeholder dates and do not touch the working system clock.
    feed(gps, "201010.00", "310226"); assert(!logger.syncMasterClockFromGnss() && utc.sets == 3);
    feed(gps, "201010.00", "211080"); assert(!logger.syncMasterClockFromGnss() && utc.sets == 3);
  }
  // Failed synchronization preserves an existing system clock; read failure is
  // never promoted to clock-valid even when the setter itself reports success.
  monotonic.us = 0;
  {
    Store store(root); TestUtc utc(monotonic); utc.epoch = 1792600000;
    TinyGPSPlus gps; logging::State state; const auto policy = config();
    logging::Logger logger(store, monotonic, utc, gps, uart, console, policy, state);
    logger.initializeClock(); feed(gps); utc.set_ok = false;
    assert(!logger.syncMasterClockFromGnss() && state.master_clock_valid && utc.epoch == 1792600000);
    monotonic.us += 1000000; utc.set_ok = true; utc.read_ok = false;
    assert(!logger.syncMasterClockFromGnss() && !state.master_clock_valid);
  }
  // A fallback session keeps its filename when time arrives later.
  monotonic.us = 0;
  {
    Store store(root); TestUtc utc(monotonic); TinyGPSPlus gps; logging::State state;
    auto policy = config(); policy.require_valid_time = false; state.storage_ready = true;
    logging::Logger logger(store, monotonic, utc, gps, uart, console, policy, state);
    logger.initializeClock(); logger.tryRecoverLogging(); assert(state.csv_ready);
    const std::string path = state.csv_log_path;
    assert(path.find("NO_GNSS_TIME") != std::string::npos);
    feed(gps); assert(logger.syncMasterClockFromGnss());
    logger.tryRecoverLogging(); assert(path == state.csv_log_path && state.master_clock_valid);
  }
  std::filesystem::remove_all(root);
  puts("PASS: logger system-time startup/GNSS without fixes/failure handling/staleness/timezone/boot waits/fallback");
}
