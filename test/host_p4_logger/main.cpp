#include "controller_logging.h"
#include "posix/log_storage.h"
#include "spi_protocol_shared.h"
#include <cassert>
#include <fstream>
#include <iterator>
#include <string>
#include <utility>

namespace p4 { bool storageBegin() { return true; } }
static std::string read(const std::string& path) {
  std::ifstream input(path);
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

struct TestClock : controller::hal::Clock {
  uint32_t milliseconds() const override { return fakeMillis; }
  uint32_t microseconds() const override { return fakeMillis * 1000; }
  uint64_t microseconds64() const override { return uint64_t(fakeMillis) * 1000; }
  void sleepMicroseconds(uint32_t us) override { fakeMillis += us / 1000; }
};
struct TestUtc : controller::hal::UtcClock {
  controller::hal::EpochSeconds epoch = 1792600000;
  bool read(controller::hal::EpochSeconds& out) const override { out = epoch; return true; }
  bool synchronize(controller::hal::EpochSeconds value) override { epoch = value; return true; }
};
struct Faults { bool flush = false, size = false, write = false, close = false; };
class CheckedFile final : public controller::logging::LogFile {
 public:
  CheckedFile(std::unique_ptr<controller::logging::LogFile> inner, Faults& faults)
      : inner_(std::move(inner)), faults_(faults) {}
  bool isOpen() const override { return inner_->isOpen(); }
  uint64_t size() const override {
    if (faults_.size) { error_ = true; return 0; }
    return inner_->size();
  }
  size_t write(const uint8_t* data, size_t length) override {
    if (faults_.write) { error_ = true; return inner_->write(data, length / 2); }
    return inner_->write(data, length);
  }
  bool flush() override {
    if (faults_.flush) error_ = true;
    return inner_->flush() && !error_;
  }
  bool close() override { if (faults_.close) error_ = true; return inner_->close() && !error_; }
  bool hasError() const override { return error_ || inner_->hasError(); }
 private:
  std::unique_ptr<controller::logging::LogFile> inner_;
  Faults& faults_;
  mutable bool error_ = false;
};
class TestStorage final : public controller::storage::PosixLogStorage {
 public:
  using PosixLogStorage::PosixLogStorage;
  Faults faults;
  bool fail_open = false;
  unsigned fail_mounts = 0, mounts = 0;
  bool mount() override { ++mounts; if (fail_mounts) { --fail_mounts; return false; } return PosixLogStorage::mount(); }
  std::unique_ptr<controller::logging::LogFile> openAppend(const char* path) override {
    if (fail_open) return {};
    auto file = PosixLogStorage::openAppend(path);
    if (!file) return {};
    return std::make_unique<CheckedFile>(std::move(file), faults);
  }
};
static void storageFailures(const std::string& root, const controller::logging::Config& config) {
  TinyGPSPlus gps; HardwareSerial uart; Stream console; TestClock monotonic; TestUtc utc;
  controller::ScanResult row = {}; row.channel = 1; row.rssi = -50;
  for (unsigned failure = 0; failure < 7; ++failure) {
    TestStorage storage(root.c_str()); controller::logging::State state;
    controller::logging::Logger logger(storage, monotonic, utc, gps, uart, console, config, state);
    const std::string path = "/fault-" + std::to_string(failure) + ".csv";
    if (failure == 0) storage.fail_open = true;
    if (failure == 1) storage.faults.size = true;
    if (failure == 2) storage.faults.write = true;
    if (failure == 3) storage.faults.flush = true;
    const bool opened = logger.initCsvLog(path.c_str());
    assert(opened == (failure >= 4));
    if (failure == 1) {
      assert(read(root + path).empty()); // Failed size query cannot add a header.
      const std::string beforeReset = read(root + "/RESETLOG.CSV");
      state.storage_ready = true;
      logger.initializeClock(); logger.setBootResetReason(1, "PWRON");
      logger.tryAppendBootResetLog();
      assert(!state.boot_reset_log_appended && read(root + "/RESETLOG.CSV") == beforeReset);
    }
    state.csv_ready = state.storage_ready = opened;
    if (failure == 4) {
      storage.faults.write = true; logger.appendCsvRow(row);
      assert(!state.csv_ready && !state.storage_ready && state.csv_rows == 0);
    }
    if (failure == 5) {
      storage.faults.flush = true; logger.flushNow();
      assert(!state.csv_ready && !state.storage_ready);
    }
    if (failure == 6) {
      logger.initializeClock(); logger.setBootResetReason(1, "PWRON");
      storage.faults.close = true; logger.tryAppendBootResetLog();
      assert(!state.boot_reset_log_appended && state.boot_reset_log_pending);
      storage.faults.close = false; logger.tryAppendBootResetLog();
      assert(state.boot_reset_log_appended && !state.boot_reset_log_pending);
    }
  }
  TestStorage storage(root.c_str()); controller::logging::State state;
  auto delayed = config; delayed.storage_init_retry_delay_ms = 7;
  controller::logging::Logger logger(storage, monotonic, utc, gps, uart, console, delayed, state);
  storage.fail_mounts = 2; const uint32_t before = fakeMillis;
  assert(logger.mountStorageWithRetries(3) && storage.mounts == 3 && fakeMillis - before == 14);
  storage.fail_mounts = 1;
  logger.tryRecoverLogging(); assert(!state.storage_ready && storage.mounts == 4);
  fakeMillis += delayed.storage_retry_background_ms;
  logger.tryRecoverLogging(); assert(state.storage_ready && storage.mounts == 5);
}
int main(int argc, char** argv) {
  assert(argc == 2);
  const std::string root = argv[1];
  TestStorage sd(root.c_str());
  assert(sd.mount());
  TinyGPSPlus gps;
  TestClock monotonic; TestUtc utc;
  HardwareSerial uart;
  Stream console;
  // Accept the host's current system clock without ever changing it.
  const controller::logging::Config config = {0, 30000, 0, 1, 3000, 1000, 5000, 2000, P4_REQUIRE_GNSS_TIME_FOR_LOGGING != 0};
  controller::logging::State state;
  state.storage_ready = true;
  controller::logging::Logger logger(sd, monotonic, utc, gps, uart, console, config, state);
  logger.initializeClock();
  logger.registerStorageTimestamp();
  logger.tryRecoverLogging();
  assert(state.csv_ready);
  const std::string originalPath = state.csv_log_path;
  const std::string path = root + originalPath;
  const std::string header = read(path);
  assert(header.find("WigleWifi-1.4,") == 0);
  assert(header.find("010203040506") != std::string::npos);
  assert(header.find("MAC,SSID,AuthMode,FirstSeen,") != std::string::npos);

  controller::ScanResult result = {};
  memcpy(result.bssid, "\x02\x03\x04\x05\x06\x07", 6);
  strcpy(result.ssid, "quoted,\"ssid");
  result.channel = 6;
  result.band = 2;
  result.rssi = -50;
  result.capabilities = CAP_ESS;
  logger.appendCsvRow(result);
  assert(state.csv_rows == 1 && state.csv_rows_blank_gps == 1);
  const char* gga = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
  while (*gga) gps.encode(*gga++);
  assert(gps.location.isValid());
  logger.appendCsvRow(result);
  fakeMillis += 5000;
  logger.flushCsvIfDue();
  assert(state.csv_rows == 2 && state.csv_rows_blank_gps == 1 && !state.csv_dirty);
  const std::string contents = read(path);
  assert(contents.find("\"quoted,\"\"ssid\"") != std::string::npos);
  assert(contents.find("48.1173000,11.5166667") != std::string::npos);
  assert(logger.initCsvLog(originalPath.c_str()));
  assert(read(path) == contents);  // Reopen appends: no header duplication or truncation.

  logger.setBootResetReason(1, "PWRON");
  logger.tryAppendBootResetLog();
  assert(state.boot_reset_log_appended);
  assert(read(root + "/RESETLOG.CSV").find(",PWRON,1,") != std::string::npos);

  // An injected backend flush failure must pause logging and recover safely.
  sd.faults.flush = true;
  logger.appendCsvRow(result);
  fakeMillis += 5000;
  logger.flushCsvIfDue();
  assert(!state.csv_ready && !state.storage_ready);
  const std::string beforeRecovery = read(path);
  sd.faults.flush = false;
  fakeMillis += 30000;
  logger.tryRecoverLogging();
  assert(state.csv_ready && state.storage_ready && std::string(state.csv_log_path) != originalPath);
  assert(read(path) == beforeRecovery);  // Recovery preserves the previous session file.

  controller::storage::PosixLogFile invalid;
  const uint8_t byte = 'x';
  assert(invalid.write(&byte, 1) == 0 && invalid.hasError());
  auto moved = sd.openAppend("/move-test.bin");
  assert(moved && moved->write(&byte, 1) == 1);
  auto owner = std::move(moved);
  assert(!moved && owner && owner->size() == 1);
  owner->flush();
  assert(!owner->hasError());
  assert(!sd.openAppend(std::string(300, 'x').c_str()));

  // Reject the real host clock without changing it: exercise both no-time
  // policies independently from the normal clock-backed session above.
  controller::logging::Config noTimeConfig = config;
  noTimeConfig.min_valid_year = 2070;
  controller::logging::State noTimeState;
  noTimeState.storage_ready = true;
  TinyGPSPlus noTimeGps;
  controller::logging::Logger noTimeLogger(sd, monotonic, utc, noTimeGps, uart, console, noTimeConfig, noTimeState);
  noTimeLogger.initializeClock();
  noTimeLogger.tryRecoverLogging();
#if P4_REQUIRE_GNSS_TIME_FOR_LOGGING
  assert(!noTimeState.csv_ready);
#else
  assert(noTimeState.csv_ready);
  assert(std::string(noTimeState.csv_log_path).find("NO_GNSS_TIME") != std::string::npos);
  const std::string fallbackPath = noTimeState.csv_log_path;
  noTimeLogger.appendCsvRow(result);
  fakeMillis += 5000;
  noTimeLogger.flushCsvIfDue();
  assert(noTimeState.csv_rows == 1 && !noTimeState.master_clock_valid);
  assert(read(root + fallbackPath).find(",,6,-50,") != std::string::npos);
  noTimeLogger.tryRecoverLogging();
  assert(fallbackPath == noTimeState.csv_log_path);
#endif
  storageFailures(root, config);
  puts("PASS: real logger CSV/UID/GPS/reset records, append preservation, checked flush failures/recovery, backend ownership/errors");
}
