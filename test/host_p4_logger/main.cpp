#include "pico_logging.h"
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

int main(int argc, char** argv) {
  assert(argc == 2);
  const std::string root = argv[1];
  pico_logging::LogStorage sd(root.c_str());
  pico_logging::LogFile log;
  TinyGPSPlus gps;
  HardwareSerial uart;
  Stream console;
  const uint32_t clocks[] = {20000000};
  // Accept the host's current system clock without ever changing it.
  const pico_logging::Config config = {0, clocks, 1, 0, 30000, 0, 1, 3000, 1000, 5000, 2000};
  pico_logging::State state;
  state.sd_ready = true;
  pico_logging::Logger logger(sd, log, gps, uart, console, config, state);
  logger.initUtcTimezone();
  logger.registerSdDateTimeCallback();
  logger.tryRecoverSdLogging();
  assert(state.csv_ready && log);
  const std::string originalPath = state.csv_log_path;
  const std::string path = root + originalPath;
  const std::string header = read(path);
  assert(header.find("WigleWifi-1.4,") == 0);
  assert(header.find("010203040506") != std::string::npos);
  assert(header.find("MAC,SSID,AuthMode,FirstSeen,") != std::string::npos);

  pico_logging::QueuedScanResult result = {};
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
  log.close();
  assert(logger.initCsvLog(originalPath.c_str()));
  assert(read(path) == contents);  // Reopen appends: no header duplication or truncation.

  logger.setBootResetReason(1, "PWRON");
  logger.tryAppendBootResetLog();
  assert(state.boot_reset_log_appended);
  assert(read(root + "/RESETLOG.CSV").find(",PWRON,1,") != std::string::npos);

  // /dev/null accepts writes but rejects fsync: periodic flush must disable
  // logging rather than silently reporting durability, then recover safely.
  log = pico_logging::LogFile(::open("/dev/null", O_WRONLY));
  logger.appendCsvRow(result);
  fakeMillis += 5000;
  logger.flushCsvIfDue();
  assert(!state.csv_ready && !state.sd_ready && !log);
  fakeMillis += 30000;
  logger.tryRecoverSdLogging();
  assert(state.csv_ready && state.sd_ready && std::string(state.csv_log_path) != originalPath);
  assert(read(path) == contents);  // Recovery preserves the previous session file.

  pico_logging::LogFile invalid;
  assert(invalid.write(static_cast<uint8_t>('x')) == 0 && invalid.getWriteError());
  auto moved = sd.openAppend("/move-test.bin");
  assert(moved && moved.write(static_cast<uint8_t>('x')) == 1);
  auto owner = std::move(moved);
  assert(!moved && owner && owner.size() == 1);
  owner.flush();
  assert(!owner.getWriteError());
  assert(!sd.openAppend(std::string(300, 'x').c_str()));

  // Reject the real host clock without changing it: exercise both no-time
  // policies independently from the normal clock-backed session above.
  pico_logging::Config noTimeConfig = config;
  noTimeConfig.gnss_min_valid_year = 2070;
  pico_logging::State noTimeState;
  noTimeState.sd_ready = true;
  pico_logging::LogFile noTimeFile;
  TinyGPSPlus noTimeGps;
  pico_logging::Logger noTimeLogger(sd, noTimeFile, noTimeGps, uart, console, noTimeConfig, noTimeState);
  noTimeLogger.initUtcTimezone();
  noTimeLogger.tryRecoverSdLogging();
#if P4_REQUIRE_GNSS_TIME_FOR_LOGGING
  assert(!noTimeState.csv_ready && !noTimeFile);
#else
  assert(noTimeState.csv_ready && noTimeFile);
  assert(std::string(noTimeState.csv_log_path).find("NO_GNSS_TIME") != std::string::npos);
  const std::string fallbackPath = noTimeState.csv_log_path;
  noTimeLogger.appendCsvRow(result);
  fakeMillis += 5000;
  noTimeLogger.flushCsvIfDue();
  assert(noTimeState.csv_rows == 1 && !noTimeState.master_clock_valid);
  assert(read(root + fallbackPath).find(",,6,-50,") != std::string::npos);
  noTimeLogger.tryRecoverSdLogging();
  assert(fallbackPath == noTimeState.csv_log_path);
#endif
  puts("PASS: real logger CSV/UID/GPS/reset records, append preservation, checked flush failures/recovery, backend ownership/errors");
}
