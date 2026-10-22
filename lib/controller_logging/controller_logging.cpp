#include "controller_logging.h"
#include "controller/version.h"

#include "hal/logging_platform.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace controller::logging {
namespace {

// Some GNSS modules emit 2080+ placeholder dates before RTC/fix is valid.
static constexpr uint16_t GNSS_INVALID_RTC_YEAR_FLOOR = 2080;
static constexpr size_t DEVICE_UID_HEX_BYTES = 8;
static constexpr char RESET_LOG_PATH[] = "/RESETLOG.CSV";

void appendUidToValue(const char* base, const char* suffix, char* out, size_t out_len) {
  if (!out || out_len == 0) {
    return;
  }

  const char* base_value = (base != nullptr) ? base : "";
  if (suffix != nullptr && suffix[0] != '\0') {
    (void)snprintf(out, out_len, "%s-%s", base_value, suffix);
  } else {
    (void)snprintf(out, out_len, "%s", base_value);
  }
}

void getBoardUidHex(char* out, size_t length) { controller::hal::boardUidHex(out, length); }

}  // namespace

Logger::Logger(LogStorage& storage,
               hal::Clock& monotonic,
               hal::UtcClock& utc,
               TinyGPSPlus& gps,
               Stream& gnss_uart,
               Stream& serial,
               const Config& config,
               State& state)
    : storage_(storage),
      monotonic_(monotonic), utc_(utc),
      gps_ptr_(&gps),
      gnss_uart_(gnss_uart),
      serial_(serial),
      config_(config),
      state_(state) {}

void Logger::setGpsSource(TinyGPSPlus* gps) {
  gps_ptr_ = gps;
}

void Logger::initializeClock() { recoverMasterClockFromSystem(); }

void Logger::registerStorageTimestamp() {
  storage_.setTimestampSource(this, [](void* context, UtcTimestamp& timestamp) {
    return static_cast<Logger*>(context)->storageTimestamp(timestamp);
  });
}

void Logger::setBootResetReason(uint8_t reason_code, const char* reason_text) {
  state_.boot_reset_reason_code = reason_code;
  state_.boot_reset_log_pending = true;
  state_.boot_reset_log_appended = false;

  const char* text = (reason_text != nullptr && reason_text[0] != '\0')
                         ? reason_text
                         : "UNKNOWN";
  strncpy(state_.boot_reset_reason, text, sizeof(state_.boot_reset_reason) - 1);
  state_.boot_reset_reason[sizeof(state_.boot_reset_reason) - 1] = '\0';
}

bool Logger::gnssDateTime(hal::UtcDateTime& utc, TinyGPSPlus* source) const {
  auto& gps = source ? *source : *gps_ptr_;
  if (!gps.date.isValid() || !gps.time.isValid() ||
      gps.date.age() >= config_.gps_field_max_age_ms || gps.time.age() >= config_.gps_field_max_age_ms) return false;
  utc = {static_cast<uint16_t>(gps.date.year()), static_cast<uint8_t>(gps.date.month()),
         static_cast<uint8_t>(gps.date.day()), static_cast<uint8_t>(gps.time.hour()),
         static_cast<uint8_t>(gps.time.minute()), static_cast<uint8_t>(gps.time.second())};
  hal::EpochSeconds epoch;
  return utc.year >= config_.min_valid_year && utc.year < GNSS_INVALID_RTC_YEAR_FLOOR && hal::epochFromUtc(utc, epoch);
}

bool Logger::systemClockEpochLooksPlausible(hal::EpochSeconds epoch) const {
  hal::UtcDateTime utc;
  return hal::utcFromEpoch(epoch, utc) && utc.year >= config_.min_valid_year && utc.year < GNSS_INVALID_RTC_YEAR_FLOOR;
}

bool Logger::recoverMasterClockFromSystem() {
  hal::EpochSeconds epoch;
  const bool valid = utc_.read(epoch) && systemClockEpochLooksPlausible(epoch);
  const bool was_valid = state_.master_clock_valid;
  state_.master_clock_valid = valid;
  if (valid && !was_valid) {
    char stamp[24] = {};
    if (hal::formatUtc(epoch, stamp, sizeof(stamp))) {
      serial_.print("System clock available: "); serial_.print(stamp); serial_.println(" UTC");
    }
  }
  return valid;
}

bool Logger::masterClockNowEpochSeconds(hal::EpochSeconds& epoch) const {
  return state_.master_clock_valid && utc_.read(epoch) && systemClockEpochLooksPlausible(epoch);
}

bool Logger::syncMasterClockFromGnss(TinyGPSPlus* source) {
  hal::UtcDateTime utc;
  if (!gnssDateTime(utc, source)) return false;
  hal::EpochSeconds epoch;
  if (!hal::epochFromUtc(utc, epoch)) return false;
  hal::EpochSeconds current;
  if (gnss_synchronized_ && epoch == last_gnss_epoch_ && masterClockNowEpochSeconds(current)) return true;
  const uint32_t now = monotonic_.milliseconds();
  if (sync_failed_ && uint32_t(now - last_sync_attempt_ms_) < 1000) return false;
  last_sync_attempt_ms_ = now;
  if (!utc_.synchronize(epoch) || !utc_.read(current) || !systemClockEpochLooksPlausible(current)) {
    sync_failed_ = true;
    recoverMasterClockFromSystem();
    serial_.println("GNSS system clock synchronization failed; will retry");
    return false;
  }
  const bool first_sync = !gnss_synchronized_ || !state_.master_clock_valid;
  last_gnss_epoch_ = epoch; gnss_synchronized_ = true; sync_failed_ = false;
  state_.master_clock_valid = true;
  if (first_sync) {
    char stamp[24] = {};
    if (hal::formatUtc(epoch, stamp, sizeof(stamp))) {
      serial_.print("Master clock synchronized: "); serial_.print(stamp); serial_.println(" UTC (GNSS)");
    }
  }
  return true;
}

bool Logger::ensureBootTimestampFromClock() {
  const bool has_fallback_timestamp = strcmp(state_.csv_boot_timestamp, "NO_GNSS_TIME") == 0;
  if (state_.csv_boot_timestamp[0] != '\0' &&
      (config_.require_valid_time || !has_fallback_timestamp || state_.csv_path_selected)) {
    return true;
  }

  recoverMasterClockFromSystem();

  hal::EpochSeconds epoch = 0;
  if (!masterClockNowEpochSeconds(epoch)) {
    if (config_.require_valid_time) return false;
    snprintf(state_.csv_boot_timestamp, sizeof(state_.csv_boot_timestamp), "%s", "NO_GNSS_TIME");
    serial_.println("No valid clock; using NO_GNSS_TIME for CSV filename");
    return true;
  }

  if (!hal::formatUtc(epoch, state_.csv_boot_timestamp, sizeof(state_.csv_boot_timestamp), true)) return false;
  serial_.print("Log boot timestamp: ");
  serial_.println(state_.csv_boot_timestamp);
  return true;
}

bool Logger::storageTimestamp(UtcTimestamp& timestamp) {
  recoverMasterClockFromSystem();
  hal::EpochSeconds epoch;
  if (masterClockNowEpochSeconds(epoch)) return hal::utcFromEpoch(epoch, timestamp);
  return gnssDateTime(timestamp);
}

void Logger::formatTimestamp(char* out, size_t out_len) const {
  hal::EpochSeconds epoch;
  if (!masterClockNowEpochSeconds(epoch) || !hal::formatUtc(epoch, out, out_len)) {
    if (out_len) out[0] = '\0';
  }
}

void Logger::tryAppendBootResetLog() {
  if (!state_.boot_reset_log_pending ||
      state_.boot_reset_log_appended ||
      !state_.storage_ready) {
    return;
  }

  recoverMasterClockFromSystem();

  char timestamp[24] = {};
  formatTimestamp(timestamp, sizeof(timestamp));
  if (timestamp[0] == '\0') {
    return;
  }

  auto reset_log = storage_.openAppend(RESET_LOG_PATH);
  if (!reset_log) {
    serial_.println("Reset log open failed; will retry");
    return;
  }

  const uint64_t reset_size = reset_log->size();
  if (reset_log->hasError()) {
    serial_.println("Reset log size query failed; will retry");
    return;
  }
  if (reset_size == 0) {
    reset_log->println("TimestampUTC,ResetReason,ResetCode,AppVersion");
  }

  char row[128] = {};
  (void)snprintf(row,
                 sizeof(row),
                 "%s,%s,%u,%s",
                 timestamp,
                 state_.boot_reset_reason[0] != '\0' ? state_.boot_reset_reason : "UNKNOWN",
                 (unsigned)state_.boot_reset_reason_code,
                 APP_RELEASE_VERSION);
  reset_log->println(row);
  reset_log->flush();

  if (reset_log->hasError()) {
    serial_.println("Reset log write failed; will retry");
    reset_log->close();
    return;
  }

  if (!reset_log->close()) {
    serial_.println("Reset log close failed; will retry");
    return;
  }
  state_.boot_reset_log_appended = true;
  state_.boot_reset_log_pending = false;

  serial_.print("Reset log appended: ");
  serial_.println(row);
}

void Logger::captureBootTimestamp() {
  // A platform RTC/system clock can supply the boot timestamp immediately.
  if (recoverMasterClockFromSystem()) { ensureBootTimestampFromClock(); return; }
  while (gnss_uart_.available()) {
    gnss_uart_.read();
  }

  const uint8_t reads =
      (config_.gnss_boot_timestamp_reads == 0) ? 1 : config_.gnss_boot_timestamp_reads;
  const uint32_t per_read_timeout_ms = config_.gnss_boot_timestamp_wait_ms / reads;

  for (uint8_t read_idx = 0; read_idx < reads; read_idx++) {
    const uint32_t read_start = monotonic_.milliseconds();
    bool line_complete = false;

    while ((monotonic_.milliseconds() - read_start) < per_read_timeout_ms) {
      while (gnss_uart_.available()) {
        const char c = static_cast<char>(gnss_uart_.read());
        gps_ptr_->encode(c);
        if (c == '\n') {
          line_complete = true;
          break;
        }
      }
      if (line_complete) {
        break;
      }
      monotonic_.sleepMilliseconds(1);
    }
  }

  syncMasterClockFromGnss();
  if (!ensureBootTimestampFromClock()) {
    serial_.println("GNSS time unavailable at boot; delaying log file creation");
  }
}

bool Logger::selectCsvLogPath() {
  if (!state_.storage_ready || state_.csv_boot_timestamp[0] == '\0') {
    return false;
  }
  if (state_.csv_path_selected) {
    return true;
  }

  for (uint16_t suffix = 0; suffix < 100; suffix++) {
    if (suffix == 0) {
      snprintf(state_.csv_log_path, sizeof(state_.csv_log_path),
               "/shuriken_%s.csv", state_.csv_boot_timestamp);
    } else {
      snprintf(state_.csv_log_path, sizeof(state_.csv_log_path),
               "/shuriken_%s_%02u.csv", state_.csv_boot_timestamp, suffix);
    }
    if (!storage_.exists(state_.csv_log_path)) {
      state_.csv_path_selected = true;
      return true;
    }
  }

  return false;
}

bool Logger::mountStorageWithRetries(int max_attempts) {
  for (int attempt = 1; attempt <= max_attempts; attempt++) {
    if (storage_.mount()) {
      return true;
    }

    serial_.print("Storage initialization failed (attempt ");
    serial_.print(attempt);
    serial_.print("/");
    serial_.print(max_attempts);
    serial_.println(")");

    if (attempt < max_attempts) {
      monotonic_.sleepMilliseconds(config_.storage_init_retry_delay_ms);
    }
  }

  return false;
}

bool Logger::initCsvLog(const char* path) {
  log_file_ = storage_.openAppend(path);
  if (!log_file_) return false;

  const uint64_t size = log_file_->size();
  if (log_file_->hasError()) { log_file_.reset(); return false; }
  if (size == 0) {
    char board_uid[DEVICE_UID_HEX_BYTES * 2 + 1] = {};
    char app_release_with_uid[64] = {};
    char release_with_uid[64] = {};

    getBoardUidHex(board_uid, sizeof(board_uid));

    appendUidToValue(APP_RELEASE_VERSION, board_uid, app_release_with_uid, sizeof(app_release_with_uid));
    appendUidToValue(RELEASE_VERSION, board_uid, release_with_uid, sizeof(release_with_uid));

    char header[256] = {};
    (void)snprintf(
        header,
        sizeof(header),
        "WigleWifi-1.4,appRelease=%s,model=WiFi-Shuriken,release=%s,device=WiFi-Shuriken,display=none,board=WiFi-Shuriken,brand=CoD_Segfault",
        app_release_with_uid,
        release_with_uid);
    log_file_->println(header);
    log_file_->println("MAC,SSID,AuthMode,FirstSeen,Channel,RSSI,CurrentLatitude,CurrentLongitude,AltitudeMeters,AccuracyMeters,Type");
    log_file_->flush();
  }
  if (log_file_->hasError()) {
    log_file_->close();
    log_file_.reset();
    return false;
  }
  state_.csv_dirty = false;
  state_.csv_last_flush_ms = monotonic_.milliseconds();
  return true;
}

void Logger::disableLoggingAndScheduleRetry(const char* reason) {
  if (reason && reason[0] != '\0') {
    serial_.println(reason);
  }
  if (log_file_) {
    log_file_->close();
    log_file_.reset();
  }
  state_.csv_ready = false;
  state_.csv_dirty = false;
  state_.storage_ready = false;
  state_.storage_next_retry_ms = monotonic_.milliseconds() + config_.storage_retry_background_ms;
}

void Logger::tryRecoverLogging() {
  if (state_.csv_ready) {
    return;
  }

  const uint32_t now = monotonic_.milliseconds();
  const uint32_t retry_interval =
      (state_.storage_ready && state_.csv_boot_timestamp[0] == '\0')
          ? config_.csv_time_wait_retry_ms
          : config_.storage_retry_background_ms;
  if ((int32_t)(now - state_.storage_next_retry_ms) < 0) {
    return;
  }
  state_.storage_next_retry_ms = now + retry_interval;

  if (!state_.storage_ready) {
    state_.storage_ready = storage_.recover();
    if (!state_.storage_ready) {
      return;
    }
    state_.csv_path_selected = false;
  }

  if (!ensureBootTimestampFromClock()) {
    serial_.println("Waiting for valid GNSS time before creating CSV log file");
    return;
  }

  if (!selectCsvLogPath()) {
    serial_.println("CSV log path selection failed");
    return;
  }

  state_.csv_ready = initCsvLog(state_.csv_log_path);
  if (state_.csv_ready) {
    serial_.print("CSV log ready: ");
    serial_.println(state_.csv_log_path);
  } else {
    serial_.print("CSV log open failed: ");
    serial_.println(state_.csv_log_path);
  }
}

void Logger::appendCsvRow(const ScanResult& r) {
  if (!state_.csv_ready || !log_file_) return;

  char ts[20];
  char lat_csv[20] = "";
  char lon_csv[20] = "";
  char alt_csv[20] = "";
  char acc_csv[20] = "";
  char row_csv[384] = "";

  formatTimestamp(ts, sizeof(ts));

  const bool have_location =
      gps_ptr_->location.isValid() &&
      gps_ptr_->location.age() < config_.gps_field_max_age_ms;
  const bool have_altitude =
      gps_ptr_->altitude.isValid() &&
      gps_ptr_->altitude.age() < config_.gps_field_max_age_ms;
  const bool have_accuracy =
      gps_ptr_->hdop.isValid() &&
      gps_ptr_->hdop.age() < config_.gps_field_max_age_ms;

  if (have_location) {
    snprintf(lat_csv, sizeof(lat_csv), "%.7f", gps_ptr_->location.lat());
    snprintf(lon_csv, sizeof(lon_csv), "%.7f", gps_ptr_->location.lng());
  }
  if (have_altitude) {
    snprintf(alt_csv, sizeof(alt_csv), "%.2f", gps_ptr_->altitude.meters());
  }
  if (have_accuracy) {
    snprintf(acc_csv, sizeof(acc_csv), "%.2f", gps_ptr_->hdop.hdop() * 5.0);
  }
  if (!have_location) {
    state_.csv_rows_blank_gps++;
  }

  if (!buildCsvWifiRow(r.bssid,
                       r.ssid,
                       r.capabilities,
                       ts,
                       r.channel,
                       r.rssi,
                       lat_csv,
                       lon_csv,
                       alt_csv,
                       acc_csv,
                       row_csv,
                       sizeof(row_csv))) {
    disableLoggingAndScheduleRetry("CSV row format error; disabling logging until recovery");
    return;
  }

  log_file_->print(row_csv);
  if (log_file_->hasError()) {
    disableLoggingAndScheduleRetry("CSV log write error; disabling logging until recovery");
    return;
  }
  state_.csv_dirty = true;

  state_.csv_rows++;
  if ((state_.csv_rows % 25) == 0) {
    log_file_->flush();
    if (log_file_->hasError()) {
      disableLoggingAndScheduleRetry("CSV log flush error; disabling logging until recovery");
      return;
    }
    state_.csv_dirty = false;
    state_.csv_last_flush_ms = monotonic_.milliseconds();
  }
}

void Logger::flushCsvIfDue() {
  if (!state_.csv_ready || !log_file_ || !state_.csv_dirty) {
    return;
  }
  const uint32_t now = monotonic_.milliseconds();
  if ((int32_t)(now - state_.csv_last_flush_ms) < (int32_t)config_.csv_flush_interval_ms) {
    return;
  }

  log_file_->flush();
  if (log_file_->hasError()) {
    disableLoggingAndScheduleRetry("CSV log periodic flush error; disabling logging until recovery");
    return;
  }
  state_.csv_dirty = false;
  state_.csv_last_flush_ms = now;
}

}  // namespace controller::logging

namespace controller::logging {
void Logger::flushNow() {
  if (log_file_ && !log_file_->flush())
    disableLoggingAndScheduleRetry("Storage flush failed; logging paused");
}
}
