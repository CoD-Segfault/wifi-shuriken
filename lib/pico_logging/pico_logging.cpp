#include "pico_logging.h"
#include "Configuration.h"

#include <SPI.h>

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#if defined(__has_include)
#if __has_include(<pico/unique_id.h>)
#include <pico/unique_id.h>
#define PICO_LOGGING_HAS_UNIQUE_ID 1
#else
#define PICO_LOGGING_HAS_UNIQUE_ID 0
#endif
#else
#define PICO_LOGGING_HAS_UNIQUE_ID 0
#endif

namespace pico_logging {
namespace {

// Some GNSS modules emit 2080+ placeholder dates before RTC/fix is valid.
static constexpr uint16_t GNSS_INVALID_RTC_YEAR_FLOOR = 2080;
static constexpr size_t PICO_UID_HEX_BYTES = 8;
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

void getBoardUidHex(char* out, size_t out_len) {
  if (!out || out_len == 0) {
    return;
  }
  out[0] = '\0';

#if PICO_LOGGING_HAS_UNIQUE_ID
  pico_unique_board_id_t uid = {};
  pico_get_unique_board_id(&uid);

  uint8_t uid_bytes[PICO_UID_HEX_BYTES] = {};
  const size_t uid_size = sizeof(uid);
  const size_t copy_bytes = (uid_size < PICO_UID_HEX_BYTES) ? uid_size : PICO_UID_HEX_BYTES;
  memcpy(uid_bytes, &uid, copy_bytes);

  if (out_len <= (PICO_UID_HEX_BYTES * 2)) {
    return;
  }
  size_t pos = 0;
  for (size_t i = 0; i < copy_bytes; i++) {
    const int written = snprintf(out + pos, out_len - pos, "%02X", uid_bytes[i]);
    if (written < 0) {
      out[0] = '\0';
      return;
    }
    pos += static_cast<size_t>(written);
  }
  out[pos] = '\0';
#endif
}

}  // namespace

Logger* Logger::date_time_callback_instance_ = nullptr;

Logger::Logger(SdFat& sd,
               FsFile& log_file,
               TinyGPSPlus& gps,
               HardwareSerial& gnss_uart,
               Stream& serial,
               const Config& config,
               State& state)
    : sd_(sd),
      log_file_(log_file),
      gps_ptr_(&gps),
      gnss_uart_(gnss_uart),
      serial_(serial),
      config_(config),
      state_(state) {}

void Logger::setGpsSource(TinyGPSPlus* gps) {
  gps_ptr_ = gps;
}

void Logger::initUtcTimezone() {
  static bool tz_initialized = false;
  if (tz_initialized) {
    recoverMasterClockFromSystem();
    return;
  }

  setenv("TZ", "UTC0", 1);
  tzset();
  tz_initialized = true;
  recoverMasterClockFromSystem();
}

void Logger::registerSdDateTimeCallback() {
  date_time_callback_instance_ = this;
  SdFile::dateTimeCallback(sdDateTimeCallbackThunk);
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

void Logger::sdDateTimeCallbackThunk(uint16_t* date, uint16_t* time) {
  if (date_time_callback_instance_) {
    date_time_callback_instance_->sdDateTimeCallback(date, time);
    return;
  }
  *date = FS_DATE(1980, 1, 1);
  *time = FS_TIME(0, 0, 0);
}

bool Logger::gnssDateTimeValid() const {
  return gps_ptr_->date.isValid() &&
         gps_ptr_->time.isValid() &&
         gps_ptr_->date.year() >= config_.gnss_min_valid_year &&
         gps_ptr_->date.year() < GNSS_INVALID_RTC_YEAR_FLOOR &&
         gps_ptr_->date.month() >= 1 && gps_ptr_->date.month() <= 12 &&
         gps_ptr_->date.day() >= 1 && gps_ptr_->date.day() <= 31 &&
         gps_ptr_->time.hour() <= 23 &&
         gps_ptr_->time.minute() <= 59 &&
         gps_ptr_->time.second() <= 59;
}

bool Logger::gnssDateTimeToTmUtc(struct tm& tm_out) const {
  if (!gnssDateTimeValid()) {
    return false;
  }

  tm_out = {};
  tm_out.tm_year = (int)gps_ptr_->date.year() - 1900;
  tm_out.tm_mon = (int)gps_ptr_->date.month() - 1;
  tm_out.tm_mday = (int)gps_ptr_->date.day();
  tm_out.tm_hour = (int)gps_ptr_->time.hour();
  tm_out.tm_min = (int)gps_ptr_->time.minute();
  tm_out.tm_sec = (int)gps_ptr_->time.second();
  tm_out.tm_isdst = 0;
  return true;
}

bool Logger::systemClockEpochLooksPlausible(time_t epoch) const {
  if (epoch <= 0) {
    return false;
  }

  struct tm tm_utc = {};
  if (gmtime_r(&epoch, &tm_utc) == nullptr) {
    return false;
  }

  const uint16_t year = (uint16_t)(tm_utc.tm_year + 1900);
  return year >= config_.gnss_min_valid_year &&
         year < GNSS_INVALID_RTC_YEAR_FLOOR;
}

bool Logger::recoverMasterClockFromSystem() {
  if (state_.master_clock_valid) {
    return true;
  }

  time_t now = 0;
  time(&now);
  if (!systemClockEpochLooksPlausible(now)) {
    return false;
  }

  state_.master_clock_valid = true;

  char stamp[24] = {};
  struct tm tm_utc = {};
  if (gmtime_r(&now, &tm_utc) != nullptr &&
      strftime(stamp, sizeof(stamp), "%Y-%m-%d %H:%M:%S", &tm_utc) > 0) {
    serial_.print("Recovered persisted master clock: ");
    serial_.print(stamp);
    serial_.println(" UTC");
  }
  return true;
}

bool Logger::masterClockNowEpochSeconds(uint32_t& epoch_out) const {
  if (!state_.master_clock_valid) {
    return false;
  }

  time_t now = 0;
  time(&now);
  if (now < 0) {
    return false;
  }
  epoch_out = (uint32_t)now;
  return true;
}

void Logger::syncMasterClockFromGnss() {
  if (!gnssDateTimeValid()) {
    return;
  }
  if (gps_ptr_->date.age() >= config_.gps_field_max_age_ms ||
      gps_ptr_->time.age() >= config_.gps_field_max_age_ms) {
    return;
  }

  struct tm tm_utc = {};
  if (!gnssDateTimeToTmUtc(tm_utc)) {
    return;
  }

  const time_t epoch = mktime(&tm_utc);
  if (epoch < 0) {
    return;
  }

  const bool was_valid = state_.master_clock_valid;
  struct timeval tv = {};
  tv.tv_sec = epoch;
  tv.tv_usec = 0;
  settimeofday(&tv, nullptr);
  state_.master_clock_valid = true;

  if (!was_valid) {
    char stamp[24] = {};
    struct tm verified_utc = {};
    if (gmtime_r(&epoch, &verified_utc) != nullptr &&
        strftime(stamp, sizeof(stamp), "%Y-%m-%d %H:%M:%S", &verified_utc) > 0) {
      serial_.print("Master clock synchronized: ");
      serial_.print(stamp);
      serial_.println(" UTC");
    }
  }
}

bool Logger::ensureBootTimestampFromClock() {
  if (state_.csv_boot_timestamp[0] != '\0') {
    return true;
  }

  recoverMasterClockFromSystem();

  uint32_t epoch = 0;
  if (!masterClockNowEpochSeconds(epoch)) {
    return false;
  }

  const time_t now = (time_t)epoch;
  struct tm tm_utc = {};
  if (gmtime_r(&now, &tm_utc) == nullptr) {
    return false;
  }

  if (strftime(state_.csv_boot_timestamp, sizeof(state_.csv_boot_timestamp),
               "%Y%m%d%H%M%S", &tm_utc) == 0) {
    return false;
  }
  serial_.print("Log boot timestamp: ");
  serial_.println(state_.csv_boot_timestamp);
  return true;
}

void Logger::sdDateTimeCallback(uint16_t* date, uint16_t* time) {
  uint16_t y = 1980;
  uint8_t mon = 1;
  uint8_t d = 1;
  uint8_t h = 0;
  uint8_t min = 0;
  uint8_t s = 0;

  recoverMasterClockFromSystem();

  uint32_t epoch = 0;
  if (masterClockNowEpochSeconds(epoch)) {
    const time_t now = (time_t)epoch;
    struct tm tm_utc = {};
    if (gmtime_r(&now, &tm_utc) != nullptr) {
      y = (uint16_t)(tm_utc.tm_year + 1900);
      mon = (uint8_t)(tm_utc.tm_mon + 1);
      d = (uint8_t)tm_utc.tm_mday;
      h = (uint8_t)tm_utc.tm_hour;
      min = (uint8_t)tm_utc.tm_min;
      s = (uint8_t)tm_utc.tm_sec;
    }
  } else {
    struct tm tm_utc = {};
    if (gnssDateTimeToTmUtc(tm_utc)) {
      y = (uint16_t)(tm_utc.tm_year + 1900);
      mon = (uint8_t)(tm_utc.tm_mon + 1);
      d = (uint8_t)tm_utc.tm_mday;
      h = (uint8_t)tm_utc.tm_hour;
      min = (uint8_t)tm_utc.tm_min;
      s = (uint8_t)tm_utc.tm_sec;
    }
  }

  if (y < 1980) {
    y = 1980;
    mon = 1;
    d = 1;
    h = 0;
    min = 0;
    s = 0;
  }

  *date = FS_DATE(y, mon, d);
  *time = FS_TIME(h, min, s);
}

void Logger::formatGpsTimestamp(char* out, size_t out_len) const {
  uint32_t epoch = 0;
  if (!masterClockNowEpochSeconds(epoch)) {
    out[0] = '\0';
    return;
  }

  const time_t now = (time_t)epoch;
  struct tm tm_utc = {};
  if (gmtime_r(&now, &tm_utc) == nullptr) {
    out[0] = '\0';
    return;
  }

  if (strftime(out, out_len, "%Y-%m-%d %H:%M:%S", &tm_utc) == 0) {
    out[0] = '\0';
  }
}

void Logger::tryAppendBootResetLog() {
  if (!state_.boot_reset_log_pending ||
      state_.boot_reset_log_appended ||
      !state_.sd_ready) {
    return;
  }

  recoverMasterClockFromSystem();

  char timestamp[24] = {};
  formatGpsTimestamp(timestamp, sizeof(timestamp));
  if (timestamp[0] == '\0') {
    return;
  }

  const bool exists = sd_.exists(RESET_LOG_PATH);
  FsFile reset_log = sd_.open(RESET_LOG_PATH, O_RDWR | O_CREAT | O_APPEND);
  if (!reset_log) {
    serial_.println("Reset log open failed; will retry");
    return;
  }

  if (!exists || reset_log.size() == 0) {
    reset_log.println("TimestampUTC,ResetReason,ResetCode,AppVersion");
  }

  char row[128] = {};
  (void)snprintf(row,
                 sizeof(row),
                 "%s,%s,%u,%s",
                 timestamp,
                 state_.boot_reset_reason[0] != '\0' ? state_.boot_reset_reason : "UNKNOWN",
                 (unsigned)state_.boot_reset_reason_code,
                 APP_RELEASE_VERSION);
  reset_log.println(row);
  reset_log.flush();

  if (reset_log.getWriteError()) {
    serial_.println("Reset log write failed; will retry");
    reset_log.close();
    return;
  }

  reset_log.close();
  state_.boot_reset_log_appended = true;
  state_.boot_reset_log_pending = false;

  serial_.print("Reset log appended: ");
  serial_.println(row);
}

void Logger::captureBootTimestampFromGnss() {
  while (gnss_uart_.available()) {
    gnss_uart_.read();
  }

  const uint8_t reads =
      (config_.gnss_boot_timestamp_reads == 0) ? 1 : config_.gnss_boot_timestamp_reads;
  const uint32_t per_read_timeout_ms = config_.gnss_boot_timestamp_wait_ms / reads;

  for (uint8_t read_idx = 0; read_idx < reads; read_idx++) {
    const uint32_t read_start = millis();
    bool line_complete = false;

    while ((millis() - read_start) < per_read_timeout_ms) {
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
      delay(1);
    }
  }

  syncMasterClockFromGnss();
  if (!ensureBootTimestampFromClock()) {
    serial_.println("GNSS time unavailable at boot; delaying log file creation");
  }
}

bool Logger::selectCsvLogPath() {
  if (!state_.sd_ready || state_.csv_boot_timestamp[0] == '\0') {
    return false;
  }
  if (state_.csv_path_selected) {
    return true;
  }

  for (uint16_t suffix = 0; suffix < 100; suffix++) {
    if (suffix == 0) {
      snprintf(state_.csv_log_path, sizeof(state_.csv_log_path),
               "/CSVLOG_%s.CSV", state_.csv_boot_timestamp);
    } else {
      snprintf(state_.csv_log_path, sizeof(state_.csv_log_path),
               "/CSVLOG_%s_%02u.CSV", state_.csv_boot_timestamp, suffix);
    }
    if (!sd_.exists(state_.csv_log_path)) {
      state_.csv_path_selected = true;
      return true;
    }
  }

  return false;
}

bool Logger::beginSdAtClock(uint32_t hz) {
  const SdSpiConfig cfg(config_.sd_pin_cs, DEDICATED_SPI, hz, &SPI);
  if (sd_.begin(cfg)) {
    serial_.print("SD initialized at ");
    serial_.print(hz);
    serial_.println(" Hz");
    return true;
  }

  serial_.print("SD init failed @");
  serial_.print(hz);
  serial_.print("Hz err=");
  printSdErrorSymbol(&serial_, sd_.sdErrorCode());
  serial_.print(" (0x");
  serial_.print(sd_.sdErrorCode(), HEX);
  serial_.print(")");
  serial_.print(" data=0x");
  serial_.println(sd_.sdErrorData(), HEX);
  delay(20);
  return false;
}

bool Logger::beginSdAtAnyClock() {
  if (!config_.sd_spi_clocks_hz || config_.sd_spi_clock_count == 0) {
    return false;
  }
  for (size_t i = 0; i < config_.sd_spi_clock_count; i++) {
    if (beginSdAtClock(config_.sd_spi_clocks_hz[i])) {
      return true;
    }
  }
  return false;
}

bool Logger::mountSdWithRetries(int max_attempts) {
  for (int attempt = 1; attempt <= max_attempts; attempt++) {
    if (beginSdAtAnyClock()) {
      return true;
    }

    serial_.print("SD initialization failed (attempt ");
    serial_.print(attempt);
    serial_.print("/");
    serial_.print(max_attempts);
    serial_.println(")");

    if (attempt < max_attempts) {
      delay(config_.sd_init_retry_delay_ms);
    }
  }

  return false;
}

bool Logger::initCsvLog(const char* path) {
  const bool exists = sd_.exists(path);
  log_file_ = sd_.open(path, O_RDWR | O_CREAT | O_APPEND);
  if (!log_file_) return false;

  if (!exists || log_file_.size() == 0) {
    char board_uid[PICO_UID_HEX_BYTES * 2 + 1] = {};
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
    log_file_.println(header);
    log_file_.println("MAC,SSID,AuthMode,FirstSeen,Channel,RSSI,CurrentLatitude,CurrentLongitude,AltitudeMeters,AccuracyMeters,Type");
    log_file_.flush();
  }
  state_.csv_dirty = false;
  state_.csv_last_flush_ms = millis();
  return true;
}

void Logger::disableSdLoggingAndScheduleRetry(const char* reason) {
  if (reason && reason[0] != '\0') {
    serial_.println(reason);
  }
  if (log_file_) {
    log_file_.close();
  }
  state_.csv_ready = false;
  state_.csv_dirty = false;
  state_.sd_ready = false;
  state_.sd_next_retry_ms = millis() + config_.sd_retry_background_ms;
}

void Logger::tryRecoverSdLogging() {
  if (state_.csv_ready) {
    return;
  }

  const uint32_t now = millis();
  const uint32_t retry_interval =
      (state_.sd_ready && state_.csv_boot_timestamp[0] == '\0')
          ? config_.csv_time_wait_retry_ms
          : config_.sd_retry_background_ms;
  if ((int32_t)(now - state_.sd_next_retry_ms) < 0) {
    return;
  }
  state_.sd_next_retry_ms = now + retry_interval;

  if (!state_.sd_ready) {
    if (!config_.sd_spi_clocks_hz || config_.sd_spi_clock_count == 0) {
      return;
    }

    const uint32_t retry_hz = config_.sd_spi_clocks_hz[state_.sd_retry_clock_index];
    state_.sd_retry_clock_index = (state_.sd_retry_clock_index + 1) % config_.sd_spi_clock_count;

    serial_.print("Retrying SD initialization @");
    serial_.print(retry_hz);
    serial_.println("Hz...");
    state_.sd_ready = beginSdAtClock(retry_hz);
    if (!state_.sd_ready) {
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

void Logger::appendCsvRow(const QueuedScanResult& r) {
  if (!state_.csv_ready || !log_file_) return;

  char ts[20];
  char lat_csv[20] = "";
  char lon_csv[20] = "";
  char alt_csv[20] = "";
  char acc_csv[20] = "";
  char row_csv[384] = "";

  formatGpsTimestamp(ts, sizeof(ts));

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
    disableSdLoggingAndScheduleRetry("CSV row format error; disabling SD logging until recovery");
    return;
  }

  log_file_.print(row_csv);
  if (log_file_.getWriteError()) {
    disableSdLoggingAndScheduleRetry("CSV log write error; disabling SD logging until recovery");
    return;
  }
  state_.csv_dirty = true;

  state_.csv_rows++;
  if ((state_.csv_rows % 25) == 0) {
    log_file_.flush();
    if (log_file_.getWriteError()) {
      disableSdLoggingAndScheduleRetry("CSV log flush error; disabling SD logging until recovery");
      return;
    }
    state_.csv_dirty = false;
    state_.csv_last_flush_ms = millis();
  }
}

void Logger::flushCsvIfDue() {
  if (!state_.csv_ready || !log_file_ || !state_.csv_dirty) {
    return;
  }
  const uint32_t now = millis();
  if ((int32_t)(now - state_.csv_last_flush_ms) < (int32_t)config_.csv_flush_interval_ms) {
    return;
  }

  log_file_.flush();
  if (log_file_.getWriteError()) {
    disableSdLoggingAndScheduleRetry("CSV log periodic flush error; disabling SD logging until recovery");
    return;
  }
  state_.csv_dirty = false;
  state_.csv_last_flush_ms = now;
}

}  // namespace pico_logging
