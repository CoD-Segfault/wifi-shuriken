#include "pico_master_logging.h"

#include <SPI.h>

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "spi_protocol_shared.h"

namespace pico_logging {
namespace {

void appendAuthToken(char* out, size_t out_len, const char* token) {
  if (!out || out_len == 0 || !token || token[0] == '\0') {
    return;
  }

  const size_t used = strlen(out);
  if (used >= (out_len - 1)) {
    return;
  }

  const size_t remaining = out_len - used - 1;
  strncat(out, token, remaining);
  out[out_len - 1] = '\0';
}

void appendProtocolAuthToken(char* out,
                             size_t out_len,
                             const char* proto,
                             bool psk,
                             bool eap,
                             bool tkip,
                             bool ccmp) {
  char token[48] = {};
  size_t pos = 0;
  pos += (size_t)snprintf(token + pos, sizeof(token) - pos, "[%s", proto);

  if (psk) {
    pos += (size_t)snprintf(token + pos, sizeof(token) - pos, "-PSK");
  } else if (eap) {
    pos += (size_t)snprintf(token + pos, sizeof(token) - pos, "-EAP");
  }

  if (tkip && ccmp) {
    pos += (size_t)snprintf(token + pos, sizeof(token) - pos, "-CCMP+TKIP");
  } else if (ccmp) {
    pos += (size_t)snprintf(token + pos, sizeof(token) - pos, "-CCMP");
  } else if (tkip) {
    pos += (size_t)snprintf(token + pos, sizeof(token) - pos, "-TKIP");
  }

  (void)snprintf(token + pos, sizeof(token) - pos, "]");
  appendAuthToken(out, out_len, token);
}

}  // namespace

Logger* Logger::date_time_callback_instance_ = nullptr;

void formatBssid(const uint8_t* bssid, char* out, size_t out_len) {
  snprintf(out, out_len, "%02X:%02X:%02X:%02X:%02X:%02X",
           bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
}

void wigleCsvEscape(const char* in, char* out, size_t out_len) {
  size_t j = 0;
  if (out_len == 0) return;
  out[j++] = '"';
  for (size_t i = 0; in[i] != '\0' && j + 2 < out_len; i++) {
    if (in[i] == '"') {
      if (j + 2 >= out_len) break;
      out[j++] = '"';
      out[j++] = '"';
    } else {
      out[j++] = in[i];
    }
  }
  out[j++] = '"';
  out[j] = '\0';
}

void buildAndroidCapabilitiesString(uint16_t caps, char* out, size_t out_len) {
  if (!out || out_len == 0) {
    return;
  }

  out[0] = '\0';

  const bool has_wep = (caps & CAP_WEP) != 0;
  const bool has_wpa = (caps & CAP_WPA) != 0;
  const bool has_wpa2 = (caps & CAP_WPA2) != 0;
  const bool has_wpa3 = (caps & CAP_WPA3) != 0;
  const bool has_psk = (caps & CAP_PSK) != 0;
  const bool has_eap = (caps & CAP_EAP) != 0;
  const bool has_tkip = (caps & CAP_TKIP) != 0;
  const bool has_ccmp = (caps & CAP_CCMP) != 0;

  if (has_wep) {
    appendAuthToken(out, out_len, "[WEP]");
  }
  if (has_wpa) {
    appendProtocolAuthToken(out, out_len, "WPA", has_psk, has_eap, has_tkip, has_ccmp);
  }
  if (has_wpa2) {
    appendProtocolAuthToken(out, out_len, "WPA2", has_psk, has_eap, has_tkip, has_ccmp);
  }
  if (has_wpa3) {
    appendProtocolAuthToken(out, out_len, "RSN", has_psk, has_eap, has_tkip, has_ccmp);
  }

  if (out[0] == '\0') {
    appendAuthToken(out, out_len, "[OPEN]");
  }
  if ((caps & CAP_ESS) != 0) {
    appendAuthToken(out, out_len, "[ESS]");
  }
}

Logger::Logger(SdFat& sd,
               FsFile& log_file,
               TinyGPSPlus& gps,
               HardwareSerial& gnss_uart,
               Stream& serial,
               const Config& config,
               State& state)
    : sd_(sd),
      log_file_(log_file),
      gps_(gps),
      gnss_uart_(gnss_uart),
      serial_(serial),
      config_(config),
      state_(state) {}

void Logger::initUtcTimezone() {
  static bool tz_initialized = false;
  if (tz_initialized) {
    return;
  }

  setenv("TZ", "UTC0", 1);
  tzset();
  tz_initialized = true;
}

void Logger::registerSdDateTimeCallback() {
  date_time_callback_instance_ = this;
  SdFile::dateTimeCallback(sdDateTimeCallbackThunk);
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
  return gps_.date.isValid() &&
         gps_.time.isValid() &&
         gps_.date.year() >= config_.gnss_min_valid_year &&
         gps_.date.month() >= 1 && gps_.date.month() <= 12 &&
         gps_.date.day() >= 1 && gps_.date.day() <= 31 &&
         gps_.time.hour() <= 23 &&
         gps_.time.minute() <= 59 &&
         gps_.time.second() <= 59;
}

bool Logger::gnssDateTimeToTmUtc(struct tm& tm_out) const {
  if (!gnssDateTimeValid()) {
    return false;
  }

  tm_out = {};
  tm_out.tm_year = (int)gps_.date.year() - 1900;
  tm_out.tm_mon = (int)gps_.date.month() - 1;
  tm_out.tm_mday = (int)gps_.date.day();
  tm_out.tm_hour = (int)gps_.time.hour();
  tm_out.tm_min = (int)gps_.time.minute();
  tm_out.tm_sec = (int)gps_.time.second();
  tm_out.tm_isdst = 0;
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
  if (gps_.date.age() >= config_.gps_field_max_age_ms ||
      gps_.time.age() >= config_.gps_field_max_age_ms) {
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
  if (state_.wigle_boot_timestamp[0] != '\0') {
    return true;
  }

  uint32_t epoch = 0;
  if (!masterClockNowEpochSeconds(epoch)) {
    return false;
  }

  const time_t now = (time_t)epoch;
  struct tm tm_utc = {};
  if (gmtime_r(&now, &tm_utc) == nullptr) {
    return false;
  }

  if (strftime(state_.wigle_boot_timestamp, sizeof(state_.wigle_boot_timestamp),
               "%Y%m%d%H%M%S", &tm_utc) == 0) {
    return false;
  }
  serial_.print("Log boot timestamp: ");
  serial_.println(state_.wigle_boot_timestamp);
  return true;
}

void Logger::sdDateTimeCallback(uint16_t* date, uint16_t* time) {
  uint16_t y = 1980;
  uint8_t mon = 1;
  uint8_t d = 1;
  uint8_t h = 0;
  uint8_t min = 0;
  uint8_t s = 0;

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
        gps_.encode(c);
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

bool Logger::selectWigleLogPath() {
  if (!state_.sd_ready || state_.wigle_boot_timestamp[0] == '\0') {
    return false;
  }
  if (state_.wigle_path_selected) {
    return true;
  }

  for (uint16_t suffix = 0; suffix < 100; suffix++) {
    if (suffix == 0) {
      snprintf(state_.wigle_log_path, sizeof(state_.wigle_log_path),
               "/WIGLE_%s.CSV", state_.wigle_boot_timestamp);
    } else {
      snprintf(state_.wigle_log_path, sizeof(state_.wigle_log_path),
               "/WIGLE_%s_%02u.CSV", state_.wigle_boot_timestamp, suffix);
    }
    if (!sd_.exists(state_.wigle_log_path)) {
      state_.wigle_path_selected = true;
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

bool Logger::initWigleCsv(const char* path) {
  const bool exists = sd_.exists(path);
  log_file_ = sd_.open(path, O_RDWR | O_CREAT | O_APPEND);
  if (!log_file_) return false;

  if (!exists || log_file_.size() == 0) {
    log_file_.println("WigleWifi-1.4,appRelease=WiFiShuriken,model=Pico2,release=1.0,device=RP2350,display=none,board=rpipico2,brand=custom");
    log_file_.println("MAC,SSID,AuthMode,FirstSeen,Channel,RSSI,CurrentLatitude,CurrentLongitude,AltitudeMeters,AccuracyMeters,Type");
    log_file_.flush();
  }
  state_.wigle_dirty = false;
  state_.wigle_last_flush_ms = millis();
  return true;
}

void Logger::disableSdLoggingAndScheduleRetry(const char* reason) {
  if (reason && reason[0] != '\0') {
    serial_.println(reason);
  }
  if (log_file_) {
    log_file_.close();
  }
  state_.wigle_ready = false;
  state_.wigle_dirty = false;
  state_.sd_ready = false;
  state_.sd_next_retry_ms = millis() + config_.sd_retry_background_ms;
}

void Logger::tryRecoverSdLogging() {
  if (state_.wigle_ready) {
    return;
  }

  const uint32_t now = millis();
  const uint32_t retry_interval =
      (state_.sd_ready && state_.wigle_boot_timestamp[0] == '\0')
          ? config_.wigle_time_wait_retry_ms
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
    state_.wigle_path_selected = false;
  }

  if (!ensureBootTimestampFromClock()) {
    serial_.println("Waiting for valid GNSS time before creating WiGLE log file");
    return;
  }

  if (!selectWigleLogPath()) {
    serial_.println("WiGLE path selection failed");
    return;
  }

  state_.wigle_ready = initWigleCsv(state_.wigle_log_path);
  if (state_.wigle_ready) {
    serial_.print("WiGLE CSV ready: ");
    serial_.println(state_.wigle_log_path);
  } else {
    serial_.print("WiGLE CSV open failed: ");
    serial_.println(state_.wigle_log_path);
  }
}

void Logger::appendWigleRow(const QueuedScanResult& r) {
  if (!state_.wigle_ready || !log_file_) return;

  char mac[18];
  char ts[20];
  char ssid_csv[80];
  char auth_mode[96];
  char lat_csv[20] = "";
  char lon_csv[20] = "";
  char alt_csv[20] = "";
  char acc_csv[20] = "";

  formatBssid(r.bssid, mac, sizeof(mac));
  formatGpsTimestamp(ts, sizeof(ts));
  wigleCsvEscape(r.ssid, ssid_csv, sizeof(ssid_csv));
  buildAndroidCapabilitiesString(r.capabilities, auth_mode, sizeof(auth_mode));

  const bool have_location =
      gps_.location.isValid() &&
      gps_.location.age() < config_.gps_field_max_age_ms;
  const bool have_altitude =
      gps_.altitude.isValid() &&
      gps_.altitude.age() < config_.gps_field_max_age_ms;
  const bool have_accuracy =
      gps_.hdop.isValid() &&
      gps_.hdop.age() < config_.gps_field_max_age_ms;

  if (have_location) {
    snprintf(lat_csv, sizeof(lat_csv), "%.7f", gps_.location.lat());
    snprintf(lon_csv, sizeof(lon_csv), "%.7f", gps_.location.lng());
  }
  if (have_altitude) {
    snprintf(alt_csv, sizeof(alt_csv), "%.2f", gps_.altitude.meters());
  }
  if (have_accuracy) {
    snprintf(acc_csv, sizeof(acc_csv), "%.2f", gps_.hdop.hdop() * 5.0);
  }
  if (!have_location) {
    state_.wigle_rows_blank_gps++;
  }

  log_file_.printf("%s,%s,%s,%s,%u,%d,%s,%s,%s,%s,WIFI\n",
                   mac,
                   ssid_csv,
                   auth_mode,
                   ts,
                   (unsigned)r.channel,
                   (int)r.rssi,
                   lat_csv,
                   lon_csv,
                   alt_csv,
                   acc_csv);
  if (log_file_.getWriteError()) {
    disableSdLoggingAndScheduleRetry("WiGLE write error; disabling SD logging until recovery");
    return;
  }
  state_.wigle_dirty = true;

  state_.wigle_rows++;
  if ((state_.wigle_rows % 25) == 0) {
    log_file_.flush();
    if (log_file_.getWriteError()) {
      disableSdLoggingAndScheduleRetry("WiGLE flush error; disabling SD logging until recovery");
      return;
    }
    state_.wigle_dirty = false;
    state_.wigle_last_flush_ms = millis();
  }
}

void Logger::flushWigleIfDue() {
  if (!state_.wigle_ready || !log_file_ || !state_.wigle_dirty) {
    return;
  }
  const uint32_t now = millis();
  if ((int32_t)(now - state_.wigle_last_flush_ms) < (int32_t)config_.wigle_flush_interval_ms) {
    return;
  }

  log_file_.flush();
  if (log_file_.getWriteError()) {
    disableSdLoggingAndScheduleRetry("WiGLE periodic flush error; disabling SD logging until recovery");
    return;
  }
  state_.wigle_dirty = false;
  state_.wigle_last_flush_ms = now;
}

}  // namespace pico_logging

