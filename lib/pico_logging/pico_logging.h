#pragma once

#include <Arduino.h>
#include <SdFat.h>
#include <TinyGPSPlus.h>

#include <stddef.h>
#include <stdint.h>

#include "../log_format/log_format.h"

namespace pico_logging {

struct QueuedScanResult {
  char ssid[33];
  uint8_t bssid[6];
  int8_t rssi;
  uint8_t channel;
  uint8_t band;
  uint16_t capabilities;
};

struct Config {
  uint8_t sd_pin_cs;
  const uint32_t* sd_spi_clocks_hz;
  size_t sd_spi_clock_count;
  uint32_t sd_init_retry_delay_ms;
  uint32_t sd_retry_background_ms;
  uint32_t gnss_boot_timestamp_wait_ms;
  uint8_t gnss_boot_timestamp_reads;
  uint32_t gps_field_max_age_ms;
  uint32_t csv_time_wait_retry_ms;
  uint32_t csv_flush_interval_ms;
  uint16_t gnss_min_valid_year;
};

struct State {
  bool sd_ready = false;
  bool csv_ready = false;
  bool csv_dirty = false;
  bool csv_path_selected = false;
  bool master_clock_valid = false;
  bool boot_reset_log_pending = false;
  bool boot_reset_log_appended = false;
  uint32_t sd_next_retry_ms = 0;
  size_t sd_retry_clock_index = 0;
  uint32_t csv_last_flush_ms = 0;
  uint32_t csv_rows = 0;
  uint32_t csv_rows_blank_gps = 0;
  uint8_t boot_reset_reason_code = 0;
  char boot_reset_reason[24] = "";
  char csv_boot_timestamp[15] = "";
  char csv_log_path[40] = "";
};

class Logger {
 public:
  Logger(SdFat& sd,
         FsFile& log_file,
         TinyGPSPlus& gps,
         HardwareSerial& gnss_uart,
         Stream& serial,
         const Config& config,
         State& state);

  // Switch the GPS source used for location fields in CSV rows and clock sync.
  // Pass &gps (hardware) normally; pass &gps_phone when falling back to the
  // phone-sourced fix. Must not be called with nullptr.
  void setGpsSource(TinyGPSPlus* gps);

  void initUtcTimezone();
  void registerSdDateTimeCallback();
  void setBootResetReason(uint8_t reason_code, const char* reason_text);
  void tryAppendBootResetLog();

  void syncMasterClockFromGnss();
  bool ensureBootTimestampFromClock();
  void captureBootTimestampFromGnss();

  bool selectCsvLogPath();
  bool mountSdWithRetries(int max_attempts);
  bool initCsvLog(const char* path);

  void disableSdLoggingAndScheduleRetry(const char* reason);
  void tryRecoverSdLogging();

  void appendCsvRow(const QueuedScanResult& r);
  void flushCsvIfDue();

 private:
  static Logger* date_time_callback_instance_;
  static void sdDateTimeCallbackThunk(uint16_t* date, uint16_t* time);
  void sdDateTimeCallback(uint16_t* date, uint16_t* time);

  bool gnssDateTimeValid() const;
  bool gnssDateTimeToTmUtc(struct tm& tm_out) const;
  bool systemClockEpochLooksPlausible(time_t epoch) const;
  bool recoverMasterClockFromSystem();
  bool masterClockNowEpochSeconds(uint32_t& epoch_out) const;
  void formatGpsTimestamp(char* out, size_t out_len) const;

  bool beginSdAtClock(uint32_t hz);
  bool beginSdAtAnyClock();

  SdFat& sd_;
  FsFile& log_file_;
  TinyGPSPlus* gps_ptr_;
  HardwareSerial& gnss_uart_;
  Stream& serial_;
  const Config& config_;
  State& state_;
};

}  // namespace pico_logging


