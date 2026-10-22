#pragma once

#include <Arduino.h>
#include "hal/log_storage.h"
#include <TinyGPSPlus.h>
#include "controller/scan_result.h"

#include <stddef.h>
#include <stdint.h>
#include "hal/time.h"

#include "../log_format/log_format.h"

namespace controller::logging {

struct Config {
  uint32_t storage_init_retry_delay_ms;
  uint32_t storage_retry_background_ms;
  uint32_t gnss_boot_timestamp_wait_ms;
  uint8_t gnss_boot_timestamp_reads;
  uint32_t gps_field_max_age_ms;
  uint32_t csv_time_wait_retry_ms;
  uint32_t csv_flush_interval_ms;
  uint16_t min_valid_year;
  bool require_valid_time = true;
};

struct State {
  bool storage_ready = false;
  bool csv_ready = false;
  bool csv_dirty = false;
  bool csv_path_selected = false;
  bool master_clock_valid = false;
  bool boot_reset_log_pending = false;
  bool boot_reset_log_appended = false;
  uint32_t storage_next_retry_ms = 0;
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
  Logger(LogStorage& storage,
         hal::Clock& monotonic,
         hal::UtcClock& utc,
         TinyGPSPlus& gps,
         Stream& gnss_uart,
         Stream& serial,
         const Config& config,
         State& state);

  // Switch the GPS source used for location fields in CSV rows and clock sync.
  // Pass &gps (hardware) normally; pass &gps_phone when falling back to the
  // phone-sourced fix. Must not be called with nullptr.
  void setGpsSource(TinyGPSPlus* gps);

  void initializeClock();
  void registerStorageTimestamp();
  void setBootResetReason(uint8_t reason_code, const char* reason_text);
  void tryAppendBootResetLog();

  bool syncMasterClockFromGnss(TinyGPSPlus* source = nullptr);
  bool ensureBootTimestampFromClock();
  void captureBootTimestamp();

  bool selectCsvLogPath();
  bool mountStorageWithRetries(int max_attempts);
  bool initCsvLog(const char* path);

  void disableLoggingAndScheduleRetry(const char* reason);
  void tryRecoverLogging();

  void appendCsvRow(const ScanResult& r);
  void flushCsvIfDue();
  void flushNow();

 private:
  bool storageTimestamp(UtcTimestamp& timestamp);

  bool gnssDateTime(hal::UtcDateTime& utc, TinyGPSPlus* source = nullptr) const;
  bool systemClockEpochLooksPlausible(hal::EpochSeconds epoch) const;
  bool recoverMasterClockFromSystem();
  bool masterClockNowEpochSeconds(hal::EpochSeconds& epoch_out) const;
  void formatTimestamp(char* out, size_t out_len) const;

  LogStorage& storage_;
  hal::Clock& monotonic_;
  hal::UtcClock& utc_;
  hal::EpochSeconds last_gnss_epoch_ = 0;
  bool gnss_synchronized_ = false, sync_failed_ = false;
  uint32_t last_sync_attempt_ms_ = 0;
  std::unique_ptr<LogFile> log_file_;
  TinyGPSPlus* gps_ptr_;
  Stream& gnss_uart_;
  Stream& serial_;
  const Config& config_;
  State& state_;
};

}  // namespace controller::logging


