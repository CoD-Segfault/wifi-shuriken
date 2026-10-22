#pragma once
#include "controller/application.h"
#include "controller/phone_gnss.h"
#include "controller/gps_debug.h"
#include "controller_logging.h"
#include "controller/scanner_runtime.h"
#include <atomic>

namespace controller {
// Single-producer/single-consumer queues: scanner worker writes; application
// reads. Fixed storage, no allocations, locks or waits on either core.
template<class T, size_t Capacity> class BoundedQueue {
 public:
  bool push(const T& value) {
    const uint32_t head = head_.load(std::memory_order_relaxed);
    if (uint32_t(head - tail_.load(std::memory_order_acquire)) == Capacity) return false;
    items_[head % Capacity] = value;
    head_.store(head + 1, std::memory_order_release); return true;
  }
  bool pop(T& value) {
    const uint32_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) return false;
    value = items_[tail % Capacity];
    tail_.store(tail + 1, std::memory_order_release); return true;
  }
  unsigned size() const { return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire); }
 private:
  static_assert((Capacity & (Capacity - 1)) == 0, "Queue capacity must be a power of two for counter rollover");
  T items_[Capacity] = {};
  std::atomic<uint32_t> head_{0}, tail_{0};
};
class OutputStream final : public Stream {
 public:
  Output* output = nullptr;
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  void flush() override {}
  int availableForWrite() override { return 2048; }
  size_t write(uint8_t byte) override { return write(&byte, 1); }
  size_t write(const uint8_t* data, size_t length) override {
    if (output) output->write(data, length);
    return length;
  }
};
class RuntimeScanners final : public Scanners {
 public:
  explicit RuntimeScanners(System& system) : system_(system) {}
  bool begin() override;
  bool updateFirmware(ScannerUpdateStorage& storage, Output& output, const ScannerUpdateConfig& config) override;
  bool result(controller::ScanResult& result) override { return results_.pop(result); }
  bool diagnostic(char* text, size_t capacity) override;
  void resetDedupe() override { controllerScannerRuntimeRequestDedupeReset(); }
  ScannerStatus status() const override;
 private:
  static RuntimeScanners* instance_;
  static void worker();
  struct Diagnostic { char text[192]; };
  System& system_;
  BoundedQueue<controller::ScanResult, 1024> results_;
  BoundedQueue<Diagnostic, 64> diagnostics_;
  WiFiDedupeHash dedupe_slots_[WIFI_DEDUPE_TABLE_SIZE] = {};
  WiFiDedupeHash dedupe_fifo_[WIFI_DEDUPE_TABLE_CAPACITY] = {};
  WiFiDedupeTable dedupe_ = {};
  uint32_t drops_ = 0, dedupe_drops_ = 0;
  volatile uint32_t sweeps_ = 0, sweep_ms_ = 0, heartbeat_ = 0, present_ = 0;
  std::atomic<unsigned> state_{0};
};
struct DataOptions {
  unsigned mount_attempts = 1;
  uint8_t reset_code = 0;
  const char* reset_reason = "UNKNOWN";
};
class RuntimeData final : public Data {
 public:
  RuntimeData(System& system, controller::logging::LogStorage& storage,
              Stream& uart, Stream& output, const controller::logging::Config& config,
              DataOptions options)
    : system_(system), output_(output), config_(config), options_(options),
      logger_(storage, system.clock(), system.utcClock(), gps_, uart, output, config_, state_) {}
  void begin() override;
  void beginNavigation() override { system_.beginNavigation(); }
  void captureBootTime() override { logger_.captureBootTimestamp(); }
  void pollNavigation(const LinkStatus& link) override;
  bool receivePhone(const PhoneMessage& message, const LinkStatus& link) override;
  void selectSource(unsigned source) override;
  DataStatus status() const override;
  void syncClock() override;
  void recover() override { logger_.tryRecoverLogging(); }
  void appendReset() override { logger_.tryAppendBootResetLog(); }
  void append(const controller::ScanResult& result) override { logger_.appendCsvRow(result); }
  void flush() override { logger_.flushCsvIfDue(); }
  void flushNow() override { logger_.flushNow(); }
  void debugStart(bool watch) override;
  void debugStop() override;
  void debugPoll() override;
 private:
  System& system_;
  Stream& output_;
  const controller::logging::Config& config_;
  DataOptions options_;
  TinyGPSPlus gps_, gps_usb_;
  BleGnss gps_ble_;
  bool fix_[3] = {};
  unsigned selected_ = 0;
  controller::logging::State state_;
  controller::logging::Logger logger_;
  GpsDebugReport report_;
  bool debug_pending_ = false, debug_watch_ = false;
  size_t debug_line_ = 0;
  uint32_t debug_started_ = 0, debug_captured_ = 0, debug_last_line_ = 0;
};
}  // namespace controller
