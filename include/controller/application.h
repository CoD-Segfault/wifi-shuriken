#pragma once
#include "hal/controller.h"
#include "hal/time.h"
#include "controller/scan_result.h"
#include "controller/scanner_update.h"
#include <stddef.h>
#include <stdint.h>

namespace controller {
class Output {
 public:
  virtual ~Output() = default;
  virtual void write(const uint8_t* data, size_t length, bool mirror = true) = 0;
  void text(const char* text, bool mirror = true);
};
struct LinkStatus {
  bool ready = false, connected = false, subscribed = false;
  uint32_t session = 0, rx_drops = 0, tx_drops = 0;
};
struct PhoneMessage { uint8_t data[512]; size_t length = 0; uint32_t session = 0; };
class Wireless {
 public:
  virtual ~Wireless() = default;
  virtual void poll() = 0;
  virtual LinkStatus status() const = 0;
  virtual bool receive(PhoneMessage& message) = 0;
  virtual size_t send(const uint8_t* data, size_t length) = 0;
};
struct DataStatus {
  bool storage_ready = false, logging = false, clock_valid = false;
  bool fix[3] = {false, false, false};  // UART, USB, BLE; policy uses this order.
  uint32_t rows = 0, blank_rows = 0;
  const char* path = "";
};
class Data {
 public:
  virtual ~Data() = default;
  virtual void begin() = 0;
  virtual void beginNavigation() = 0;
  virtual void captureBootTime() = 0;
  virtual void pollNavigation(const LinkStatus& link) = 0;
  virtual bool receivePhone(const PhoneMessage& message, const LinkStatus& link) = 0;
  virtual void selectSource(unsigned source) = 0;
  virtual DataStatus status() const = 0;
  virtual void syncClock() = 0;
  virtual void recover() = 0;
  virtual void appendReset() = 0;
  virtual void append(const controller::ScanResult& result) = 0;
  virtual void flush() = 0;
  virtual void flushNow() = 0;
  virtual void debugStart(bool watch) = 0;
  virtual void debugStop() = 0;
  virtual void debugPoll() = 0;
};
struct ScannerStatus {
  unsigned state = 0, slots = 0, present = 0, queued = 0;
  uint32_t drops = 0, dedupe_drops = 0, heartbeat = 0, sweeps = 0, sweep_ms = 0;
};
class Scanners {
 public:
  virtual ~Scanners() = default;
  virtual bool begin() = 0;
  virtual bool updateFirmware(ScannerUpdateStorage&, Output&, const ScannerUpdateConfig&) = 0;
  virtual bool result(controller::ScanResult& result) = 0;
  virtual bool diagnostic(char* text, size_t capacity) = 0;
  virtual void resetDedupe() = 0;
  virtual ScannerStatus status() const = 0;
};
class System {
 public:
  virtual ~System() = default;
  virtual bool begin(Output& output) = 0;
  virtual hal::Clock& clock() = 0;
  virtual hal::UtcClock& utcClock() = 0;
  virtual hal::ByteStream& console() = 0;
  virtual Wireless& wireless() = 0;
  virtual hal::Capabilities capabilities() const = 0;
  // Only backend operations here: peripherals, bootloader and worker lifecycle.
  virtual bool bootMaintenance(bool storage_ready) = 0;
  virtual ScannerUpdateStorage* scannerUpdateStorage() { return nullptr; }
  virtual void scannerUpdateStarting() {}
  virtual hal::ScannerTransport* beginScannerUpdate() { return nullptr; }
  virtual void endScannerUpdate() {}
  virtual void beginNavigation() = 0;
  virtual hal::ByteStream* usbNmea() { return nullptr; }
  virtual void mirrorNmea(uint8_t) {}
  virtual bool launchScanner(void (*entry)()) = 0;
  virtual bool scannerRunning() const { return true; }
  virtual void beginWatchdog(uint32_t heartbeat) = 0;
  virtual void poll(uint32_t heartbeat) = 0;
  virtual bool tryStorageLock() = 0;
  virtual void unlockStorage(bool ready) = 0;
  virtual bool resetRequested() = 0;
  virtual void reboot() = 0;
  virtual void indicate(bool storage, bool fix, bool logging) = 0;
  virtual bool verifyStorage() = 0;
  virtual void idle() = 0;
};
struct Policy {
  unsigned result_budget = 64, diagnostic_budget = 16;
  uint32_t reset_cooldown_ms = 5000, periodic_status_ms = 0;
  bool relog_on_fix = true, capture_boot_time = false;
  ScannerUpdateConfig scanner_update;
};
struct Services { System& system; Data& data; Scanners& scanners; Policy policy; };
class Application final : public Output {
 public:
  explicit Application(Services services) : services_(services) {}
  bool begin();
  void poll();
  void write(const uint8_t* data, size_t length, bool mirror = true) override;
  void printStatus();
 private:
  void pumpConsole();
  void serviceConsole();
  void handleLine(const uint8_t* data, size_t length);
  void serviceStorageTest();
  Services services_;
  bool attempted_ = false, system_ready_ = false, active_ = false, last_fix_ = false, reset_once_ = false;
  uint32_t last_reset_ = 0, last_status_ = 0;
  unsigned selected_source_ = 0;
  uint8_t console_buffer_[2048] = {}, line_[128] = {};
  size_t head_ = 0, tail_ = 0, used_ = 0, line_length_ = 0;
  bool discard_line_ = false;
  uint32_t last_byte_ = 0;
  struct StorageTest {
    bool active = false;
    uint32_t start = 0, next = 0, last_report = 0, passed = 0, failed = 0;
  } storage_test_;
};
Services& controllerServices();
Application& application();
}  // namespace controller
