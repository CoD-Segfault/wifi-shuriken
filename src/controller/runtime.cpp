#include "controller/runtime.h"
#include "hal/platform.h"
#include <cstdio>
#include <cstring>

namespace controller {
RuntimeScanners* RuntimeScanners::instance_ = nullptr;
bool RuntimeScanners::updateFirmware(ScannerUpdateStorage& storage, Output& output, const ScannerUpdateConfig& config) {
  class Observer final : public ScannerUpdateObserver {
   public:
    explicit Observer(System& system) : system_(system) {}
    void started() override { system_.scannerUpdateStarting(); }
   private:
    System& system_;
  } observer(system_);
  if (!config.enabled || !config.image_path || !storage.exists(config.image_path)) return false;
  auto* transport = system_.beginScannerUpdate();
  if (!transport) { output.text("Scanner update transport unavailable\r\n"); return false; }
  const bool updated = updateScannerFirmware(storage, *transport, system_.clock(), output, config, &observer);
  system_.endScannerUpdate();
  return updated;
}
bool RuntimeScanners::begin() {
  instance_ = this;
  wifiDedupeTableInit(&dedupe_, dedupe_slots_, WIFI_DEDUPE_TABLE_SIZE, dedupe_fifo_, WIFI_DEDUPE_TABLE_CAPACITY);
  // Seed heartbeat before enabling a platform watchdog.
  heartbeat_ = system_.clock().milliseconds();
  if (!system_.launchScanner(worker)) { state_ = 2; return false; }
  return true;
}
void RuntimeScanners::worker() {
  auto& self = *instance_;
  if (!controllerScannerRuntimeInitBus()) {
    self.state_ = 2;
    Diagnostic message = {}; std::snprintf(message.text, sizeof(message.text), "Scanner bus initialization failed\r\n");
    self.diagnostics_.push(message); return;
  }
  const ControllerScannerRuntimeContext context = {
    [](const controller::ScanResult& result) { return instance_->results_.push(result); },
    &self.drops_, &self.dedupe_drops_, &self.sweeps_, &self.sweep_ms_, &self.dedupe_,
    [](const char* text) {
      Diagnostic message = {}; std::snprintf(message.text, sizeof(message.text), "%s", text);
      instance_->diagnostics_.push(message);
    }, &self.heartbeat_, &self.present_
  };
  self.state_ = 1;
  controllerScannerRuntimeStart(context);
  while (self.system_.scannerRunning()) controllerScannerRuntimePoll();
}
bool RuntimeScanners::diagnostic(char* text, size_t capacity) {
  Diagnostic message;
  if (!diagnostics_.pop(message)) return false;
  std::snprintf(text, capacity, "%s", message.text); return true;
}
ScannerStatus RuntimeScanners::status() const {
  return {state_.load(), hal::scannerTransport().slotCount(), __atomic_load_n(&present_, __ATOMIC_RELAXED),
    results_.size(), __atomic_load_n(&drops_, __ATOMIC_RELAXED), __atomic_load_n(&dedupe_drops_, __ATOMIC_RELAXED),
    __atomic_load_n(&heartbeat_, __ATOMIC_RELAXED), __atomic_load_n(&sweeps_, __ATOMIC_RELAXED),
    __atomic_load_n(&sweep_ms_, __ATOMIC_RELAXED)};
}
void RuntimeData::begin() {
  logger_.initializeClock(); logger_.registerStorageTimestamp();
  logger_.setBootResetReason(options_.reset_code, options_.reset_reason);
  state_.storage_ready = logger_.mountStorageWithRetries(options_.mount_attempts);
  if (!state_.storage_ready) state_.storage_next_retry_ms = system_.clock().milliseconds() + config_.storage_retry_background_ms;
}
void RuntimeData::pollNavigation(const LinkStatus& link) {
  uint8_t byte;
  for (unsigned i = 0; i < 512 && hal::gnssStream().read(&byte, 1) == 1; ++i) {
    gps_.encode(static_cast<char>(byte)); system_.mirrorNmea(byte);
  }
  if (auto* usb = system_.usbNmea())
    for (unsigned i = 0; i < 256 && usb->read(&byte, 1) == 1; ++i) gps_usb_.encode(static_cast<char>(byte));
  gps_ble_.updateSession(link.connected, link.session, link.rx_drops);
  fix_[0] = gps_.location.isValid() && gps_.location.age() < config_.gps_field_max_age_ms &&
            gps_.satellites.isValid() && gps_.satellites.value() >= 3;
  fix_[1] = system_.capabilities().usb_nmea && gps_usb_.location.isValid() && gps_usb_.location.age() < config_.gps_field_max_age_ms;
  fix_[2] = system_.capabilities().ble_nmea && gps_ble_.usable(config_.gps_field_max_age_ms);
}
bool RuntimeData::receivePhone(const PhoneMessage& message, const LinkStatus& link) {
  gps_ble_.updateSession(link.connected, link.session, link.rx_drops);
  if (!link.connected || message.session != link.session) return true;
  return gps_ble_.feed(message.data, message.length, message.session);
}
void RuntimeData::selectSource(unsigned source) {
  selected_ = source < 3 ? source : 0;
  logger_.setGpsSource(selected_ == 0 ? &gps_ : selected_ == 1 ? &gps_usb_ : &gps_ble_.parser());
}
DataStatus RuntimeData::status() const {
  return {state_.storage_ready, state_.csv_ready, state_.master_clock_valid,
    {fix_[0], fix_[1], fix_[2]}, state_.csv_rows, state_.csv_rows_blank_gps, state_.csv_log_path};
}
void RuntimeData::syncClock() {
  logger_.initializeClock();
  // Clock validity does not require a location fix. Try fresh GNSS times in
  // source priority order independently of the parser used for CSV coordinates.
  if (logger_.syncMasterClockFromGnss(&gps_)) return;
  if (system_.capabilities().usb_nmea && logger_.syncMasterClockFromGnss(&gps_usb_)) return;
  if (system_.capabilities().ble_nmea && system_.wireless().status().connected)
    logger_.syncMasterClockFromGnss(&gps_ble_.parser());
}

void RuntimeData::debugStart(bool watch) {
  debug_watch_ = watch; debug_pending_ = true; debug_started_ = system_.clock().milliseconds();
}
void RuntimeData::debugStop() { debug_watch_ = debug_pending_ = false; debug_line_ = report_.size(); }
void RuntimeData::debugPoll() {
  const uint32_t now = system_.clock().milliseconds();
  if (debug_watch_ && uint32_t(now - debug_started_) >= 30000) {
    debug_watch_ = false; output_.println("GPS DEBUG watch complete");
  }
  if (debug_pending_ || (debug_watch_ && debug_line_ == report_.size() && uint32_t(now - debug_captured_) >= 2000)) {
    const auto caps = system_.capabilities();
    const GpsDebugSource sources[] = {{"UART", &gps_, true, fix_[0]}, {"USB", &gps_usb_, caps.usb_nmea, fix_[1]},
                                     {"BLE", &gps_ble_.parser(), caps.ble_nmea, fix_[2]}};
    const char* names[] = {"UART", "USB", "BLE"};
    report_.capture(sources, fix_[0] || fix_[1] || fix_[2] ? names[selected_] : "none", config_.gps_field_max_age_ms, now);
    debug_pending_ = false; debug_line_ = 0; debug_captured_ = now; debug_last_line_ = now - 100;
  }
  // Output is supplied through the platform's shared nonblocking stream.
  if (debug_line_ < report_.size() && uint32_t(now - debug_last_line_) >= 100) {
    // Keep diagnostics observational: capture() works on parser copies.
    output_.print(report_.line(debug_line_++)); debug_last_line_ = now;
  }
}
}  // namespace controller
