#include "controller/application.h"
#include <cstdio>
#include <cstring>

namespace controller {
void Output::text(const char* value, bool mirror) {
  uint8_t buffer[256];
  size_t count = 0;
  char previous = 0;
  while (*value) {
    const char byte = *value++;
    if (count + 2 > sizeof(buffer)) { write(buffer, count, mirror); count = 0; }
    if (byte == '\n' && previous != '\r') buffer[count++] = '\r';
    buffer[count++] = static_cast<uint8_t>(byte); previous = byte;
  }
  if (count) write(buffer, count, mirror);
}
void Application::write(const uint8_t* data, size_t length, bool mirror) {
  auto& link = services_.system.wireless();
  const auto state = link.status();
  if (mirror && state.connected && state.subscribed) link.send(data, length);
  if (length > sizeof(console_buffer_) - used_) return;
  for (size_t i = 0; i < length; ++i) {
    console_buffer_[head_] = data[i]; head_ = (head_ + 1) % sizeof(console_buffer_);
  }
  used_ += length;
}
void Application::pumpConsole() {
  if (!used_) return;
  size_t count = sizeof(console_buffer_) - tail_;
  if (count > used_) count = used_;
  if (count > 64) count = 64;
  const int written = services_.system.console().write(console_buffer_ + tail_, count);
  if (written <= 0 || static_cast<size_t>(written) > count) return;
  tail_ = (tail_ + written) % sizeof(console_buffer_); used_ -= written;
}
bool Application::begin() {
  if (attempted_) return active_;
  attempted_ = true;
  system_ready_ = services_.system.begin(*this);
  if (!system_ready_) { text("Controller hardware initialization failed\r\n"); return false; }
  text("WiFi Shuriken controller startup\r\n");
  services_.data.begin();
  if (services_.system.bootMaintenance(services_.data.status().storage_ready)) return false;
  if (services_.system.capabilities().scanner_update && services_.policy.scanner_update.enabled && services_.data.status().storage_ready) {
    if (auto* storage = services_.system.scannerUpdateStorage())
      services_.scanners.updateFirmware(*storage, *this, services_.policy.scanner_update);
    else text("Scanner update storage unavailable\r\n");
  }
  services_.data.beginNavigation();
  if (services_.policy.capture_boot_time) services_.data.captureBootTime();
  if (services_.system.tryStorageLock()) {
    services_.data.recover(); services_.data.appendReset();
    services_.system.unlockStorage(services_.data.status().storage_ready);
  }
  active_ = services_.scanners.begin();
  if (!active_) { text("Scanner worker unavailable; diagnostics remain available\r\n"); return false; }
  services_.system.beginWatchdog(services_.scanners.status().heartbeat);
  printStatus();
  return true;
}
void Application::poll() {
  if (!system_ready_) return;
  pumpConsole();
  services_.system.poll(services_.scanners.status().heartbeat);
  serviceConsole();
  auto& wireless = services_.system.wireless();
  wireless.poll();
  PhoneMessage message;
  for (unsigned i = 0; i < 2 && wireless.receive(message); ++i) {
    if (message.length > sizeof(message.data)) continue;
    const auto link = wireless.status();
    if (!link.connected || message.session != link.session) continue;
    if (!services_.data.receivePhone(message, link)) {
      text("BLE RX: ", false); write(message.data, message.length, false); text("\r\n", false);
    }
  }
  if (active_) {
    services_.data.pollNavigation(wireless.status());
    const auto navigation = services_.data.status();
    selected_source_ = navigation.fix[0] ? 0 : navigation.fix[1] ? 1 : navigation.fix[2] ? 2 : 0;
    services_.data.selectSource(selected_source_);
    const bool fix = navigation.fix[0] || navigation.fix[1] || navigation.fix[2];
    const uint32_t now = services_.system.clock().milliseconds();
    if (services_.policy.relog_on_fix && fix && !last_fix_ &&
        (!reset_once_ || uint32_t(now - last_reset_) >= services_.policy.reset_cooldown_ms)) {
      services_.scanners.resetDedupe(); reset_once_ = true; last_reset_ = now;
    }
    last_fix_ = fix;
    services_.data.syncClock();
    char diagnostic[192];
    for (unsigned i = 0; i < services_.policy.diagnostic_budget &&
         services_.scanners.diagnostic(diagnostic, sizeof(diagnostic)); ++i) text(diagnostic);
    if (services_.system.tryStorageLock()) {
      services_.data.recover();
      controller::ScanResult result;
      for (unsigned i = 0; i < services_.policy.result_budget &&
           services_.data.status().logging && services_.scanners.result(result); ++i)
        services_.data.append(result);
      services_.data.flush(); services_.data.appendReset();
      if (services_.system.resetRequested()) {
        services_.data.flushNow();
        services_.system.unlockStorage(services_.data.status().storage_ready);
        services_.system.reboot();
        return;
      }
      services_.system.unlockStorage(services_.data.status().storage_ready);
    }
    const auto state = services_.data.status();
    services_.system.indicate(state.storage_ready, fix, state.logging);
    services_.data.debugPoll();
    if (services_.policy.periodic_status_ms && uint32_t(now - last_status_) >= services_.policy.periodic_status_ms) {
      last_status_ = now; printStatus();
    }
  }
  serviceStorageTest();
  services_.system.idle();
}
void Application::printStatus() {
  const auto scanner = services_.scanners.status();
  const auto data = services_.data.status();
  const auto link = services_.system.wireless().status();
  const auto caps = services_.system.capabilities();
  unsigned present = 0;
  for (unsigned i = 0; i < scanner.slots; ++i) if (scanner.present & (1u << i)) ++present;
  char line[384];
  std::snprintf(line, sizeof(line),
    "Scanner worker=%u; present=%u/%u; queue=%u; drops=%lu; dedupe=%lu; heartbeat age=%lu ms; Storage=%s; CSV=%s; rows=%lu; blank_gps=%lu; sweeps=%lu; last_sweep_ms=%lu\r\n",
    scanner.state, present, scanner.slots, scanner.queued, static_cast<unsigned long>(scanner.drops),
    static_cast<unsigned long>(scanner.dedupe_drops),
    static_cast<unsigned long>(services_.system.clock().milliseconds() - scanner.heartbeat),
    data.storage_ready ? "ready" : "unavailable", data.logging ? "logging" : "waiting",
    static_cast<unsigned long>(data.rows), static_cast<unsigned long>(data.blank_rows),
    static_cast<unsigned long>(scanner.sweeps), static_cast<unsigned long>(scanner.sweep_ms));
  text(line);
  const char* names[] = {"UART", "USB", "BLE"};
  std::snprintf(line, sizeof(line), "GPS source=%s; UART=%s; USB=%s; BLE=%s; clock=%s; phone=%s; RX drops=%lu; TX drops=%lu\r\n",
    last_fix_ ? names[selected_source_] : "none", data.fix[0] ? "fix" : "no fix",
    !caps.usb_nmea ? "unavailable" : data.fix[1] ? "fix" : "no fix",
    !caps.ble_nmea ? "unavailable" : data.fix[2] ? "fix" : "no fix",
    data.clock_valid ? "valid" : "unset", link.connected ? "connected" : "disconnected",
    static_cast<unsigned long>(link.rx_drops), static_cast<unsigned long>(link.tx_drops));
  text(line);
  if (data.logging) { text("Log: "); text(data.path); text("\r\n"); }
}
void Application::handleLine(const uint8_t* data, size_t length) {
  if (!length) return;
  if (data[0] != ':') { services_.system.wireless().send(data, length); return; }
  char command[129];
  std::memcpy(command, data, length); command[length] = '\0';
  while (length && (command[length - 1] == '\r' || command[length - 1] == '\n')) command[--length] = '\0';
  if (std::strcmp(command, ":status") == 0) printStatus();
  else if (std::strcmp(command, ":gps-debug") == 0) services_.data.debugStart(false);
  else if (std::strcmp(command, ":gps-debug on") == 0) services_.data.debugStart(true);
  else if (std::strcmp(command, ":gps-debug off") == 0) services_.data.debugStop();
  else if (std::strcmp(command, ":storage-test") == 0 || std::strcmp(command, ":sd-test") == 0) {
    if (storage_test_.active) text("Storage test already running\r\n");
    else if (!services_.data.status().storage_ready) text("Storage test unavailable: storage is not ready\r\n");
    else {
      storage_test_ = {}; storage_test_.active = true;
      storage_test_.start = storage_test_.next = services_.system.clock().milliseconds();
      text("Storage test started: repeated 4096-byte checks for 10 seconds\r\n");
    }
  } else text("Local commands: :status, :storage-test, :gps-debug [on|off]\r\n");
}
void Application::serviceConsole() {
  const uint32_t now = services_.system.clock().milliseconds();
  for (unsigned i = 0; i < 128; ++i) {
    uint8_t byte;
    if (services_.system.console().read(&byte, 1) != 1) break;
    if (discard_line_) { if (byte == '\n') discard_line_ = false; continue; }
    line_[line_length_++] = byte; last_byte_ = now;
    if (byte == '\n') { handleLine(line_, line_length_); line_length_ = 0; }
    else if (line_length_ == sizeof(line_)) {
      if (line_[0] == ':') { discard_line_ = true; text("Local command too long; discarded\r\n"); }
      else handleLine(line_, line_length_);
      line_length_ = 0;
    }
  }
  if (line_length_ && line_[0] != ':' && uint32_t(now - last_byte_) >= 50) {
    handleLine(line_, line_length_); line_length_ = 0;
  }
}
void Application::serviceStorageTest() {
  if (!storage_test_.active) return;
  const uint32_t now = services_.system.clock().milliseconds();
  if (uint32_t(now - storage_test_.start) < 10000) {
    if (static_cast<int32_t>(now - storage_test_.next) < 0) return;
    if (!services_.system.tryStorageLock()) return;
    const bool passed = services_.system.verifyStorage();
    services_.system.unlockStorage(services_.data.status().storage_ready);
    if (passed) ++storage_test_.passed;
    else { ++storage_test_.failed; storage_test_.active = false; }
    storage_test_.next = services_.system.clock().milliseconds() + 100;
    if (passed && uint32_t(now - storage_test_.last_report) < 1000) return;
    storage_test_.last_report = now;
  } else storage_test_.active = false;
  char line[192];
  std::snprintf(line, sizeof(line), "Storage test %s: %lu passed, %lu failed, %lu KiB verified\r\n",
    storage_test_.active ? "running" : storage_test_.failed || !storage_test_.passed ? "FAIL" : "PASS",
    static_cast<unsigned long>(storage_test_.passed), static_cast<unsigned long>(storage_test_.failed),
    static_cast<unsigned long>(storage_test_.passed * 4)); text(line);
}
Application& application() { static Application instance(controllerServices()); return instance; }
}  // namespace controller
