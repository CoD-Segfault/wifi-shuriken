#pragma once
#include "controller/application.h"
#include <algorithm>
#include <cassert>
#include <cstring>
#include <deque>
#include <string>
#include <vector>
namespace fixture {
struct Clock : controller::hal::Clock {
  uint32_t now = 0;
  uint32_t milliseconds() const override { return now; }
  uint32_t microseconds() const override { return now * 1000; }
  uint64_t microseconds64() const override { return uint64_t(now) * 1000; }
  void sleepMicroseconds(uint32_t duration) override { now += duration / 1000; }
};
struct Stream : controller::hal::ByteStream {
  std::deque<uint8_t> input;
  std::string output;
  size_t writable = 64;
  void feed(const std::string& text) { for (auto byte : text) input.push_back(byte); }
  int read(uint8_t* data, size_t length) override {
    size_t count = 0;
    while (count < length && !input.empty()) { data[count++] = input.front(); input.pop_front(); }
    return count;
  }
  int write(const uint8_t* data, size_t length) override {
    const size_t count = std::min(length, writable);
    output.append(reinterpret_cast<const char*>(data), count); return count;
  }
};
struct Wireless : controller::Wireless {
  controller::LinkStatus state;
  std::deque<controller::PhoneMessage> messages;
  std::string output;
  unsigned polls = 0;
  void poll() override { ++polls; }
  controller::LinkStatus status() const override { return state; }
  bool receive(controller::PhoneMessage& message) override {
    if (messages.empty()) return false;
    message = messages.front(); messages.pop_front(); return true;
  }
  size_t send(const uint8_t* data, size_t length) override {
    if (!state.connected || !state.subscribed) return 0;
    output.append(reinterpret_cast<const char*>(data), length); return length;
  }
  void feed(const std::string& text, uint32_t session = 1) {
    controller::PhoneMessage message;
    assert(text.size() <= sizeof(message.data));
    message.length = text.size(); message.session = session;
    memcpy(message.data, text.data(), text.size()); messages.push_back(message);
  }
};
struct UtcClock : controller::hal::UtcClock {
  controller::hal::EpochSeconds epoch = 0;
  bool read_ok = true, set_ok = true;
  unsigned sets = 0;
  bool read(controller::hal::EpochSeconds& out) const override { out = epoch; return read_ok; }
  bool synchronize(controller::hal::EpochSeconds value) override { ++sets; if (!set_ok) return false; epoch = value; return true; }
};
struct System : controller::System {
  controller::ScannerUpdateStorage* update_storage = nullptr;
  controller::ScannerUpdateStorage* scannerUpdateStorage() override { return update_storage; }
  Clock time;
  UtcClock utc;
  controller::hal::UtcClock& utcClock() override { return utc; }
  Stream local, usb;
  Wireless phone;
  controller::Output* output = nullptr;
  controller::hal::Capabilities caps = {false, false, false, false, false};
  bool startup_ok = true, maintenance = false, launch_ok = true, locked = false, reset = false, verify_ok = true;
  unsigned begins = 0, maintenance_calls = 0, starts = 0, polls = 0, indications = 0, reboots = 0, verifications = 0;
  bool begin(controller::Output& sink) override { ++begins; output = &sink; return startup_ok; }
  controller::hal::Clock& clock() override { return time; }
  controller::hal::ByteStream& console() override { return local; }
  controller::Wireless& wireless() override { return phone; }
  controller::hal::Capabilities capabilities() const override { return caps; }
  bool bootMaintenance(bool) override { ++maintenance_calls; return maintenance; }
  void beginNavigation() override {}
  controller::hal::ByteStream* usbNmea() override { return caps.usb_nmea ? &usb : nullptr; }
  bool launchScanner(void (*)()) override { ++starts; return launch_ok; }
  void beginWatchdog(uint32_t) override {}
  void poll(uint32_t) override { ++polls; }
  bool tryStorageLock() override { return !locked; }
  void unlockStorage(bool) override {}
  bool resetRequested() override { return reset; }
  void reboot() override { ++reboots; }
  void indicate(bool, bool, bool) override { ++indications; }
  bool verifyStorage() override { assert(phone.polls > 0); ++verifications; return verify_ok; }
  void idle() override {}
};
struct Data : controller::Data {
  controller::DataStatus state;
  unsigned begins = 0, nav_begins = 0, captures = 0, polls = 0, receives = 0;
  unsigned source = 0, syncs = 0, recoveries = 0, resets = 0, appends = 0, flushes = 0, durable_flushes = 0;
  unsigned debug_starts = 0, debug_stops = 0, debug_polls = 0;
  bool nmea = true, fail_append = false, debug_watch = false;
  void begin() override { ++begins; }
  void beginNavigation() override { ++nav_begins; }
  void captureBootTime() override { ++captures; }
  void pollNavigation(const controller::LinkStatus&) override { ++polls; }
  bool receivePhone(const controller::PhoneMessage&, const controller::LinkStatus&) override { ++receives; return nmea; }
  void selectSource(unsigned selected) override { source = selected; }
  controller::DataStatus status() const override { return state; }
  void syncClock() override { ++syncs; }
  void recover() override { ++recoveries; }
  void appendReset() override { ++resets; }
  void append(const controller::ScanResult&) override { ++appends; if (fail_append) state.logging = false; }
  void flush() override { ++flushes; }
  void flushNow() override { ++durable_flushes; }
  void debugStart(bool watch) override { ++debug_starts; debug_watch = watch; }
  void debugStop() override { ++debug_stops; }
  void debugPoll() override { ++debug_polls; }
};
struct Scanners : controller::Scanners {
  std::deque<controller::ScanResult> results;
  std::deque<std::string> diagnostics;
  bool startup_ok = true;
  unsigned begins = 0, resets = 0, updates = 0;
  bool begin() override { ++begins; return startup_ok; }
  bool updateFirmware(controller::ScannerUpdateStorage&, controller::Output&, const controller::ScannerUpdateConfig&) override { assert(begins == 0); ++updates; return false; }
  bool result(controller::ScanResult& value) override {
    if (results.empty()) return false;
    value = results.front(); results.pop_front(); return true;
  }
  bool diagnostic(char* text, size_t capacity) override {
    if (diagnostics.empty()) return false;
    std::snprintf(text, capacity, "%s", diagnostics.front().c_str()); diagnostics.pop_front(); return true;
  }
  void resetDedupe() override { ++resets; }
  controller::ScannerStatus status() const override { return {1, 6, 63, unsigned(results.size()), 0, 0, 0, 1, 10}; }
};
}
