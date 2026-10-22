// The same application is tested across capabilities; there is no P4 loop copy.
#include "../host_application/fakes.h"
#include <cstdio>
#include <stdexcept>
namespace controller { Services& controllerServices() { throw std::logic_error("unused global factory"); } }
static void applicationPolicy() {
  for (unsigned target = 0; target < 3; ++target) {
    fixture::System system; fixture::Data data; fixture::Scanners scanner;
    system.caps.usb_nmea = target == 0; system.caps.ble_nmea = target == 1;
    controller::Policy policy; policy.capture_boot_time = target == 0;
    controller::Application app({system, data, scanner, policy});
    assert(app.begin() && app.begin());
    assert(system.begins == 1 && scanner.begins == 1 && data.begins == 1);
    assert(data.captures == unsigned(target == 0));
    for (unsigned i = 0; i < 70; ++i) scanner.results.push_back({});
    for (unsigned i = 0; i < 20; ++i) scanner.diagnostics.push_back("diagnostic\r\n");
    app.poll(); assert(scanner.results.size() == 70 && scanner.diagnostics.size() == 4);
    data.state.storage_ready = data.state.logging = true;
    system.locked = true; app.poll(); assert(data.appends == 0);
    system.locked = false; app.poll(); assert(data.appends == 64 && scanner.results.size() == 6);
    data.fail_append = true; app.poll(); assert(data.appends == 65 && scanner.results.size() == 5);
    data.fail_append = false; data.state.logging = true;
    data.state.fix[2] = true; app.poll(); assert(data.source == 2 && scanner.resets == 1);
    data.state.fix[1] = true; app.poll(); assert(data.source == 1 && scanner.resets == 1);
    data.state.fix[0] = true; app.poll(); assert(data.source == 0 && scanner.resets == 1);
    data.state.fix[0] = data.state.fix[1] = data.state.fix[2] = false;
    app.poll(); assert(data.source == 0);
    system.time.now += 1000; data.state.fix[0] = true;
    app.poll(); assert(scanner.resets == 1);
    data.state.fix[0] = false; app.poll(); system.time.now += 5000;
    data.state.fix[0] = true; app.poll(); assert(scanner.resets == 2);
    data.state.fix[0] = false; app.poll(); system.time.now = UINT32_MAX - 1000;
    data.state.fix[0] = true; app.poll(); assert(scanner.resets == 3);
    data.state.fix[0] = false; app.poll(); system.time.now += 6000;
    data.state.fix[0] = true; app.poll(); assert(scanner.resets == 4);
    system.reset = true; app.poll(); assert(data.durable_flushes == 1 && system.reboots == 1);
  }
}
static void consoleAndDiagnostics() {
  fixture::System system; fixture::Data data; fixture::Scanners scanners;
  data.state.storage_ready = data.state.logging = true;
  controller::Application app({system, data, scanners, {}}); assert(app.begin());
  // A blocked local console must not block phone output or replay its backlog.
  system.local.writable = 0;
  app.text("old backlog\r\n");
  system.phone.state = {true, true, true, 1, 0, 0};
  app.text("current\r\n"); assert(system.phone.output == "current\r\n");
  for (int i = 0; i < 4; ++i) system.phone.feed("$NMEA\r\n");
  app.poll(); assert(data.receives == 2 && system.phone.messages.size() == 2);
  app.poll(); assert(data.receives == 4);
  data.nmea = false; system.phone.feed("hello");
  const auto before = system.phone.output; app.poll(); assert(system.phone.output == before);
  system.phone.feed("stale", 0); app.poll(); assert(data.receives == 5);
  system.local.feed(":gps-debug on\n"); app.poll(); assert(data.debug_starts == 1 && data.debug_watch);
  system.local.feed(":gps-debug off\n"); app.poll(); assert(data.debug_stops == 1);
  system.local.feed(":" + std::string(200, 'x') + "\n:status\n");
  app.poll(); app.poll(); assert(data.debug_starts == 1);
  system.local.feed("ordinary bytes"); app.poll(); system.time.now += 51; app.poll();
  assert(system.phone.output.find("ordinary bytes") != std::string::npos);
  system.local.feed(":storage-test\n"); app.poll(); assert(system.verifications == 1);
  for (unsigned i = 1; i < 100; ++i) { system.time.now += 100; app.poll(); }
  assert(system.verifications == 100);
  system.time.now += 100; app.poll(); assert(system.verifications == 100);
  system.local.feed(":sd-test\n"); system.verify_ok = false; app.poll();
  assert(system.verifications == 101); app.poll(); assert(system.verifications == 101);
  system.local.writable = 1;
  const auto phone_before = system.phone.output;
  for (int i = 0; i < 100; ++i) app.poll();
  assert(system.phone.output == phone_before);
}
static void failedStartup() {
  for (unsigned failure = 0; failure < 3; ++failure) {
    fixture::System system; fixture::Data data; fixture::Scanners scanners;
    if (failure == 0) system.startup_ok = false;
    if (failure == 1) system.maintenance = true;
    if (failure == 2) scanners.startup_ok = false;
    controller::Application app({system, data, scanners, {}});
    assert(!app.begin() && !app.begin());
    assert(system.begins == 1);
    app.poll(); assert(data.polls == 0 && data.appends == 0);
  }
}
static void scannerUpdateStartup() {
  struct Storage : controller::ScannerUpdateStorage {
    bool exists(const char*) const override { return false; }
    std::unique_ptr<controller::ScannerUpdateFile> open(const char*) override { return {}; }
    bool rename(const char*, const char*) override { return false; }
  } storage;
  for (unsigned scenario = 0; scenario < 7; ++scenario) {
    fixture::System system; fixture::Data data;
    struct Scanners : fixture::Scanners {
      fixture::Data* data = nullptr;
      bool updateFirmware(controller::ScannerUpdateStorage& storage, controller::Output& output,
                          const controller::ScannerUpdateConfig& config) override {
        assert(data->begins == 1 && data->nav_begins == 0);
        return fixture::Scanners::updateFirmware(storage, output, config);
      }
    } scanners;
    scanners.data = &data;
    system.caps.scanner_update = scenario != 1;
    system.update_storage = scenario == 4 ? nullptr : &storage;
    data.state.storage_ready = scenario != 3;
    system.maintenance = scenario == 5;
    system.startup_ok = scenario != 6;
    controller::Policy policy; policy.scanner_update.enabled = scenario != 2;
    controller::Application app({system, data, scanners, policy});
    assert(app.begin() == (scenario < 5));
    app.begin(); app.poll();
    assert(scanners.updates == unsigned(scenario == 0));
  }
}
int main() {
  applicationPolicy(); consoleAndDiagnostics(); failedStartup(); scannerUpdateStartup();
  std::puts("Unified application: startup/source priority/relog/queues/locks/console/diagnostics/reset passed");
}
