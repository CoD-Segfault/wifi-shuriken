#include "fakes.h"
#include "controller/runtime.h"
#include "posix/log_storage.h"
#include "controller_hal.h"
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <thread>
namespace controller { Services& controllerServices() { throw std::logic_error("unused global factory"); } }
static std::string sentence(const std::string& body) {
  unsigned checksum = 0; for (auto byte : body) checksum ^= static_cast<unsigned char>(byte);
  char tail[8]; snprintf(tail, sizeof(tail), "*%02X\r\n", checksum);
  return "$" + body + tail;
}
struct NoTransport : controller::hal::ScannerTransport {
  bool begin() override { return true; }
  uint8_t slotCount() const override { return 6; }
  bool transfer(uint8_t, uint32_t, const uint8_t*, uint8_t*) override { return false; }
};
static void integration() {
  char path[] = "/tmp/shuriken-application-XXXXXX";
  assert(mkdtemp(path));
  fixture::System system; fixture::Stream uart; NoTransport transport;
  controller::pc::bindPlatform(transport, system.time, uart);
  system.caps = {true, true, false, false, false};
  system.phone.state = {true, true, true, 1, 0, 0};
  controller::storage::PosixLogStorage storage(path);
  HardwareSerial serial; serial.stream = &uart;
  controller::OutputStream output;
  const controller::logging::Config config = {0, 30000, 0, 1, 3000, 1000, 0, 2026,
    false};
  controller::RuntimeData data(system, storage, serial, output, config, {1, 0, "TEST"});
  fixture::Scanners scanners;
  controller::Application app({system, data, scanners, {}}); output.output = &app;
  assert(app.begin()); assert(data.status().logging && !data.status().clock_valid);
  const std::string log_path = data.status().path;
  assert(log_path.find("NO_GNSS_TIME") != std::string::npos);
  controller::ScanResult observation = {};
  std::strcpy(observation.ssid, "before-fix"); observation.bssid[0] = 2;
  observation.channel = 1; observation.band = 2; observation.rssi = -50;
  scanners.results.push_back(observation); app.poll();
  assert(data.status().rows == 1 && data.status().blank_rows == 1 && scanners.resets == 0);
  const auto phone_rmc = sentence("GPRMC,140123.00,A,4807.038,N,01131.000,E,0.0,0.0,211026,,,A");
  system.phone.feed(phone_rmc.substr(0, 20)); app.poll(); assert(!data.status().fix[2]);
  system.phone.feed(phone_rmc.substr(20)); app.poll();
  assert(data.status().fix[2] && data.status().clock_valid && scanners.resets == 1);
  assert(std::string(data.status().path) == log_path);
  observation.bssid[5] = 1; std::strcpy(observation.ssid, "with-phone-fix");
  scanners.results.push_back(observation); app.poll();
  assert(data.status().rows == 2 && data.status().blank_rows == 1);
  system.usb.feed(sentence("GPRMC,140123.00,A,4907.038,N,01131.000,E,0.0,0.0,211026,,,A"));
  app.poll(); assert(data.status().fix[1]);
  observation.bssid[5] = 2; std::strcpy(observation.ssid, "with-usb-fix");
  scanners.results.push_back(observation); app.poll();
  uart.feed(sentence("GPRMC,140123.00,A,5007.038,N,01131.000,E,0.0,0.0,211026,,,A"));
  uart.feed(sentence("GPGGA,140123.00,5007.038,N,01131.000,E,1,06,1.0,100.0,M,0.0,M,,"));
  app.poll(); assert(data.status().fix[0]);
  observation.bssid[5] = 3; std::strcpy(observation.ssid, "with-uart-fix");
  scanners.results.push_back(observation); app.poll();
  // Diagnostics must not change the selected parser's update flags or active log.
  system.local.feed(":gps-debug on\n"); app.poll();
  for (int i = 0; i < 10; ++i) { system.time.now += 100; app.poll(); }
  assert(std::string(data.status().path) == log_path);
  system.phone.state.connected = false; app.poll(); assert(!data.status().fix[2] && data.status().fix[0]);
  system.time.now += 4000; app.poll();
  assert(!data.status().fix[0] && !data.status().fix[1]);
  system.phone.state.connected = true; system.phone.state.session = 2;
  system.phone.feed(phone_rmc, 1); app.poll(); assert(!data.status().fix[2]);
  system.phone.feed(phone_rmc, 2); app.poll(); assert(data.status().fix[2] && scanners.resets == 2);
  data.flushNow();
  std::ifstream log(std::string(path) + log_path);
  const std::string csv((std::istreambuf_iterator<char>(log)), {});
  assert(csv.find("before-fix") != std::string::npos && csv.find("with-phone-fix") != std::string::npos);
  assert(csv.find("48.117300") != std::string::npos);
  assert(csv.find("49.117300") != std::string::npos && csv.find("50.117300") != std::string::npos);
  data.flushNow(); std::filesystem::remove_all(path);
}
static void clockSources() {
  char path[] = "/tmp/shuriken-clock-sources-XXXXXX"; assert(mkdtemp(path));
  fixture::System system; fixture::Stream uart; NoTransport transport;
  controller::pc::bindPlatform(transport, system.time, uart);
  system.caps = {true, true, false, false, false};
  system.phone.state = {true, true, true, 1, 0, 0};
  controller::storage::PosixLogStorage storage(path);
  HardwareSerial serial; serial.stream = &uart;
  controller::OutputStream output; fixture::Scanners scanners;
  const controller::logging::Config config = {0,30000,0,1,3000,1000,5000,2026,false};
  controller::RuntimeData data(system, storage, serial, output, config, {1,0,"TEST"});
  controller::Application app({system,data,scanners,{}}); output.output = &app;
  assert(app.begin() && !data.status().clock_valid);
  // Clock-bearing sentences are useful even when none of the sources has a fix.
  system.usb.feed(sentence("GPRMC,123456.00,V,,,,,,,211026,,,N"));
  app.poll(); assert(data.status().clock_valid && !data.status().fix[1] && system.utc.sets == 1);
  const auto usb_epoch = system.utc.epoch;
  uart.feed(sentence("GPRMC,123457.00,V,,,,,,,211026,,,N"));
  app.poll(); assert(!data.status().fix[0] && system.utc.epoch == usb_epoch + 1 && system.utc.sets == 2);
  system.time.now += 4000;
  system.phone.feed(sentence("GPRMC,123458.00,V,,,,,,,211026,,,N"));
  app.poll(); assert(!data.status().fix[2] && system.utc.epoch == usb_epoch + 2 && system.utc.sets == 3);
  system.phone.state.connected = false; system.utc.epoch = 0;
  app.poll(); assert(!data.status().clock_valid && system.utc.sets == 3);
  // A later system/RTC clock recovery is detected without a GNSS source.
  system.utc.epoch = usb_epoch + 100;
  app.poll(); assert(data.status().clock_valid && system.utc.sets == 3);
  data.flushNow(); std::filesystem::remove_all(path);
}
static void queues() {
  controller::BoundedQueue<unsigned, 8> small;
  unsigned value;
  assert(!small.pop(value));
  for (unsigned i = 0; i < 8; ++i) assert(small.push(i));
  assert(!small.push(8) && small.size() == 8);
  for (unsigned i = 0; i < 8; ++i) { assert(small.pop(value)); assert(value == i); }
  controller::BoundedQueue<unsigned, 1024> queue;
  constexpr unsigned count = 200000;
  std::thread producer([&] {
    for (unsigned i = 0; i < count; ++i) while (!queue.push(i)) std::this_thread::yield();
  });
  for (unsigned i = 0; i < count; ++i) {
    while (!queue.pop(value)) std::this_thread::yield();
    assert(value == i);
  }
  producer.join(); assert(queue.size() == 0);
}
int main() {
  integration(); clockSources(); queues();
  std::puts("Unified runtime: real NMEA/CSV/source lifecycle/debug/relog and concurrent SPSC queues passed");
}
