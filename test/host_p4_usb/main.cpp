#include "../host_application/fakes.h"
#include "controller/runtime.h"
#include "controller_hal.h"
#include "posix/log_storage.h"
#include "p4/usb_nmea.h"
#include <USBCDC.h>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
namespace controller { Services& controllerServices() { throw std::logic_error("unused factory"); } }
using namespace controller;
struct NoScanner : hal::ScannerTransport {
  bool begin() override { return true; }
  uint8_t slotCount() const override { return 6; }
  bool transfer(uint8_t, uint32_t, const uint8_t*, uint8_t*) override { return false; }
};
static std::string sentence(const std::string& body) {
  unsigned checksum = 0; for (const auto byte : body) checksum ^= static_cast<uint8_t>(byte);
  char suffix[8]; std::snprintf(suffix, sizeof(suffix), "*%02X\r\n", checksum);
  return "$" + body + suffix;
}
int main() {
  assert(fake_usb::registered[1]); // Descriptor exists before application setup.
  auto& stream = p4::usbNmeaStream();
  uint8_t data[16] = {};
  fake_usb::feed(1, "abc");
  assert(stream.read(data, sizeof(data)) == 0); // Explicit initialization gate.
  p4::beginUsbNmea(); p4::beginUsbNmea();
  assert(!fake_usb::reboot[1] && fake_usb::sdk_begins[1] == 0);
  assert(stream.read(data, 2) == 2 && data[0] == 'a' && data[1] == 'b');
  assert(stream.read(data, sizeof(data)) == 1 && data[0] == 'c');
  assert(stream.read(data, sizeof(data)) == 0 && stream.read(nullptr, 0) == 0);
  assert(stream.read(nullptr, 1) == -1 && stream.write(data, 1) == -1);
  fake_usb::feed(0, "console command"); fake_usb::tx[0] = "console output";
  // Raw NMEA does not require DTR and never touches the console CDC.
  assert(!fake_usb::dtr[1]);
  for (auto byte : std::string("$NMEA\r\n")) p4::mirrorUsbNmea(byte);
  assert(fake_usb::tx[1] == "$NMEA\r\n" && fake_usb::flushes[1] == 1);
  const unsigned written = fake_usb::writes[1]; fake_usb::room[1] = 0;
  for (unsigned i = 0; i < 10000; ++i) p4::mirrorUsbNmea('x');
  p4::mirrorUsbNmea('\n');
  assert(fake_usb::writes[1] == written && fake_usb::flushes[1] == 2);
  fake_usb::mounted = false;
  assert(stream.read(data, sizeof(data)) == 0);
  p4::mirrorUsbNmea('x'); assert(fake_usb::writes[1] == written);
  fake_usb::mounted = true; fake_usb::room[1] = 512; fake_usb::tx[1].clear();
  // Exercise the real shared parser, source policy, UTC synchronization and CSV
  // logger with the actual P4 CDC1 adapter and fragmented host input.
  struct System : fixture::System {
    hal::ByteStream* usbNmea() override { return &p4::usbNmeaStream(); }
    void mirrorNmea(uint8_t byte) override { p4::mirrorUsbNmea(byte); }
  } system;
  fixture::Stream uart; NoScanner scanner;
  pc::bindPlatform(scanner, system.time, uart);
  system.caps.usb_nmea = true;
  char root[] = "/tmp/shuriken-p4-usb-XXXXXX"; assert(mkdtemp(root));
  storage::PosixLogStorage store(root); HardwareSerial serial; serial.stream = &uart;
  OutputStream output; fixture::Scanners scans;
  const logging::Config config = {0,30000,0,1,3000,1000,0,2026,false};
  RuntimeData runtime(system, store, serial, output, config, {1,0,"USB_TEST"});
  Application app({system,runtime,scans,{}}); output.output = &app;
  assert(app.begin() && !runtime.status().clock_valid && !runtime.status().fix[1]);
  const auto rmc = sentence("GPRMC,200959.00,A,4807.038,N,01131.000,E,0.0,0.0,211026,,,A");
  fake_usb::feed(1, rmc.substr(0,20)); app.poll(); assert(!runtime.status().fix[1]);
  fake_usb::feed(1, rmc.substr(20)); app.poll();
  assert(runtime.status().fix[1] && runtime.status().clock_valid && system.utc.sets == 1);
  ScanResult observation = {}; observation.bssid[0] = 2; observation.channel = 1; observation.rssi = -50;
  std::strcpy(observation.ssid, "from-usb-nmea"); scans.results.push_back(observation); app.poll();
  assert(runtime.status().rows == 1 && runtime.status().blank_rows == 0);
  runtime.flushNow();
  std::ifstream log(std::string(root) + runtime.status().path);
  const std::string contents((std::istreambuf_iterator<char>(log)), {});
  assert(contents.find("48.1173000,11.5166667") != std::string::npos);
  // UART output is mirrored solely to CDC1, and input consumption is bounded.
  const auto gga = sentence("GPGGA,200959.00,5007.038,N,01131.000,E,1,06,1.0,100.0,M,0.0,M,,");
  uart.feed(gga); app.poll(); assert(fake_usb::tx[1] == gga);
  fake_usb::feed(1, std::string(512, 'x')); app.poll(); assert(fake_usb::rx[1].size() == 256);
  app.poll(); assert(fake_usb::rx[1].empty());
  system.time.now += 3001; app.poll(); assert(!runtime.status().fix[1]);
  fake_usb::feed(1, rmc); app.poll(); assert(runtime.status().fix[1]);
  assert(fake_usb::rx[0].size() == 15 && fake_usb::tx[0] == "console output");
  assert(fake_usb::reads[0] == 0 && fake_usb::writes[0] == 0 && fake_usb::flushes[0] == 0);
  runtime.flushNow(); std::filesystem::remove_all(root);
  puts("PASS: P4 CDC1 descriptor/isolation/no-DTR/backpressure/fragmented NMEA/UTC/CSV/budgets/recovery");
}
