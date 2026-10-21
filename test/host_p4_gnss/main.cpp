#include "ble_gnss.h"
#include "gps_debug.h"
#include <cassert>
#include <cmath>
#include <string>

static std::string sentence(const std::string& body) {
  uint8_t checksum = 0;
  for (char c : body) checksum ^= c;
  char tail[8];
  snprintf(tail, sizeof(tail), "*%02X\r\n", checksum);
  return "$" + body + tail;
}

int main() {
  p4::BleGnss input;
  const auto rmc = sentence("GPRMC,140123.00,A,4807.038,N,01131.000,E,0.0,0.0,211026,,,A");
  auto feed = [&](const std::string& text, uint32_t session = 1) {
    return input.feed(reinterpret_cast<const uint8_t*>(text.data()), text.size(), session);
  };
  input.updateSession(true, 1, 0);
  assert(!feed("ordinary text\n") && !input.usable(3000));
  // ATT writes may split at any byte, including the checksum and CRLF.
  for (char c : rmc) feed(std::string(1, c));
  assert(input.usable(3000));  // RMC-only phone input needs no satellite count.
  assert(std::abs(input.parser().location.lat() - 48.1173) < 0.000001);
  assert(input.parser().date.year() == 2026);
  fakeMillis += 3000;
  assert(!input.usable(3000));
  assert(feed(rmc + rmc) && input.usable(3000));

  input.updateSession(false, 2, 0);
  assert(!input.usable(3000) && !feed(rmc));
  input.updateSession(true, 3, 0);
  assert(!feed(rmc, 1) && !input.usable(3000));  // Old queued session.
  assert(feed(rmc, 3) && input.usable(3000));
  input.updateSession(true, 4, 0);  // Reconnect between loop iterations.
  assert(!input.usable(3000));
  feed(rmc.substr(0, 30), 4);
  input.updateSession(true, 4, 1);  // RX overflow invalidates partial input/fix.
  feed(rmc.substr(30), 4);
  assert(!input.usable(3000));
  auto corrupt = rmc;
  corrupt[corrupt.find('*') + 1] = 'Z';
  feed(corrupt, 4);
  assert(!input.usable(3000) && input.parser().failedChecksum() == 1);
  feed("$" + std::string(512, 'x') + "\n", 4);
  assert(!input.usable(3000));
  feed(rmc, 4);
  assert(input.usable(3000));  // Resynchronize after malformed/oversized input.

  TinyGPSPlus hw, usb;
  auto& ble = input.parser();
  assert(p4::selectGpsSource(hw, true, usb, true, ble, true) == &hw);
  assert(p4::selectGpsSource(hw, false, usb, true, ble, true) == &usb);
  assert(p4::selectGpsSource(hw, false, usb, false, ble, true) == &ble);
  assert(p4::selectGpsSource(hw, false, usb, false, ble, false) == &hw);
  for (char c : rmc) hw.encode(c);
  const auto nearby = sentence("GPRMC,140124.00,A,4807.098,N,01131.000,E,0.0,0.0,211026,,,A");
  feed(nearby, 4);
  const bool locationUpdated = hw.location.isUpdated(), timeUpdated = hw.time.isUpdated();
  const p4::GpsDebugSource sources[3] = {{"UART", &hw, true, true}, {"USB", &usb, false, false}, {"BLE", &ble, true, true}};
  p4::GpsDebugReport report;
  report.capture(sources, "UART", 3000);
  assert(report.size() == 7);
  std::string diagnostic;
  for (size_t i = 0; i < report.size(); ++i) {
    assert(strlen(report.line(i)) < 256 && std::string(report.line(i)).find("\r\n") != std::string::npos);
    diagnostic += report.line(i);
  }
  assert(diagnostic.find("USB unavailable") != std::string::npos);
  assert(diagnostic.find("UART UTC=2026-10-21T14:01:23.00") != std::string::npos);
  assert(diagnostic.find("BLE UTC=2026-10-21T14:01:24.00") != std::string::npos);
  double separation = 0;
  assert(sscanf(diagnostic.c_str() + diagnostic.find("UART/BLE separation="), "UART/BLE separation=%lf", &separation) == 1);
  assert(separation > 110 && separation < 112);
  assert(hw.location.isUpdated() == locationUpdated && hw.time.isUpdated() == timeUpdated);
  fakeMillis += 3000;
  report.capture(sources, "none", 3000);
  assert(std::string(report.line(report.size() - 1)).find("separation=unavailable") != std::string::npos);
  const p4::GpsDebugSource allSources[3] = {{"UART", &hw, true, false}, {"USB", &usb, true, false}, {"BLE", &ble, true, false}};
  report.capture(allSources, "none", 3000);
  assert(report.size() == 10);  // All three source pairs fit in fixed memory.
  puts("PASS: GPS debug coordinates/time/separation, unavailable/stale sources, bounded reports, live parser flags preserved");
  puts("PASS: fragmented/multiple NMEA, RMC-only fixes, freshness, sessions/disconnect/drop isolation, checksum/size rejection, source priority");
}
