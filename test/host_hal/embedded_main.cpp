#include "Arduino.h"
#include "Configuration.h"
#include "hal/platform.h"
#include <cstdio>
#include <cstring>

int main() {
  auto& transport = controller::hal::scannerTransport();
  uint8_t tx[64], rx[64];
  for (size_t i = 0; i < sizeof(tx); ++i) tx[i] = uint8_t(i);
  assert(!transport.transfer(0, 10000000, tx, rx));
#if defined(SHURIKEN_P4_CONTROLLER)
  fake::begin_ok = false;
  assert(!transport.begin()); assert(!fake::enabled);
  fake::begin_ok = true; fake::pullup_ok = false;
  assert(!transport.begin()); assert(!fake::enabled);
  assert(std::find(fake::events.begin(), fake::events.end(), 2001) != fake::events.end());
  fake::pullup_ok = true;
#endif
  assert(transport.begin());
#if SCANNER_USE_SHIFTREG_CS
  assert(transport.slotCount() == 6);
  assert(fake::cs == 0xff && fake::enabled);
#else
  assert(transport.slotCount() == 1);
  assert(!fake::direct);
#endif
  // A pad pull-up must not use pinMode and detach P4 MISO from SPI.
  assert(std::find(fake::events.begin(), fake::events.end(), 3210) == fake::events.end());
  for (uint8_t slot = 0; slot < transport.slotCount(); ++slot) {
    assert(transport.transfer(slot, 10000000, tx, rx));
    const auto& frame = fake::frames.back();
    assert(frame.data.size() == 64 && frame.hz == 10000000);
    assert(std::equal(frame.data.begin(), frame.data.end(), tx));
#if SCANNER_USE_SHIFTREG_CS
    assert(frame.cs == uint8_t(0xffu & ~(1u << slot)));
    assert(fake::cs == 0xff && fake::enabled);
#else
    assert(!fake::direct);
#endif
    for (size_t i = 0; i < sizeof(rx); ++i) assert(rx[i] == uint8_t(tx[i] ^ 0xa5));
  }
  assert(!transport.transfer(transport.slotCount(), 10000000, tx, rx));
  assert(!transport.transfer(0, 0, tx, rx));
  assert(transport.transfer(0, 20000000, nullptr, nullptr)); // OTA frequency path
  for (auto byte : fake::frames.back().data) assert(byte == 0);
#if defined(SHURIKEN_P4_CONTROLLER)
  for (size_t i = 1; i < fake::frames.size(); ++i)
    assert(uint32_t(fake::frames[i].time - fake::frames[i - 1].time) >= 1010);
  fake::now = UINT32_MAX - 10;
  assert(transport.transfer(0, 10000000, tx, rx));
  fake::now += 20;
  assert(transport.transfer(0, 10000000, tx, rx));
  size_t n = fake::frames.size();
  assert(uint32_t(fake::frames[n - 1].time - fake::frames[n - 2].time) >= 1010);
#else
  // RP2350 does not inherit P4's minimum gap or CS delays.
  assert(fake::now == 0);
#endif
  auto& gnss = controller::hal::gnssStream();
  assert(gnss.read(rx, sizeof(rx)) == 0);
  fake_uart.rx = {1, 2, 3};
  assert(gnss.read(rx, 2) == 2 && rx[0] == 1 && rx[1] == 2);
  assert(gnss.read(rx, sizeof(rx)) == 1 && rx[0] == 3);
  assert(gnss.write(tx, sizeof(tx)) == 2);
  fake_uart.space = 0;
  assert(gnss.write(tx, sizeof(tx)) == 0);
  auto& clock = controller::hal::clock();
  fake::now = 1234000;
  assert(clock.microseconds() == 1234000 && clock.milliseconds() == 1234);
  clock.sleepMicroseconds(500); assert(fake::now == 1234500);
  const auto caps = controller::hal::capabilities();
#if defined(SHURIKEN_P4_CONTROLLER)
  assert(caps.usb_nmea == (P4_USB_NMEA != 0) && caps.ble_nmea && caps.mtp == (P4_USB_MTP != 0) && !caps.controller_update && caps.scanner_update);
#else
  assert(caps.usb_nmea && !caps.ble_nmea && caps.mtp && caps.controller_update && caps.scanner_update);
#endif
  assert(controller::hal::scannerWorkerBegin());
  controller::hal::scannerWorkerPoll();
#if defined(SHURIKEN_P4_CONTROLLER)
  assert(fake::watchdog_adds == 1 && fake::watchdog_feeds == 1 && fake::yields == 1);
  fake::watchdog_ok = false;
  assert(!controller::hal::scannerWorkerBegin());
  controller::hal::scannerWorkerPoll();
  assert(fake::watchdog_feeds == 1 && fake::yields == 2);
#else
  assert(fake::watchdog_adds == 0 && fake::watchdog_feeds == 0 && fake::yields == 0);
#endif
  std::puts("Embedded HAL: safe initialization/frame/CS/timing/UART/capability tests passed");
}
