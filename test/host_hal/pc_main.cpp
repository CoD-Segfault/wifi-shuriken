#include "controller_hal.h"
#include "spi_protocol_shared.h"
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <unistd.h>
#include <vector>

using namespace controller;
struct FakeClock : hal::Clock {
  uint32_t now = 0;
  uint32_t milliseconds() const override { return now / 1000; }
  uint32_t microseconds() const override { return now; }
  uint64_t microseconds64() const override { return now; }
  void sleepMicroseconds(uint32_t us) override { now += us; }
};

// Parse the actual wire command stream, including writes split at arbitrary
// USB boundaries. Assert CS/OE state at the moment clocks would reach scanners.
struct FakeUsb : pc::MpsseLink {
  struct Frame { uint32_t time; uint8_t selected; uint32_t hz; std::vector<uint8_t> data; };
  FakeClock& clock;
  std::vector<uint8_t> pending;
  std::deque<uint8_t> incoming;
  std::vector<Frame> frames;
  std::vector<uint8_t> selections;
  size_t chunk = 1000;
  bool error = false, stall = false, bad_sync = false, short_reply = false;
  bool shift = true;
  uint8_t pins = 0x28, cs = 0xff, shifted = 0xff, direction = 0;
  uint32_t hz = 0;
  explicit FakeUsb(FakeClock& c) : clock(c) {}
  int write(const uint8_t* data, size_t length) override {
    if (error) return -1;
    if (stall) return 0;
    const size_t count = std::min(length, chunk);
    pending.insert(pending.end(), data, data + count);
    while (!pending.empty()) {
      const uint8_t cmd = pending[0];
      size_t size = 1;
      if (cmd == 0x80 || cmd == 0x86) size = 3;
      if (cmd == 0x11 || cmd == 0x31) {
        if (pending.size() < 3) break;
        size = 3 + 1 + pending[1] + (size_t(pending[2]) << 8);
      }
      if (pending.size() < size) break;
      if (cmd == 0xaa) { incoming.push_back(bad_sync ? 0 : 0xfa); incoming.push_back(0xaa); }
      else if (cmd == 0x80) {
        const uint8_t next = pending[1]; direction = pending[2];
        if (shift && !(pins & 0x10) && (next & 0x10)) {
          assert(next & 0x20); cs = shifted; selections.push_back(cs);
        }
        pins = next;
      } else if (cmd == 0x81) incoming.push_back(pins);
      else if (cmd == 0x86) hz = 30000000u / (1 + pending[1] + (uint32_t(pending[2]) << 8));
      else if (cmd == 0x11) {
        assert(shift && (pins & 0x20) && size == 4 && hz == 1000000);
        shifted = pending[3];
      } else if (cmd == 0x31) {
        assert(size == SPI_FRAME_SIZE + 3);
        assert(shift ? !(pins & 0x20) && cs != 0xff : !(pins & 0x08));
        frames.push_back({clock.now, cs, hz, {pending.begin() + 3, pending.begin() + size}});
        for (size_t i = 0; i < SPI_FRAME_SIZE - (short_reply ? 1 : 0); ++i)
          incoming.push_back(uint8_t(i ^ 0xa5));
      } else assert(cmd == 0x87 || cmd == 0x8a || cmd == 0x97 || cmd == 0x8d || cmd == 0x85);
      pending.erase(pending.begin(), pending.begin() + size);
    }
    return static_cast<int>(count);
  }
  int read(uint8_t* data, size_t capacity) override {
    if (error) return -1;
    if (stall) return 0;
    const size_t count = std::min({capacity, chunk, incoming.size()});
    for (size_t i = 0; i < count; ++i) { data[i] = incoming.front(); incoming.pop_front(); }
    return static_cast<int>(count);
  }
};

static void framesAndTiming() {
  for (size_t chunk : {size_t(1), size_t(7), size_t(1000)}) {
    FakeClock clock; FakeUsb usb(clock); usb.chunk = chunk;
    pc::TigardScannerTransport transport(usb, clock);
    uint8_t tx[SPI_FRAME_SIZE], rx[SPI_FRAME_SIZE];
    for (size_t i = 0; i < sizeof(tx); ++i) tx[i] = uint8_t(i);
    assert(!transport.transfer(0, 10000000, tx, rx));
    assert(transport.begin()); assert(transport.slotCount() == 6);
    assert(usb.cs == 0xff && usb.direction == 0x3b);
    for (uint8_t slot = 0; slot < 6; ++slot) {
      assert(transport.transfer(slot, 10000000, tx, rx));
      assert(usb.frames.back().selected == uint8_t(0xffu & ~(1u << slot)));
      assert(usb.frames.back().hz == 10000000);
      assert(std::equal(tx, tx + sizeof(tx), usb.frames.back().data.begin()));
      for (size_t i = 0; i < sizeof(rx); ++i) assert(rx[i] == uint8_t(i ^ 0xa5));
      assert(usb.cs == 0xff);
      if (slot) assert(usb.frames[slot].time - usb.frames[slot - 1].time >= 1010);
    }
    assert(!transport.transfer(6, 10000000, tx, rx));
    assert(!transport.transfer(0, 0, tx, rx));
    assert(!transport.transfer(0, 30000001, tx, rx));
    assert(!transport.transfer(0, 1, tx, rx));
    assert(transport.transfer(0, 12000000, nullptr, nullptr));
    assert(usb.frames.back().hz == 10000000); // divisor rounds down
    for (uint8_t byte : usb.frames.back().data) assert(byte == 0);
    clock.now = UINT32_MAX - 20;
    assert(transport.transfer(1, 10000000, tx, rx));
    clock.now += 30;
    assert(transport.transfer(2, 10000000, tx, rx));
    const size_t n = usb.frames.size();
    assert(uint32_t(usb.frames[n - 1].time - usb.frames[n - 2].time) >= 1010);
    assert(transport.shutdown()); assert(usb.pins & 0x20);
    assert(!transport.transfer(0, 10000000, tx, rx));
    assert(transport.begin());
  }
}
static void errorsAndConfiguration() {
  for (int failure = 0; failure < 4; ++failure) {
    FakeClock clock; FakeUsb usb(clock);
    pc::TigardConfig config; config.io_timeout_us = 1000;
    pc::TigardScannerTransport transport(usb, clock, config);
    if (failure == 0) { usb.bad_sync = true; assert(!transport.begin()); continue; }
    assert(transport.begin());
    if (failure == 1) usb.error = true;
    if (failure == 2) usb.stall = true;
    if (failure == 3) usb.short_reply = true;
    uint8_t reply[SPI_FRAME_SIZE]; std::memset(reply, 0x42, sizeof(reply));
    assert(!transport.transfer(0, 10000000, nullptr, reply));
    for (auto byte : reply) assert(byte == 0x42); // never publish partial replies
    assert(!transport.transfer(0, 10000000, nullptr, reply));
    if (failure == 3) assert(usb.pins & 0x20);
  }
  for (int invalid = 0; invalid < 6; ++invalid) {
    FakeClock clock; FakeUsb usb(clock); pc::TigardConfig config;
    if (invalid == 0) config.slots = 0;
    if (invalid == 1) config.slots = 9;
    if (invalid == 2) config.latch = 0x03;
    if (invalid == 3) config.output_enable = config.latch;
    if (invalid == 4) config.io_timeout_us = 0;
    if (invalid == 5) config.output_enable = 0x60;
    pc::TigardScannerTransport transport(usb, clock, config);
    assert(!transport.begin()); assert(usb.pending.empty());
  }
  FakeClock clock; FakeUsb usb(clock); usb.shift = false;
  pc::TigardConfig config; config.shift_register = false;
  pc::TigardScannerTransport transport(usb, clock, config);
  assert(transport.begin()); assert(transport.slotCount() == 1);
  assert(transport.transfer(0, 10000000, nullptr, nullptr));
  assert(usb.pins & 0x08); assert(usb.direction == 0x0b);
  assert(!transport.transfer(1, 10000000, nullptr, nullptr));
  const auto caps = pc::capabilities();
  assert(!caps.usb_nmea && !caps.ble_nmea && !caps.mtp && !caps.controller_update && caps.scanner_update);
}
static void serialPty() {
  const int master = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
  assert(master >= 0 && grantpt(master) == 0 && unlockpt(master) == 0);
  pc::SerialStream serial;
  uint8_t data[64];
  assert(serial.read(data, sizeof(data)) == -1);
  assert(!serial.open("/no/such/uart"));
  assert(!serial.open(ptsname(master), 12345));
  assert(serial.open(ptsname(master)));
  assert(serial.read(data, sizeof(data)) == 0);
  const char nmea[] = "$GPRMC,fragment\r\n";
  assert(::write(master, nmea, sizeof(nmea)) == sizeof(nmea));
  size_t got = 0;
  for (unsigned poll = 0; poll < 100 && got < sizeof(nmea); ++poll) {
    const int count = serial.read(data + got, 3);
    assert(count >= 0); got += count;
    if (!count) usleep(1000);
  }
  assert(got == sizeof(nmea) && std::memcmp(data, nmea, sizeof(nmea)) == 0);
  assert(serial.write(reinterpret_cast<const uint8_t*>(nmea), sizeof(nmea)) == sizeof(nmea));
  int count = 0;
  for (unsigned poll = 0; poll < 100; ++poll) {
    count = ::read(master, data, sizeof(data));
    if (count > 0) break;
    usleep(1000);
  }
  assert(count == sizeof(nmea) && std::memcmp(data, nmea, sizeof(nmea)) == 0);
  for (uint32_t baud : {9600u, 38400u, 115200u}) assert(serial.open(ptsname(master), baud));
  ::close(master);
  assert(serial.write(data, 1) == -1); // disconnected tty
  serial.close(); serial.close();
  assert(serial.read(data, sizeof(data)) == -1);
}
int main() {
  framesAndTiming(); errorsAndConfiguration(); serialPty();
  std::puts("PC HAL: frame/slot/timing/USB failure/UART/capability tests passed");
}
