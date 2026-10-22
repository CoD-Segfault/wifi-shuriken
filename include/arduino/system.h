#pragma once
#include "controller/runtime.h"
#include "hal/platform.h"
#include "arduino/gnss.h"

namespace controller {
class SerialByteStream final : public hal::ByteStream {
 public:
  explicit SerialByteStream(Stream& serial) : serial_(serial) {}
  int read(uint8_t* data, size_t capacity) override {
    size_t count = 0;
    while (count < capacity && serial_.available()) {
      const int value = serial_.read(); if (value < 0) break;
      data[count++] = static_cast<uint8_t>(value);
    }
    return static_cast<int>(count);
  }
  int write(const uint8_t* data, size_t length) override {
    const int available = serial_.availableForWrite();
    if (available <= 0) return 0;
    return serial_.write(data, length < static_cast<size_t>(available) ? length : static_cast<size_t>(available));
  }
 private:
  Stream& serial_;
};
class NoWireless final : public Wireless {
 public:
  void poll() override {}
  LinkStatus status() const override { return {}; }
  bool receive(PhoneMessage&) override { return false; }
  size_t send(const uint8_t*, size_t) override { return 0; }
};
class EmbeddedSystem : public System {
 public:
  hal::Clock& clock() override { return hal::clock(); }
  hal::UtcClock& utcClock() override { return hal::utcClock(); }
  hal::Capabilities capabilities() const override { return hal::capabilities(); }
  void beginNavigation() override { controllerGnssRuntimeBegin(); }
  hal::ByteStream* usbNmea() override { return controllerUsbNmeaStream(); }
  void mirrorNmea(uint8_t byte) override { controllerMirrorNmea(byte); }
  void idle() override { delay(5); }
};
}
