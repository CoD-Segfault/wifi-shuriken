#pragma once

#include <stddef.h>
#include <stdint.h>

namespace controller { namespace hal {

// No Arduino, RTOS, filesystem or USB types cross these contracts. Each device
// has one owner; implementations need not provide internal locking.
class Clock {
 public:
  virtual ~Clock() = default;
  virtual uint32_t milliseconds() const = 0;
  virtual uint32_t microseconds() const = 0;
  // Native wide monotonic counter, independent of UTC adjustments and 32-bit rollover.
  virtual uint64_t microseconds64() const = 0;
  virtual void sleepMicroseconds(uint32_t duration) = 0;
  virtual void sleepMilliseconds(uint32_t duration) {
    // Chunk to preserve the entire uint32 millisecond range without overflow.
    while (duration >= 1000) { sleepMicroseconds(1000000); duration -= 1000; }
    if (duration) sleepMicroseconds(duration * 1000);
  }
};

class ByteStream {
 public:
  virtual ~ByteStream() = default;
  // Nonblocking reads: 0 means no bytes yet, -1 means transport failure.
  virtual int read(uint8_t* data, size_t capacity) = 0;
  // Bounded writes may be short; -1 means failure. Caller retains unsent bytes.
  virtual int write(const uint8_t* data, size_t length) = 0;
};

class ScannerTransport {
 public:
  virtual ~ScannerTransport() = default;
  // Initialize with every CS inactive; failure must leave outputs safe.
  virtual bool begin() = 0;
  // Stable while the initialized transport is in use.
  virtual uint8_t slotCount() const = 0;
  // Exactly one 64-byte, mode-0, MSB-first full-duplex frame. CS remains
  // asserted throughout. nullptr TX sends zeroes; nullptr RX discards input.
  // Backend owns CS/OE/latch sequencing and minimum frame/setup/hold delays.
  // A false return invalidates the entire reply. Frequency is explicit so OTA
  // and normal commands use the same transport without hidden mode changes.
  virtual bool transfer(uint8_t slot, uint32_t hz,
                        const uint8_t* tx, uint8_t* rx) = 0;
};

struct Capabilities {
  bool usb_nmea;
  bool ble_nmea;
  bool mtp;
  bool controller_update;
  bool scanner_update;
};

}}  // namespace controller::hal
