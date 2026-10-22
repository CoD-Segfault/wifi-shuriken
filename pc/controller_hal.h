#pragma once
#include "hal/controller.h"
#include <chrono>

namespace controller { namespace pc {
// Bind before starting the shared scanner runtime; referenced objects must
// outlive it. Call/poll only from their owning thread.
void bindPlatform(hal::ScannerTransport& scanner, hal::Clock& clock, hal::ByteStream& gnss);
class SteadyClock final : public hal::Clock {
 public:
  uint32_t milliseconds() const override;
  uint32_t microseconds() const override;
  uint64_t microseconds64() const override;
  void sleepMilliseconds(uint32_t duration) override;
  void sleepMicroseconds(uint32_t duration) override;
 private:
  std::chrono::steady_clock::time_point epoch_ = std::chrono::steady_clock::now();
};

// Linux/POSIX UART; Tigard interface A is exposed by the kernel serial driver.
class SerialStream final : public hal::ByteStream {
 public:
  ~SerialStream() override;
  bool open(const char* device, uint32_t baud = 115200);
  void close();
  int read(uint8_t* data, size_t capacity) override;
  int write(const uint8_t* data, size_t length) override;
  SerialStream() = default;
  SerialStream(const SerialStream&) = delete;
  SerialStream& operator=(const SerialStream&) = delete;
 private:
  int fd_ = -1;
};

// USB reads are bounded (not UART-style nonblocking). Injectable so command
// encoding, short USB transfers and failure cleanup can be tested without USB.
class MpsseLink {
 public:
  virtual ~MpsseLink() = default;
  virtual int write(const uint8_t* data, size_t length) = 0;
  virtual int read(uint8_t* data, size_t capacity) = 0;
};

struct TigardConfig {
  bool shift_register = true;
  uint8_t slots = 6;
  // BDBUS4/5 are exposed as auxiliary signals on the JTAG header. Use external
  // OE and scanner CS pull-ups so USB initialization/disconnect stays safe.
  uint8_t latch = 0x10;
  uint8_t output_enable = 0x20;
  uint32_t frame_gap_us = 1000;
  uint32_t cs_setup_us = 5;
  uint32_t cs_hold_us = 5;
  uint32_t io_timeout_us = 100000;
};

class TigardScannerTransport final : public hal::ScannerTransport {
 public:
  TigardScannerTransport(MpsseLink& link, hal::Clock& clock, TigardConfig config = {})
      : link_(link), clock_(clock), config_(config) {}
  ~TigardScannerTransport() override;
  TigardScannerTransport(const TigardScannerTransport&) = delete;
  TigardScannerTransport& operator=(const TigardScannerTransport&) = delete;
  bool begin() override;
  uint8_t slotCount() const override { return config_.shift_register ? config_.slots : 1; }
  bool transfer(uint8_t slot, uint32_t hz, const uint8_t* tx, uint8_t* rx) override;
  bool shutdown();
 private:
  bool send(const uint8_t* data, size_t length);
  bool receive(uint8_t* data, size_t length);
  bool pins(uint8_t value);
  bool frequency(uint32_t hz);
  bool select(uint8_t state);
  bool fail();
  MpsseLink& link_;
  hal::Clock& clock_;
  TigardConfig config_;
  bool ready_ = false;
  bool have_frame_ = false;
  uint32_t frame_end_ = 0;
};

constexpr hal::Capabilities capabilities() { return {false, false, false, false, true}; }
}}  // namespace controller::pc
