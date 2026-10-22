#include "controller_hal.h"
#include "spi_protocol_shared.h"
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <limits.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

namespace controller { namespace pc {
uint32_t SteadyClock::milliseconds() const {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - epoch_).count();
}
uint32_t SteadyClock::microseconds() const {
  return std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - epoch_).count();
}
uint64_t SteadyClock::microseconds64() const {
  return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - epoch_).count();
}
void SteadyClock::sleepMilliseconds(uint32_t duration) {
  std::this_thread::sleep_for(std::chrono::milliseconds(duration));
}
void SteadyClock::sleepMicroseconds(uint32_t duration) {
  std::this_thread::sleep_for(std::chrono::microseconds(duration));
}
SerialStream::~SerialStream() { close(); }
void SerialStream::close() { if (fd_ >= 0) ::close(fd_); fd_ = -1; }
bool SerialStream::open(const char* device, uint32_t baud) {
  close();
  speed_t speed;
  switch (baud) {
    case 9600: speed = B9600; break;
    case 38400: speed = B38400; break;
    case 115200: speed = B115200; break;
    default: return false;
  }
  fd_ = ::open(device, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) return false;
  termios settings = {};
  if (tcgetattr(fd_, &settings) != 0) { close(); return false; }
  cfmakeraw(&settings);
  settings.c_cflag &= ~(CSTOPB | CRTSCTS | PARENB | CSIZE);
  settings.c_cflag |= CS8 | CLOCAL | CREAD;
  settings.c_cc[VMIN] = 0;
  settings.c_cc[VTIME] = 0;
  if (cfsetispeed(&settings, speed) != 0 || cfsetospeed(&settings, speed) != 0 ||
      tcsetattr(fd_, TCSANOW, &settings) != 0) { close(); return false; }
  return true;
}
int SerialStream::read(uint8_t* data, size_t capacity) {
  const ssize_t result = ::read(fd_, data, std::min(capacity, size_t(INT_MAX)));
  if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) return 0;
  return static_cast<int>(result);
}
int SerialStream::write(const uint8_t* data, size_t length) {
  const ssize_t result = ::write(fd_, data, std::min(length, size_t(INT_MAX)));
  if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) return 0;
  return static_cast<int>(result);
}

// FTDI AN_108: 0x80 GPIO, 0x86 clock divisor, 0x11 write bytes on falling
// edges, 0x31 full-duplex write falling/read rising, 0x87 send immediate.
bool TigardScannerTransport::send(const uint8_t* data, size_t length) {
  const uint32_t start = clock_.microseconds();
  while (length) {
    const int count = link_.write(data, length);
    if (count < 0 || static_cast<size_t>(count) > length) return false;
    data += count; length -= count;
    if (length && uint32_t(clock_.microseconds() - start) >= config_.io_timeout_us) return false;
    if (count == 0) clock_.sleepMicroseconds(100);
  }
  return true;
}
bool TigardScannerTransport::receive(uint8_t* data, size_t length) {
  const uint32_t start = clock_.microseconds();
  while (length) {
    const int count = link_.read(data, length);
    if (count < 0 || static_cast<size_t>(count) > length) return false;
    data += count; length -= count;
    if (length && uint32_t(clock_.microseconds() - start) >= config_.io_timeout_us) return false;
    if (count == 0) clock_.sleepMicroseconds(100);
  }
  return true;
}
bool TigardScannerTransport::pins(uint8_t value) {
  const uint8_t direction = 0x0b | (config_.shift_register ? config_.latch | config_.output_enable : 0);
  // Read-back is a completion barrier: host sleeps must start after the FTDI
  // has changed the pins, not merely after USB accepted a buffered command.
  const uint8_t command[] = {0x80, value, direction, 0x81, 0x87};
  if (!send(command, sizeof(command))) return false;
  uint8_t actual;
  if (!receive(&actual, 1)) return false;
  return true;
}
bool TigardScannerTransport::frequency(uint32_t hz) {
  if (!hz || hz > 30000000) return false;
  // Round down the achievable frequency, never exceed the requested maximum.
  const uint32_t divisor = (30000000u + hz - 1) / hz - 1;
  if (divisor > 65535) return false;
  const uint8_t command[] = {0x86, uint8_t(divisor), uint8_t(divisor >> 8)};
  return send(command, sizeof(command));
}
bool TigardScannerTransport::select(uint8_t state) {
  if (!config_.shift_register) return pins(state == 0xff ? 0x08 : 0x00);
  // Disable outputs before clocking shared SCK/MOSI, then latch with clock low.
  if (!pins(config_.output_enable | 0x08) || !frequency(1000000)) return false;
  const uint8_t command[] = {0x11, 0, 0, state};
  return send(command, sizeof(command)) &&
         pins(config_.output_enable | config_.latch | 0x08) &&
         pins(config_.output_enable | 0x08) && pins(0x08);
}
bool TigardScannerTransport::begin() {
  ready_ = false; have_frame_ = false;
  const auto single_bit = [](uint8_t pin) { return pin && !(pin & (pin - 1)) && !(pin & 0x0f); };
  if (config_.io_timeout_us == 0 || config_.io_timeout_us > INT32_MAX ||
      (config_.shift_register && (!config_.slots || config_.slots > 8 ||
       !single_bit(config_.latch) || !single_bit(config_.output_enable) ||
       config_.latch == config_.output_enable))) return false;
  // Synchronize the command parser, rejecting a different interface/mode.
  const uint8_t sync[] = {0xaa, 0x87};
  uint8_t reply[2];
  if (!send(sync, sizeof(sync)) || !receive(reply, sizeof(reply)) ||
      reply[0] != 0xfa || reply[1] != 0xaa) return fail();
  const uint8_t setup[] = {0x8a, 0x97, 0x8d, 0x85};
  if (!send(setup, sizeof(setup)) || !select(0xff)) return fail();
  ready_ = true;
  return true;
}
bool TigardScannerTransport::fail() {
  // Best effort only after USB loss. Physical pull-ups are required.
  pins(0x08 | (config_.shift_register ? config_.output_enable : 0));
  ready_ = false;
  return false;
}
bool TigardScannerTransport::shutdown() {
  const bool was_ready = ready_;
  ready_ = false;
  return !was_ready || pins(0x08 | (config_.shift_register ? config_.output_enable : 0));
}
TigardScannerTransport::~TigardScannerTransport() { shutdown(); }
bool TigardScannerTransport::transfer(uint8_t slot, uint32_t hz,
                                      const uint8_t* tx, uint8_t* rx) {
  if (!ready_ || slot >= slotCount() || !hz || hz > 30000000 ||
      (30000000u + hz - 1) / hz - 1 > 65535) return false;
  if (have_frame_) {
    const uint32_t elapsed = clock_.microseconds() - frame_end_;
    if (elapsed < config_.frame_gap_us) clock_.sleepMicroseconds(config_.frame_gap_us - elapsed);
  }
  if (!select(uint8_t(0xffu & ~(1u << slot))) || !frequency(hz)) return fail();
  clock_.sleepMicroseconds(config_.cs_setup_us);
  uint8_t command[3 + SPI_FRAME_SIZE + 1] = {0x31, SPI_FRAME_SIZE - 1, 0};
  if (tx) std::memcpy(command + 3, tx, SPI_FRAME_SIZE);
  command[sizeof(command) - 1] = 0x87;
  uint8_t reply[SPI_FRAME_SIZE];
  if (!send(command, sizeof(command)) || !receive(reply, sizeof(reply))) return fail();
  clock_.sleepMicroseconds(config_.cs_hold_us);
  if (!select(0xff)) return fail();
  frame_end_ = clock_.microseconds(); have_frame_ = true;
  if (rx) std::memcpy(rx, reply, sizeof(reply));
  return true;
}
}}
