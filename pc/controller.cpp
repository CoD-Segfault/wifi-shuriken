#include "controller/runtime.h"
#include "posix/log_storage.h"
#include "posix/utc_clock.h"
#include "controller_hal.h"
#include "ftdi_link.h"
#include "hal/posix_update_storage.h"
#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <thread>

namespace controller {
namespace {
struct Options { const char* serial; const char* uart; const char* directory; bool direct; } options;
OutputStream output;
class Console final : public hal::ByteStream {
 public:
  int read(uint8_t* data, size_t length) override {
    const auto count = ::read(STDIN_FILENO, data, length);
    return count < 0 && (errno == EAGAIN || errno == EINTR) ? 0 : static_cast<int>(count);
  }
  int write(const uint8_t* data, size_t length) override {
    const auto count = ::write(STDOUT_FILENO, data, length);
    return count < 0 && (errno == EAGAIN || errno == EINTR) ? 0 : static_cast<int>(count);
  }
};
class NoWireless final : public Wireless {
 public:
  void poll() override {}
  LinkStatus status() const override { return {}; }
  bool receive(PhoneMessage&) override { return false; }
  size_t send(const uint8_t*, size_t) override { return 0; }
};
class PcSystem final : public System {
 public:
  pc::SteadyClock clock_;
  pc::SerialStream uart_;
  pc::FtdiLink usb_;
  pc::TigardScannerTransport scanner_;
  HardwareSerial serial_;
  PcSystem() : scanner_(usb_, clock_, configuration()) { serial_.stream = &uart_; }
  bool begin(Output& sink) override {
    output.output = &sink;
    if (!uart_.open(options.uart) || !usb_.open(options.serial)) {
      std::fprintf(stderr, "Tigard/UART initialization failed: %s\n", usb_.error()); return false;
    }
    pc::bindPlatform(scanner_, clock_, uart_);
    input_flags_ = fcntl(STDIN_FILENO, F_GETFL);
    output_flags_ = fcntl(STDOUT_FILENO, F_GETFL);
    if (input_flags_ >= 0) fcntl(STDIN_FILENO, F_SETFL, input_flags_ | O_NONBLOCK);
    if (output_flags_ >= 0) fcntl(STDOUT_FILENO, F_SETFL, output_flags_ | O_NONBLOCK);
    return true;
  }
  hal::Clock& clock() override { return clock_; }
  hal::UtcClock& utcClock() override { return utc_; }
  hal::ByteStream& console() override { return console_; }
  Wireless& wireless() override { return wireless_; }
  hal::Capabilities capabilities() const override { return pc::capabilities(); }
  bool bootMaintenance(bool) override { return false; }
  void beginNavigation() override {}  // UART is already open; no implicit device reconfiguration.
  ScannerUpdateStorage* scannerUpdateStorage() override { static hal::PosixUpdateStorage updates(options.directory); return &updates; }
  hal::ScannerTransport* beginScannerUpdate() override { return &scanner_; }
  bool launchScanner(void (*entry)()) override {
    running_ = true;
    try { worker_ = std::thread(entry); } catch (...) { running_ = false; return false; }
    return true;
  }
  bool scannerRunning() const override { return running_.load(); }
  void beginWatchdog(uint32_t) override {}
  void poll(uint32_t) override {}
  bool tryStorageLock() override { return true; }
  void unlockStorage(bool) override {}
  bool resetRequested() override { return false; }
  void reboot() override { stop(); }
  void indicate(bool, bool, bool) override {}
  bool verifyStorage() override {
    char path[1024];
    if (std::snprintf(path, sizeof(path), "%s/shuriken-test-XXXXXX", options.directory) >= int(sizeof(path))) return false;
    const int fd = mkstemp(path);
    if (fd < 0) return false;
    uint8_t bytes[4096], readback[4096];
    for (size_t i = 0; i < sizeof(bytes); ++i) bytes[i] = static_cast<uint8_t>(i ^ 0xa5);
    const bool ok = ::write(fd, bytes, sizeof(bytes)) == sizeof(bytes) && fsync(fd) == 0 &&
      lseek(fd, 0, SEEK_SET) == 0 && ::read(fd, readback, sizeof(readback)) == sizeof(readback) &&
      memcmp(bytes, readback, sizeof(bytes)) == 0;
    const bool closed = ::close(fd) == 0; unlink(path); return ok && closed;
  }
  void idle() override { clock_.sleepMicroseconds(5000); }
  void stop() {
    running_ = false;
    if (worker_.joinable()) worker_.join();
    scanner_.shutdown();
    if (input_flags_ >= 0) fcntl(STDIN_FILENO, F_SETFL, input_flags_);
    if (output_flags_ >= 0) fcntl(STDOUT_FILENO, F_SETFL, output_flags_);
    input_flags_ = output_flags_ = -1;
  }
 private:
  static pc::TigardConfig configuration() { pc::TigardConfig config; config.shift_register = !options.direct; return config; }
  Console console_;
  NoWireless wireless_;
  std::atomic<bool> running_{false};
  std::thread worker_;
  int input_flags_ = -1, output_flags_ = -1;
  hal::SystemUtcClock host_utc_;
  hal::LocalUtcClock utc_{host_utc_, clock_};
};
PcSystem& pcSystem() { static PcSystem system; return system; }
volatile std::sig_atomic_t stop_requested = 0;
void signalHandler(int) { stop_requested = 1; }
}
Services& controllerServices() {
  auto& system = pcSystem();
  static storage::PosixLogStorage storage(options.directory);
  static const controller::logging::Config config = {0, 30000, 0, 1, 3000, 1000, 5000, 2026,
    false};
  static RuntimeData data(system, storage, system.serial_, output, config, {1, 0, "PROCESS_START"});
  static RuntimeScanners scanners(system);
  static const ScannerUpdateConfig updates = [] { ScannerUpdateConfig config; config.normal_hz = config.update_hz = 10000000; return config; }();
  static Services services = {system, data, scanners, {64, 16, 5000, 10000, true, false, updates}};
  return services;
}
}
int main(int argc, char** argv) {
  if (argc < 4 || argc > 5 || (argc == 5 && std::strcmp(argv[4], "--direct") != 0)) {
    std::fprintf(stderr, "Usage: %s TIGARD_SERIAL UART_DEVICE EXISTING_LOG_DIRECTORY [--direct]\n", argv[0]); return 2;
  }
  struct stat info = {};
  if (stat(argv[3], &info) != 0 || !S_ISDIR(info.st_mode)) { std::fprintf(stderr, "Log directory must already exist\n"); return 2; }
  controller::options = {argv[1], argv[2], argv[3], argc == 5};
  std::signal(SIGINT, controller::signalHandler); std::signal(SIGTERM, controller::signalHandler);
  auto& app = controller::application();
  const bool started = app.begin();
  if (started) while (!controller::stop_requested) app.poll();
  // Stop the scanner before flushing; the application still owns all files.
  controller::pcSystem().stop();
  controller::controllerServices().data.flushNow();
  return started ? 0 : 1;
}
