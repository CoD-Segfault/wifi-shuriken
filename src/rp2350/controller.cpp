#include "arduino/system.h"
#include "Configuration.h"
#include "rp2350/mtp_storage.h"
#include "rp2350/controller_update.h"
#include "rp2350/update_storage.h"
#include "rp2350/log_storage.h"
#include "arduino/update_config.h"
#include "controller_status.h"
#include <Adafruit_NeoPixel.h>
#include <Adafruit_TinyUSB.h>
#include <SPI.h>
#include <pico/multicore.h>
#include <hardware/gpio.h>
#include <hardware/watchdog.h>

namespace controller {
namespace {
SdFat storage;
OutputStream output;
Adafruit_NeoPixel pixels(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);
class Rp2350System final : public EmbeddedSystem {
 public:
  bool begin(Output& sink) override {
    output.output = &sink;
#if RGB_POWER_ENABLED
    pinMode(RGB_POWER_PIN, OUTPUT); digitalWrite(RGB_POWER_PIN, HIGH); delay(10);
#endif
    pixels.begin(); pixels.setPixelColor(0, pixels.Color(128, 0, 0)); pixels.show();
    const bool registered = controllerMtpStorageBegin(storage, [] { controllerServices().data.flushNow(); });
    TinyUSBDevice.setManufacturerDescriptor(USB_DEVICE_MANUFACTURER_NAME);
    TinyUSBDevice.setProductDescriptor(USB_DEVICE_PRODUCT_NAME);
    if (USB_DEVICE_SERIAL_OVERRIDE[0]) TinyUSBDevice.setSerialDescriptor(USB_DEVICE_SERIAL_OVERRIDE);
    Serial.begin(115200); controllerGnssRuntimeInitUsbPassthrough(); delay(2000);
    if (!registered) sink.text("MTP interface registration failed\r\n");
    pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
    SPI.setSCK(SD_PIN_SCK); SPI.setTX(SD_PIN_MOSI); SPI.setRX(SD_PIN_MISO);
#if SD_MISO_PULLUP
    gpio_pull_up(SD_PIN_MISO);
#endif
    pinMode(SD_PIN_CS, OUTPUT); digitalWrite(SD_PIN_CS, HIGH); SPI.begin();
#if SD_MISO_PULLUP
    gpio_pull_up(SD_PIN_MISO);
#endif
    return true;
  }
  hal::ByteStream& console() override { static SerialByteStream stream(Serial); return stream; }
  Wireless& wireless() override { return wireless_; }
  bool bootMaintenance(bool ready) override {
    if (controllerUpdateRuntimeHandleBoot(storage, ready, output, pixels)) return true;
    return false;
  }
  ScannerUpdateStorage* scannerUpdateStorage() override { static rp2350::UpdateStorage updates(storage); return &updates; }
  hal::ScannerTransport* beginScannerUpdate() override { return &hal::scannerTransport(); }
  void scannerUpdateStarting() override { pixels.setPixelColor(0, pixels.Color(80, 0, 80)); pixels.show(); }
  bool launchScanner(void (*entry)()) override { multicore_launch_core1(entry); return true; }
  void beginWatchdog(uint32_t) override {
#if CONTROLLER_WATCHDOG_TIMEOUT_MS > 0
    watchdog_enable(CONTROLLER_WATCHDOG_TIMEOUT_MS, true);
#endif
  }
  void poll(uint32_t heartbeat) override {
    controllerMtpStorageTask();
#if CONTROLLER_WATCHDOG_TIMEOUT_MS > 0
    if (uint32_t(millis() - heartbeat) < CONTROLLER_WATCHDOG_TIMEOUT_MS / 2) watchdog_update();
#else
    (void)heartbeat;
#endif
  }
  bool tryStorageLock() override { return controllerMtpStorageTryLock(); }
  void unlockStorage(bool ready) override { controllerMtpStorageSetReady(ready); controllerMtpStorageUnlock(); }
  bool resetRequested() override {
    if (digitalRead(RESET_BUTTON_PIN) != LOW) { pressed_ = false; return false; }
    if (!pressed_) { pressed_ = true; pressed_at_ = millis(); return false; }
    return uint32_t(millis() - pressed_at_) >= 50;
  }
  void reboot() override { watchdog_reboot(0, 0, 0); }
  void indicate(bool storage_ready, bool fix, bool logging) override {
    controllerStatusUpdateLed(pixels, controllerStatusCompute(storage_ready, fix, logging));
  }
  bool verifyStorage() override {
    char path[48];
    for (unsigned suffix = 0; suffix < 1000; ++suffix) {
      snprintf(path, sizeof(path), "/shuriken-test-%lu-%u.bin", static_cast<unsigned long>(millis()), suffix);
      if (storage.exists(path)) continue;
      FsFile scratch = storage.open(path, O_RDWR | O_CREAT | O_EXCL);
      if (!scratch) return false;
      uint8_t bytes[512], readback[512];
      for (size_t i = 0; i < sizeof(bytes); ++i) bytes[i] = static_cast<uint8_t>(i ^ 0xa5);
      bool ok = true;
      for (unsigned i = 0; i < 8 && ok; ++i) ok = scratch.write(bytes, sizeof(bytes)) == sizeof(bytes);
      scratch.flush(); ok = ok && !scratch.getWriteError() && scratch.seekSet(0);
      for (unsigned i = 0; i < 8 && ok; ++i) ok = scratch.read(readback, sizeof(readback)) == sizeof(readback) && memcmp(bytes, readback, sizeof(bytes)) == 0;
      scratch.close(); storage.remove(path); return ok;
    }
    return false;
  }
 private:
  NoWireless wireless_;
  bool pressed_ = false;
  uint32_t pressed_at_ = 0;
} system;
const char* resetName(RP2040::resetReason_t reason) {
  switch (reason) {
    case RP2040::PWRON_RESET: return "PWRON";
    case RP2040::RUN_PIN_RESET: return "RUN_PIN";
    case RP2040::SOFT_RESET: return "SOFT";
    case RP2040::WDT_RESET: return "WATCHDOG";
    case RP2040::DEBUG_RESET: return "DEBUG";
    case RP2040::GLITCH_RESET: return "GLITCH";
    case RP2040::BROWNOUT_RESET: return "BROWNOUT";
    default: return "UNKNOWN";
  }
}
}
Services& controllerServices() {
  static constexpr uint32_t clocks[] = {SD_SPI_CLOCK, 4000000, 1000000, 400000};
  static rp2350::LogStorage logs(storage, SPI, SD_PIN_CS, clocks, 4, output);
  static const controller::logging::Config config = {500, 30000,
    CONTROLLER_GNSS_BOOT_TIMESTAMP_WAIT_MS, CONTROLLER_GNSS_BOOT_TIMESTAMP_READS,
    CONTROLLER_GPS_FIELD_MAX_AGE_MS, 1000, 5000, CONTROLLER_GNSS_MIN_VALID_YEAR};
  const auto reason = rp2040.getResetReason();
  static RuntimeData data(system, logs, GNSS_UART, output, config,
    {5, static_cast<uint8_t>(reason), resetName(reason)});
  static RuntimeScanners scanners(system);
  static Services services = {system, data, scanners,
    {CONTROLLER_SCAN_RESULT_DRAIN_PER_LOOP, CONTROLLER_SERIAL_MSG_DRAIN_PER_LOOP,
     CONTROLLER_DEDUPE_RESET_COOLDOWN_MS, PERIODIC_STATUS_INTERVAL_MS,
     CONTROLLER_DEDUPE_RESET_ON_FIX_ACQUIRE != 0, true, configuredScannerUpdate()}};
  return services;
}
}
