#include "arduino/system.h"
#include "Configuration.h"
#include "hal/posix_update_storage.h"
#include "arduino/update_config.h"
#include "scanner_update_transport.h"
#include "log_storage.h"
#include "usb_mtp.h"
#include "p4/ble_uart.h"
#include "p4/sdmmc_storage.h"
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#ifndef P4_NATIVE_USB_CONSOLE
#define P4_NATIVE_USB_CONSOLE 0
#endif
#if P4_NATIVE_USB_CONSOLE && (!ARDUINO_USB_CDC_ON_BOOT || ARDUINO_USB_MODE)
#error "P4 native USB console requires CDC on boot and TinyUSB OTG mode (ARDUINO_USB_MODE=0)"
#endif

namespace controller {
namespace {
static_assert(ARDUINO_RUNNING_CORE == 0 && CONFIG_BT_NIMBLE_PINNED_TO_CORE == 0,
              "Application and precompiled NimBLE host must remain on core 0");
OutputStream output;
class BleWireless final : public Wireless {
 public:
  void poll() override { p4::blePoll(); }
  LinkStatus status() const override {
    const auto value = p4::bleStatus();
    return {value.ready, value.connected, value.subscribed, value.session, value.rxDropped, value.txDroppedBytes};
  }
  bool receive(PhoneMessage& message) override {
    p4::BleMessage incoming;
    if (!p4::bleRead(incoming)) return false;
    message.length = incoming.length; message.session = incoming.session;
    if (message.length > sizeof(message.data)) return false;
    memcpy(message.data, incoming.data, message.length); return true;
  }
  size_t send(const uint8_t* data, size_t length) override { return p4::bleWrite(data, length); }
} wireless;
class P4System final : public EmbeddedSystem {
 public:
  bool begin(Output& sink) override {
    output.output = &sink;
    Serial0.begin(115200);
#if P4_NATIVE_USB_CONSOLE
    Serial.begin(115200); // Arduino starts the TinyUSB device before setup().
    Serial.setTxTimeoutMs(0); // Preserve nonblocking console writes if the host stalls.
#endif
    controllerGnssRuntimeInitUsbPassthrough();
    delay(1500);
    // Preserve the hardware-qualified SD-before-BLE startup order.
    if (!p4::storageBegin()) sink.text("SD mount unavailable; continuing without a card\r\n");
    if (!p4::bleBegin()) sink.text("BLE initialization failed\r\n");
    return true;
  }
  hal::ByteStream& console() override {
#if P4_NATIVE_USB_CONSOLE
    static SerialByteStream stream(Serial);
#else
    static SerialByteStream stream(Serial0);
#endif
    return stream;
  }
  Wireless& wireless() override { return controller::wireless; }
  bool bootMaintenance(bool) override { return false; }
  ScannerUpdateStorage* scannerUpdateStorage() override { static hal::PosixUpdateStorage updates("/sdcard"); return &updates; }
  hal::ScannerTransport* beginScannerUpdate() override {
    return updateTransport().open() ? &updateTransport() : nullptr;
  }
  void endScannerUpdate() override {
    if (!updateTransport().close()) output.println("Scanner update SPI task did not stop; normal scanning disabled");
  }
  bool launchScanner(void (*entry)()) override {
    if (!updateTransport().closed()) return false;
    entry_ = entry;
    return xTaskCreatePinnedToCore([](void* self) {
      static_cast<P4System*>(self)->entry_(); vTaskDelete(nullptr);
    }, "shuriken-scanner", 8192, this, 1, nullptr, 1) == pdPASS;
  }
  void beginWatchdog(uint32_t) override { watched_ = esp_task_wdt_add(nullptr) == ESP_OK; }
  void poll(uint32_t) override {
    if (watched_) esp_task_wdt_reset();
#if P4_USB_MTP
    p4::pollUsbMtp([] { controllerServices().data.flushNow(); });
#endif
  }
  bool tryStorageLock() override { return true; }
  void unlockStorage(bool) override {}
  bool resetRequested() override { return false; }
  void reboot() override { esp_restart(); }
  void indicate(bool, bool, bool) override {}
  bool verifyStorage() override { return p4::storageVerify(); }
 private:
  p4::ScannerUpdateTransport& updateTransport() {
    static p4::ScannerUpdateTransport transport(hal::scannerTransport()); return transport;
  }
  void (*entry_)() = nullptr;
  bool watched_ = false;
} system;
const char* resetName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON: return "PWRON";
    case ESP_RST_SW: return "SOFT";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: case ESP_RST_TASK_WDT: case ESP_RST_WDT: return "WATCHDOG";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    default: return "OTHER";
  }
}
}
Services& controllerServices() {
  static p4::LogStorage storage(output);
  static const controller::logging::Config config = {500, 30000,
    CONTROLLER_GNSS_BOOT_TIMESTAMP_WAIT_MS, CONTROLLER_GNSS_BOOT_TIMESTAMP_READS,
    CONTROLLER_GPS_FIELD_MAX_AGE_MS, 1000, 5000, CONTROLLER_GNSS_MIN_VALID_YEAR, P4_REQUIRE_GNSS_TIME_FOR_LOGGING != 0};
  const auto reason = esp_reset_reason();
  static RuntimeData data(system, storage, GNSS_UART, output, config,
    {1, static_cast<uint8_t>(reason), resetName(reason)});
  static RuntimeScanners scanners(system);
  static Services services = {system, data, scanners,
    {CONTROLLER_SCAN_RESULT_DRAIN_PER_LOOP, CONTROLLER_SERIAL_MSG_DRAIN_PER_LOOP,
     CONTROLLER_DEDUPE_RESET_COOLDOWN_MS, PERIODIC_STATUS_INTERVAL_MS,
     CONTROLLER_DEDUPE_RESET_ON_FIX_ACQUIRE != 0, false, configuredScannerUpdate()}};
  return services;
}
}
