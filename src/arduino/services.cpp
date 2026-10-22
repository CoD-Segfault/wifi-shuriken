#include "hal/platform.h"
#include <Arduino.h>
#include "Configuration.h"
#if defined(SHURIKEN_P4_CONTROLLER)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#else
#include <pico/time.h>
#endif

namespace controller { namespace hal {
namespace {
#if defined(SHURIKEN_P4_CONTROLLER)
bool scanner_watched = false;
#endif
class ArduinoClock final : public Clock {
 public:
  uint32_t milliseconds() const override { return millis(); }
  uint32_t microseconds() const override { return micros(); }
  uint64_t microseconds64() const override {
#if defined(SHURIKEN_P4_CONTROLLER)
    return static_cast<uint64_t>(esp_timer_get_time());
#else
    return time_us_64();
#endif
  }
  void sleepMilliseconds(uint32_t duration) override { delay(duration); }
  void sleepMicroseconds(uint32_t duration) override {
    delayMicroseconds(duration);
  }
};

class GnssStream final : public ByteStream {
 public:
  int read(uint8_t* data, size_t capacity) override {
    size_t count = 0;
    while (count < capacity && GNSS_UART.available()) {
      const int value = GNSS_UART.read();
      if (value < 0) break;
      data[count++] = static_cast<uint8_t>(value);
    }
    return static_cast<int>(count);
  }
  int write(const uint8_t* data, size_t length) override {
    const size_t available = GNSS_UART.availableForWrite();
    return static_cast<int>(GNSS_UART.write(data, length < available ? length : available));
  }
};
}
Clock& clock() { static ArduinoClock instance; return instance; }
ByteStream& gnssStream() { static GnssStream instance; return instance; }
Capabilities capabilities() {
#if defined(SHURIKEN_P4_CONTROLLER)
  return {P4_USB_NMEA != 0, true, P4_USB_MTP != 0, false, SCANNER_SPI_UPDATE_ENABLED != 0};
#else
  return {true, false, true, CONTROLLER_SD_UPDATE_ENABLED != 0, SCANNER_SPI_UPDATE_ENABLED != 0};
#endif
}
bool scannerWorkerBegin() {
#if defined(SHURIKEN_P4_CONTROLLER)
  scanner_watched = esp_task_wdt_add(nullptr) == ESP_OK;
  return scanner_watched;
#else
  return true;  // RP2350 watchdog is fed by core0 only after a fresh heartbeat.
#endif
}
void scannerWorkerPoll() {
#if defined(SHURIKEN_P4_CONTROLLER)
  if (scanner_watched) esp_task_wdt_reset();
  vTaskDelay(1);
#endif
}
}}
