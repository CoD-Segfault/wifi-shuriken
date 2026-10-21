#pragma once

// Waveshare ESP32-P4-WIFI6 external wiring. Override with -D build flags.
// SD GPIO39-45 and C6 GPIO14-19/54 are reserved by the board.
#ifndef ESP_PIN_SCK
#define ESP_PIN_SCK 20
#endif
#ifndef ESP_PIN_MOSI
#define ESP_PIN_MOSI 22
#endif
#ifndef ESP_PIN_MISO
#define ESP_PIN_MISO 21
#endif
#ifndef SCANNER_SPI_CLOCK
#define SCANNER_SPI_CLOCK 10000000  // Conservative clock for wired P4 hardware validation.
#endif
#ifndef P4_SCANNER_FRAME_GAP_US
#define P4_SCANNER_FRAME_GAP_US 1000
#endif
#ifndef P4_SCANNER_CS_SETUP_US
#define P4_SCANNER_CS_SETUP_US 5
#endif
#ifndef P4_SCANNER_CS_HOLD_US
#define P4_SCANNER_CS_HOLD_US 5
#endif
#ifndef SCANNER_SHIFTREG_LATCH_PIN
#define SCANNER_SHIFTREG_LATCH_PIN 26
#endif
#ifndef SCANNER_SHIFTREG_OE_PIN
#define SCANNER_SHIFTREG_OE_PIN 27
#endif
#ifndef SCANNER_USE_SHIFTREG_CS
#define SCANNER_USE_SHIFTREG_CS 1
#endif
#ifndef SCANNER_SHIFTREG_OUTPUTS
#define SCANNER_SHIFTREG_OUTPUTS 6
#endif
#ifndef GNSS_TX
#define GNSS_TX 32
#endif
#ifndef GNSS_RX
#define GNSS_RX 33
#endif
#ifndef NMEA_PASSTHROUGH
#define NMEA_PASSTHROUGH 0
#endif
#ifndef SCANNER_SPI_UPDATE_ENABLED
#define SCANNER_SPI_UPDATE_ENABLED 0
#endif
#ifndef P4_REQUIRE_GNSS_TIME_FOR_LOGGING
#define P4_REQUIRE_GNSS_TIME_FOR_LOGGING 0
#endif

namespace p4_pins {
constexpr int used[] = {ESP_PIN_SCK, ESP_PIN_MOSI, ESP_PIN_MISO,
                       SCANNER_SHIFTREG_LATCH_PIN, SCANNER_SHIFTREG_OE_PIN, GNSS_TX, GNSS_RX};
constexpr bool valid() {
  for (unsigned i = 0; i < sizeof(used) / sizeof(used[0]); ++i) {
    const int pin = used[i];
    if (!((pin >= 20 && pin <= 23) || pin == 26 || pin == 27 || pin == 32 || pin == 33 ||
          (pin >= 46 && pin <= 48))) return false;
    for (unsigned j = 0; j < i; ++j) if (used[j] == pin) return false;
  }
  return true;
}
static_assert(valid(), "P4 external pins must be unique and in the available GPIO set");
static_assert(SCANNER_USE_SHIFTREG_CS == 1 && SCANNER_SHIFTREG_OUTPUTS == 6,
              "P4 controller currently supports six shift-register scanner slots");
}  // namespace p4_pins
