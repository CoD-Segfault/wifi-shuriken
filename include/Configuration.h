#pragma once

// Firmware version used in CSV metadata fields.
#ifndef APP_VERSION
#define APP_VERSION "0.2"
#endif
#ifndef APP_RELEASE_VERSION
#define APP_RELEASE_VERSION APP_VERSION
#endif
#ifndef RELEASE_VERSION
#define RELEASE_VERSION APP_VERSION
#endif

// SD card SPI0 pin mapping on the Pico master.
#ifndef SD_PIN_SCK
#define SD_PIN_SCK 2  // XIAO RP2350 D8: SPI0 clock line to SD card.
#endif
#ifndef SD_PIN_MOSI
#define SD_PIN_MOSI 3  // XIAO RP2350 D10: SPI0 MOSI line to SD card.
#endif
#ifndef SD_PIN_MISO
#define SD_PIN_MISO 4  // XIAO RP2350 D9: SPI0 MISO line from SD card.
#endif
#ifndef SD_PIN_CS
#define SD_PIN_CS 21  // XIAO RP2350 D11: SD card chip-select pin.
#endif

// Preferred SD card SPI bus speed in Hz for normal operation.
#ifndef SD_SPI_CLOCK
#define SD_SPI_CLOCK 10000000  // Preferred SD clock in Hz (10 MHz).
#endif

// Enable RP2040 internal pull-up on SD MISO to reduce startup bus noise.
#ifndef SD_MISO_PULLUP
#define SD_MISO_PULLUP 1  // 1 enables internal pull-up on SD MISO.
#endif

// Scanner SPI1 pin mapping on the Pico master (legacy ESP_PIN_* macro names).
#ifndef ESP_PIN_SCK
#define ESP_PIN_SCK 10  // XIAO RP2350 D17: SPI1 clock line to scanner module.
#endif
#ifndef ESP_PIN_MOSI
#define ESP_PIN_MOSI 11  // XIAO RP2350 D15: SPI1 MOSI line to scanner module.
#endif
#ifndef ESP_PIN_MISO
#define ESP_PIN_MISO 12  // XIAO RP2350 D16: SPI1 MISO line from scanner module.
#endif

// Scanner SPI bus clock in Hz (Pico master <-> scanner slaves).
#ifndef SCANNER_SPI_CLOCK
#define SCANNER_SPI_CLOCK 20000000  
#endif

// Enable RP2040 internal pull-up on scanner MISO to keep idle bus high
// when all scanner slaves are deasserted/tri-stated.
#ifndef SCANNER_MISO_PULLUP
#define SCANNER_MISO_PULLUP 1  // 1 enables internal pull-up on scanner MISO.
#endif

// Scanner CS wiring mode: 0 = direct GPIO CS, 1 = 74HC595 shift-register CS.
#ifndef SCANNER_USE_SHIFTREG_CS
#define SCANNER_USE_SHIFTREG_CS 1  // XIAO RP2350 default: shift-register CS.
#endif

// Shift-register control pins and SPI speed when SCANNER_USE_SHIFTREG_CS is enabled.
#ifndef SCANNER_SHIFTREG_LATCH_PIN
#define SCANNER_SHIFTREG_LATCH_PIN 26  // XIAO RP2350 D0: 74HC595 latch (RCLK).
#endif
#ifndef SCANNER_SHIFTREG_OE_PIN
#define SCANNER_SHIFTREG_OE_PIN 9  // XIAO RP2350 D18: 74HC595 OE (active low).
#endif
#ifndef SCANNER_SHIFTREG_SPI_HZ
#define SCANNER_SHIFTREG_SPI_HZ 1000000  // Shift-register SPI clock in Hz.
#endif

// Reuse shift-register OE for CS if only one scanner connected.
#if !SCANNER_USE_SHIFTREG_CS
#ifndef ESP_PIN_CS
#define ESP_PIN_CS SCANNER_SHIFTREG_OE_PIN
#endif
#endif

// Number of active-low CS outputs driven by the shift register.
#ifndef SCANNER_SHIFTREG_OUTPUTS
#define SCANNER_SHIFTREG_OUTPUTS 6  // Number of usable CS outputs (1..8).
#endif
#if SCANNER_SHIFTREG_OUTPUTS < 1 || SCANNER_SHIFTREG_OUTPUTS > 8
#error "SCANNER_SHIFTREG_OUTPUTS must be in [1,8]"
#endif

// Startup scanner slot index for round-robin polling.
#ifndef SCANNER_INITIAL_SLOT
#define SCANNER_INITIAL_SLOT 0  // First slot polled at boot.
#endif
#if SCANNER_INITIAL_SLOT < 0 || SCANNER_INITIAL_SLOT >= SCANNER_SHIFTREG_OUTPUTS
#error "SCANNER_INITIAL_SLOT out of range for SCANNER_SHIFTREG_OUTPUTS"
#endif

// WS2812 status LED configuration.
#ifndef RGB_PIN
#define RGB_PIN 22  // XIAO RP2350 onboard WS2812 data pin.
#endif
#ifndef NUMPIXELS
#define NUMPIXELS 1  // Number of chained WS2812 pixels.
#endif
// On some boards (for example XIAO RP2350), WS2812 power is gate-controlled.
// 1 enables driving RGB_POWER_PIN high during setup, 0 leaves it untouched.
#ifndef RGB_POWER_ENABLED
#define RGB_POWER_ENABLED 1
#endif
// GPIO used to enable onboard WS2812 power when RGB_POWER_ENABLED is 1.
#ifndef RGB_POWER_PIN
#define RGB_POWER_PIN 23
#endif

// Active-low reset button input pin.
#ifndef RESET_BUTTON_PIN
#define RESET_BUTTON_PIN 17  // Active-low reset button GPIO.
#endif

// GNSS UART pin mapping and parser/transport toggles.
#ifndef GNSS_TX
#define GNSS_TX 0  // XIAO RP2350 D6: GNSS UART TX.
#endif
#ifndef GNSS_RX
#define GNSS_RX 1  // XIAO RP2350 D7: GNSS UART RX.
#endif
#ifndef GNSS_UART
#define GNSS_UART Serial1  // XIAO RP2350 default GNSS UART.
#endif

// Requested GNSS UART FIFO size in bytes.
#ifndef SERIAL_BUFFER_SIZE
#define SERIAL_BUFFER_SIZE 2048  // GNSS UART FIFO target size in bytes.
#endif

// Mirror raw NMEA bytes to USB/serial output for external tools.
#ifndef NMEA_PASSTHROUGH
#define NMEA_PASSTHROUGH 1  // 1 mirrors GNSS NMEA bytes to host output.
#endif

// Allow core1 worker diagnostics to be queued for core0 serial printing.
#ifndef CORE1_SERIAL_LOG
#define CORE1_SERIAL_LOG 1  // 1 enables core1 diagnostic queue logging.
#endif

// Cross-core queue sizing and per-loop drain limits. Larger values reduce
// drops during dense scans or multi-scanner bursts at the cost of more RAM and
// longer core0 drain slices.
#ifndef SCAN_RESULT_QUEUE_DEPTH
#define SCAN_RESULT_QUEUE_DEPTH 1024
#endif
#ifndef SERIAL_MSG_QUEUE_DEPTH
#define SERIAL_MSG_QUEUE_DEPTH 64
#endif
#ifndef CONTROLLER_SCAN_RESULT_DRAIN_PER_LOOP
#define CONTROLLER_SCAN_RESULT_DRAIN_PER_LOOP 64
#endif
#ifndef CONTROLLER_SERIAL_MSG_DRAIN_PER_LOOP
#define CONTROLLER_SERIAL_MSG_DRAIN_PER_LOOP 16
#endif

// Hardware watchdog timeout in milliseconds. Core0 feeds the hardware watchdog
// only while core1's heartbeat remains fresh; otherwise the watchdog expires
// and reboots the controller. Set to 0 to disable the watchdog entirely.
#ifndef CONTROLLER_WATCHDOG_TIMEOUT_MS
#define CONTROLLER_WATCHDOG_TIMEOUT_MS 8000
#endif

// SD-card firmware update settings for RP2350 controller builds. The incoming
// image is staged from SD into internal LittleFS, then the OTA bootloader
// applies it on reboot.
#ifndef CONTROLLER_SD_UPDATE_ENABLED
#define CONTROLLER_SD_UPDATE_ENABLED 1
#endif
#ifndef CONTROLLER_SD_UPDATE_PATH
#define CONTROLLER_SD_UPDATE_PATH "/controller.bin"
#endif
#ifndef CONTROLLER_SD_UPDATE_APPLIED_PATH
#define CONTROLLER_SD_UPDATE_APPLIED_PATH "/controller.applied.bin"
#endif
#ifndef CONTROLLER_SD_UPDATE_CHUNK_BYTES
#define CONTROLLER_SD_UPDATE_CHUNK_BYTES 1024
#endif

// Periodic runtime/GPS status print interval in milliseconds.
// Set to 0 to disable the periodic summary lines entirely.
#ifndef PERIODIC_STATUS_INTERVAL_MS
#define PERIODIC_STATUS_INTERVAL_MS 5000
#endif

// Log routine successful scan-start messages from the controller scheduler.
// Warning/error scan diagnostics still print regardless.
#ifndef SCAN_LOG_NORMAL_STARTS
#define SCAN_LOG_NORMAL_STARTS 0  // 1 enables logging of all successful scan starts.
#endif

// Log number of controller-side dedupe hits per scan when the buffered results are emitted.
#ifndef LOG_DEDUPE_HITS
#define LOG_DEDUPE_HITS 0  // 1 enables logging of dedupe hits.
#endif

// Log each individual controller-side dedupe hit. The aggregate per-scan
// suppressed summary still follows LOG_DEDUPE_HITS.
#ifndef CONTROLLER_LOG_INDIVIDUAL_DEDUPE_HITS
#define CONTROLLER_LOG_INDIVIDUAL_DEDUPE_HITS 0
#endif

// Reset controller/scanner dedupe state when GNSS transitions from no usable
// fix to a usable location lock. A cooldown prevents repeated resets if the
// lock flaps.
#ifndef CONTROLLER_DEDUPE_RESET_ON_FIX_ACQUIRE
#define CONTROLLER_DEDUPE_RESET_ON_FIX_ACQUIRE 1
#endif
#ifndef CONTROLLER_DEDUPE_RESET_COOLDOWN_MS
#define CONTROLLER_DEDUPE_RESET_COOLDOWN_MS 15000
#endif

// SPI delay between command and follow-up polling frame in microseconds.
#ifndef ESP_INTERFRAME_US
#define ESP_INTERFRAME_US 10  // Delay between SPI command and response pulls.
#endif

#if defined(USE_TINYUSB)
// USB string descriptor defaults for TinyUSB builds.
#ifndef USB_DEVICE_MANUFACTURER_NAME
#define USB_DEVICE_MANUFACTURER_NAME "CoD_Segfault"  // USB manufacturer string.
#endif
#ifndef USB_DEVICE_PRODUCT_NAME
#define USB_DEVICE_PRODUCT_NAME "WiFi Shuriken"  // USB product string.
#endif
#ifndef USB_DEVICE_SERIAL_OVERRIDE
#define USB_DEVICE_SERIAL_OVERRIDE ""  // Optional USB serial string override.
#endif
#ifndef USB_CDC0_IFACE_NAME
#define USB_CDC0_IFACE_NAME "Console"  // Primary CDC interface label.
#endif
#ifndef USB_CDC1_IFACE_NAME
#define USB_CDC1_IFACE_NAME "NMEA"  // Secondary CDC interface label.
#endif
#endif
