#pragma once

// SD card SPI0 pin mapping on the Pico master.
#ifndef SD_PIN_SCK
#define SD_PIN_SCK 2  // SPI0 clock line to SD card.
#endif
#ifndef SD_PIN_MOSI
#define SD_PIN_MOSI 3  // SPI0 MOSI line to SD card.
#endif
#ifndef SD_PIN_MISO
#define SD_PIN_MISO 4  // SPI0 MISO line from SD card.
#endif
#ifndef SD_PIN_CS
#define SD_PIN_CS 5  // SD card chip-select pin.
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
#define ESP_PIN_SCK 10  // SPI1 clock line to scanner module.
#endif
#ifndef ESP_PIN_MOSI
#define ESP_PIN_MOSI 11  // SPI1 MOSI line to scanner module.
#endif
#ifndef ESP_PIN_MISO
#define ESP_PIN_MISO 12  // SPI1 MISO line from scanner module.
#endif

// Scanner CS wiring mode: 0 = direct GPIO CS, 1 = 74HC595 shift-register CS.
#ifndef SCANNER_USE_SHIFTREG_CS
#define SCANNER_USE_SHIFTREG_CS 0  // 0: direct CS pin, 1: shift-register CS.
#endif

// Direct GPIO CS pin when shift-register CS mode is disabled.
#if !SCANNER_USE_SHIFTREG_CS
#ifndef ESP_PIN_CS
#define ESP_PIN_CS 13  // Direct scanner chip-select pin.
#endif
#endif

// Shift-register control pins and SPI speed when SCANNER_USE_SHIFTREG_CS is enabled.
#ifndef SCANNER_SHIFTREG_LATCH_PIN
#define SCANNER_SHIFTREG_LATCH_PIN 15  // 74HC595 latch (RCLK) pin.
#endif
#ifndef SCANNER_SHIFTREG_OE_PIN
#define SCANNER_SHIFTREG_OE_PIN 16  // 74HC595 output-enable pin (active low).
#endif
#ifndef SCANNER_SHIFTREG_SPI_HZ
#define SCANNER_SHIFTREG_SPI_HZ 2000000  // Shift-register SPI clock in Hz.
#endif

// Number of active-low CS outputs driven by the shift register.
#ifndef SCANNER_SHIFTREG_OUTPUTS
#define SCANNER_SHIFTREG_OUTPUTS 8  // Number of usable CS outputs (1..8).
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
#define RGB_PIN 14  // WS2812 data pin.
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
#define GNSS_TX 8  // GNSS UART TX pin on Pico.
#endif
#ifndef GNSS_RX
#define GNSS_RX 9  // GNSS UART RX pin on Pico.
#endif
#ifndef GNSS_UART
#define GNSS_UART Serial2  // Hardware UART instance used for GNSS.
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

// SPI delay between command and follow-up polling frame in microseconds.
#ifndef ESP_INTERFRAME_US
#define ESP_INTERFRAME_US 100  // Delay between SPI command and response pulls.
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
