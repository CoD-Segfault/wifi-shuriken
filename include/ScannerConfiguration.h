#pragma once

// Scanner SPI slave pin mapping for the primary XIAO ESP32-C5 target.
#ifndef PIN_SCK
#define PIN_SCK 8  // XIAO ESP32-C5 default SPI slave clock pin.
#endif
#ifndef PIN_MOSI
#define PIN_MOSI 10  // XIAO ESP32-C5 default SPI slave MOSI pin.
#endif
#ifndef PIN_MISO
#define PIN_MISO 9  // XIAO ESP32-C5 default SPI slave MISO pin.
#endif
#ifndef PIN_CS
#define PIN_CS 7  // XIAO ESP32-C5 default SPI slave chip-select pin.
#endif

// Scanner status LED wiring.
#ifndef SCANNER_STATUS_LED_ENABLED
#define SCANNER_STATUS_LED_ENABLED 1  // 1 enables the status LED output.
#endif
#ifndef SCANNER_STATUS_LED_PIN
#define SCANNER_STATUS_LED_PIN 27  // XIAO ESP32-C5 default status LED pin.
#endif
#ifndef SCANNER_STATUS_LED_ACTIVE_LOW
#define SCANNER_STATUS_LED_ACTIVE_LOW 1  // XIAO ESP32-C5 LED is active-low.
#endif

// ESP32 scanner SPI electrical tuning.
#ifndef SCANNER_SPI_MISO_DRIVE_CAP
#define SCANNER_SPI_MISO_DRIVE_CAP GPIO_DRIVE_CAP_3  // Strongest MISO drive.
#endif
#ifndef SCANNER_SPI_DMA_CH
#define SCANNER_SPI_DMA_CH SPI_DMA_CH_AUTO  // ESP32-C5 requires auto DMA allocation.
#endif

// Scanner debug logging.
#ifndef SCANNER_SERIAL_LOG
#define SCANNER_SERIAL_LOG 1  // 1 enables scanner serial status/progress logging.
#endif
#ifndef DEBUG_SPI_PROTOCOL
#define DEBUG_SPI_PROTOCOL 0  // 1 enables verbose SPI protocol tracing.
#endif

// Focused scanner-side diagnostics for SPI OTA bring-up. Logs early packets,
// flash-sector boundary packets, validation failures, and slow flash writes.
#ifndef SCANNER_OTA_DEBUG_LOG
#define SCANNER_OTA_DEBUG_LOG 0
#endif
#ifndef SCANNER_OTA_DEBUG_SECTOR_BYTES
#define SCANNER_OTA_DEBUG_SECTOR_BYTES 4096
#endif
#if SCANNER_OTA_DEBUG_SECTOR_BYTES < 1
#error "SCANNER_OTA_DEBUG_SECTOR_BYTES must be at least 1"
#endif
#ifndef SCANNER_OTA_DEBUG_EDGE_BYTES
#define SCANNER_OTA_DEBUG_EDGE_BYTES 128
#endif
#ifndef SCANNER_OTA_DEBUG_SLOW_WRITE_MS
#define SCANNER_OTA_DEBUG_SLOW_WRITE_MS 20
#endif
