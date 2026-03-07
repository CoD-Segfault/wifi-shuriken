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
#ifndef DEBUG_SPI_PROTOCOL
#define DEBUG_SPI_PROTOCOL 0  // 1 enables verbose SPI protocol tracing.
#endif
