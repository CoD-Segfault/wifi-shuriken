// scanner_platform_esp32_spi.h
#pragma once

#include <stddef.h>
#include <stdint.h>

enum ScannerSpiTransferResult : int {
  SCANNER_SPI_TRANSFER_ERROR = -1,
  SCANNER_SPI_TRANSFER_TIMEOUT = 0,
  SCANNER_SPI_TRANSFER_OK = 1,
};

bool scannerSpiSlaveInit(int pin_sck,
                         int pin_mosi,
                         int pin_miso,
                         int pin_cs,
                         size_t frame_size,
                         uint8_t queue_size);

int scannerSpiSlaveTransfer(const uint8_t* tx_buf,
                            uint8_t* rx_buf,
                            size_t frame_size,
                            uint32_t timeout_ms);

