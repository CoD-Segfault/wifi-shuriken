// scanner_platform_esp32_spi.cpp
#include "scanner_platform_esp32_spi.h"

#include <driver/spi_slave.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

bool scannerSpiSlaveInit(int pin_sck,
                         int pin_mosi,
                         int pin_miso,
                         int pin_cs,
                         size_t frame_size,
                         uint8_t queue_size) {
  // ESP32-C5 slave mode runs on SPI2 host; fixed frame_size keeps DMA/simple framing aligned.
  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = pin_mosi;
  buscfg.miso_io_num = pin_miso;
  buscfg.sclk_io_num = pin_sck;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = frame_size;

  spi_slave_interface_config_t slvcfg = {};
  slvcfg.spics_io_num = pin_cs;
  slvcfg.queue_size = queue_size;
  slvcfg.mode = 0;

  const esp_err_t ret = spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
  return ret == ESP_OK;
}

int scannerSpiSlaveTransfer(const uint8_t* tx_buf,
                            uint8_t* rx_buf,
                            size_t frame_size,
                            uint32_t timeout_ms) {
  // One full-duplex transaction clocks one command in and one response out.
  spi_slave_transaction_t trans = {};
  trans.length = frame_size * 8;
  trans.tx_buffer = tx_buf;
  trans.rx_buffer = rx_buf;
  const esp_err_t ret = spi_slave_transmit(SPI2_HOST, &trans, pdMS_TO_TICKS(timeout_ms));
  if (ret == ESP_ERR_TIMEOUT) {
    return SCANNER_SPI_TRANSFER_TIMEOUT;
  }
  if (ret != ESP_OK) {
    return SCANNER_SPI_TRANSFER_ERROR;
  }
  return SCANNER_SPI_TRANSFER_OK;
}
