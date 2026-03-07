// scanner_platform_esp32_spi.cpp
#include "scanner_platform_esp32_spi.h"
#include "ScannerConfiguration.h"

#include <driver/gpio.h>
#include <driver/spi_slave.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

// Keep a single persistent queued transaction so timeouts do not leave
// the SPI ISR with a dangling pointer to a stack-local transaction object.
static spi_slave_transaction_t g_pending_trans = {};
static bool g_trans_queued = false;
static bool g_spi_initialized = false;

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

  const esp_err_t ret = spi_slave_initialize(
    SPI2_HOST, &buscfg, &slvcfg, static_cast<spi_dma_chan_t>(SCANNER_SPI_DMA_CH));
  if (ret != ESP_OK) {
    return false;
  }

  const esp_err_t drive_ret = gpio_set_drive_capability(
    static_cast<gpio_num_t>(pin_miso),
    static_cast<gpio_drive_cap_t>(SCANNER_SPI_MISO_DRIVE_CAP));
  if (drive_ret != ESP_OK) {
    return false;
  }

  memset(&g_pending_trans, 0, sizeof(g_pending_trans));
  g_trans_queued = false;
  g_spi_initialized = true;
  return true;
}

int scannerSpiSlaveTransfer(const uint8_t* tx_buf,
                            uint8_t* rx_buf,
                            size_t frame_size,
                            uint32_t timeout_ms) {
  if (!g_spi_initialized) {
    return SCANNER_SPI_TRANSFER_ERROR;
  }

  // Queue one persistent transaction if none is currently pending.
  if (!g_trans_queued) {
    memset(&g_pending_trans, 0, sizeof(g_pending_trans));
    g_pending_trans.length = frame_size * 8;
    g_pending_trans.tx_buffer = tx_buf;
    g_pending_trans.rx_buffer = rx_buf;
    const esp_err_t qret = spi_slave_queue_trans(SPI2_HOST, &g_pending_trans, 0);
    if (qret != ESP_OK) {
      return SCANNER_SPI_TRANSFER_ERROR;
    }
    g_trans_queued = true;
  }

  spi_slave_transaction_t* completed = nullptr;
  const esp_err_t ret = spi_slave_get_trans_result(
    SPI2_HOST, &completed, pdMS_TO_TICKS(timeout_ms));
  if (ret == ESP_ERR_TIMEOUT) {
    return SCANNER_SPI_TRANSFER_TIMEOUT;
  }
  if (ret != ESP_OK) {
    g_trans_queued = false;
    memset(&g_pending_trans, 0, sizeof(g_pending_trans));
    return SCANNER_SPI_TRANSFER_ERROR;
  }
  if (completed != &g_pending_trans) {
    g_trans_queued = false;
    memset(&g_pending_trans, 0, sizeof(g_pending_trans));
    return SCANNER_SPI_TRANSFER_ERROR;
  }

  g_trans_queued = false;
  return SCANNER_SPI_TRANSFER_OK;
}
