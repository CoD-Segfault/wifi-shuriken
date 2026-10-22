#include "arduino/update_config.h"
#include "Configuration.h"
namespace controller {
ScannerUpdateConfig configuredScannerUpdate() {
  ScannerUpdateConfig config;
  config.enabled = SCANNER_SPI_UPDATE_ENABLED != 0;
  config.image_path = SCANNER_SPI_UPDATE_PATH;
  config.applied_path = SCANNER_SPI_UPDATE_APPLIED_PATH;
  config.normal_hz = SCANNER_SPI_CLOCK;
  config.update_hz = SCANNER_SPI_UPDATE_CLOCK;
  config.first_pull_us = SCANNER_SPI_UPDATE_FIRST_PULL_US;
  config.identify_first_pull_us = SCANNER_STATUS_FIRST_PULL_DELAY_US;
  config.interframe_us = SCANNER_SPI_UPDATE_INTERFRAME_US;
  config.normal_interframe_us = ESP_INTERFRAME_US;
  config.response_pulls = SCANNER_SPI_UPDATE_RESPONSE_PULLS;
  config.packet_retries = SCANNER_SPI_UPDATE_PACKET_RETRIES;
  config.hash_chunk_bytes = SCANNER_SPI_UPDATE_FILE_CHUNK_BYTES;
  config.flash_sector_bytes = SCANNER_SPI_UPDATE_FLASH_SECTOR_BYTES;
  return config;
}
}
