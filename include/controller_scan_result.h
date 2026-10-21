#pragma once
#include <stdint.h>

namespace pico_logging {
// Transport-neutral queued result, shared by scanner workers and loggers.
struct QueuedScanResult {
  char ssid[33];
  uint8_t bssid[6];
  int8_t rssi;
  uint8_t channel;
  uint8_t band;
  uint16_t capabilities;
};
}  // namespace pico_logging
