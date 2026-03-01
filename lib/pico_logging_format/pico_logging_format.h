#pragma once

#include <stddef.h>
#include <stdint.h>

namespace pico_logging {

void formatBssid(const uint8_t* bssid, char* out, size_t out_len);
void csvEscape(const char* in, char* out, size_t out_len);
void buildAndroidCapabilitiesString(uint16_t caps, char* out, size_t out_len);
bool buildCsvWifiRow(const uint8_t* bssid,
                     const char* ssid,
                     uint16_t capabilities,
                     const char* first_seen,
                     uint8_t channel,
                     int8_t rssi,
                     const char* latitude,
                     const char* longitude,
                     const char* altitude_meters,
                     const char* accuracy_meters,
                     char* out,
                     size_t out_len);

}  // namespace pico_logging
