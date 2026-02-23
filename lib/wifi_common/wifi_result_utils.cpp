// wifi_result_utils.cpp
#include "wifi_result_utils.h"

uint8_t wifiBandFromChannel(uint8_t channel) {
  if (channel >= 1 && channel <= 14) {
    return WIFI_BAND_24_GHZ;
  }
  if (channel >= 36 && channel <= 165) {
    return WIFI_BAND_5_GHZ;
  }
  return 0;
}

bool wifiBandChannelValid(uint8_t band, uint8_t channel) {
  if (band == WIFI_BAND_24_GHZ) {
    return channel >= 1 && channel <= 14;
  }
  if (band == WIFI_BAND_5_GHZ) {
    return channel >= 36 && channel <= 165;
  }
  return false;
}

bool wifiBssidIsZero(const uint8_t* bssid, size_t len) {
  if (!bssid || len == 0) {
    return true;
  }
  for (size_t i = 0; i < len; i++) {
    if (bssid[i] != 0x00) {
      return false;
    }
  }
  return true;
}

bool wifiBssidIsBroadcast(const uint8_t* bssid, size_t len) {
  if (!bssid || len == 0) {
    return false;
  }
  for (size_t i = 0; i < len; i++) {
    if (bssid[i] != 0xFF) {
      return false;
    }
  }
  return true;
}

bool wifiResultHasTerminatedSsid(const WiFiResult& result) {
  for (size_t i = 0; i < sizeof(result.ssid); i++) {
    if (result.ssid[i] == '\0') {
      return true;
    }
  }
  return false;
}

bool wifiResultIsValidForDedupe(const WiFiResult& result) {
  if (!wifiBandChannelValid(result.band, result.channel)) {
    return false;
  }
  if (wifiBandFromChannel(result.channel) != result.band) {
    return false;
  }
  if (wifiBssidIsZero(result.bssid) || wifiBssidIsBroadcast(result.bssid)) {
    return false;
  }
  if (!wifiResultHasTerminatedSsid(result)) {
    return false;
  }
  return true;
}

