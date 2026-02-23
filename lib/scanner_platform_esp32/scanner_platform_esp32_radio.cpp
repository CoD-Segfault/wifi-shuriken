// scanner_platform_esp32_radio.cpp
#include "scanner_platform_esp32_radio.h"

#include <string.h>

#include <Arduino.h>
#include <WiFi.h>

#include "wifi_result_utils.h"

// Map ESP-IDF auth modes into transport-stable capability bits.
// This intentionally captures security-family intent; detailed RSN IE parsing
// is out of scope for this backend abstraction.
static uint16_t scannerCapabilitiesFromAuthMode(wifi_auth_mode_t auth) {
  uint16_t caps = CAP_ESS;

  switch (auth) {
    case WIFI_AUTH_OPEN:
      break;
    case WIFI_AUTH_WEP:
      caps |= CAP_WEP;
      break;
    case WIFI_AUTH_WPA_PSK:
      caps |= (CAP_WPA | CAP_PSK | CAP_TKIP);
      break;
    case WIFI_AUTH_WPA2_PSK:
      caps |= (CAP_WPA2 | CAP_PSK | CAP_CCMP);
      break;
    case WIFI_AUTH_WPA_WPA2_PSK:
      caps |= (CAP_WPA | CAP_WPA2 | CAP_PSK | CAP_TKIP | CAP_CCMP);
      break;
    case WIFI_AUTH_WPA2_ENTERPRISE:
      caps |= (CAP_WPA2 | CAP_EAP | CAP_CCMP);
      break;
    case WIFI_AUTH_WPA3_PSK:
      caps |= (CAP_WPA3 | CAP_PSK | CAP_CCMP);
      break;
    case WIFI_AUTH_WPA2_WPA3_PSK:
      caps |= (CAP_WPA2 | CAP_WPA3 | CAP_PSK | CAP_CCMP);
      break;
#if defined(WIFI_AUTH_WPA3_ENT_192)
    case WIFI_AUTH_WPA3_ENT_192:
      caps |= (CAP_WPA3 | CAP_EAP | CAP_CCMP);
      break;
#endif
#if defined(WIFI_AUTH_WPA3_EXT_PSK)
    case WIFI_AUTH_WPA3_EXT_PSK:
      caps |= (CAP_WPA3 | CAP_PSK | CAP_CCMP);
      break;
#endif
#if defined(WIFI_AUTH_WPA3_EXT_PSK_MIXED_MODE)
    case WIFI_AUTH_WPA3_EXT_PSK_MIXED_MODE:
      caps |= (CAP_WPA2 | CAP_WPA3 | CAP_PSK | CAP_CCMP);
      break;
#endif
#if defined(WIFI_AUTH_OWE)
    case WIFI_AUTH_OWE:
      // Enhanced open; no matching flag in current protocol bitset.
      break;
#endif
#if defined(WIFI_AUTH_WAPI_PSK)
    case WIFI_AUTH_WAPI_PSK:
      // No WAPI bit in current protocol bitset.
      break;
#endif
    default:
      break;
  }

  return caps;
}

bool scannerRadioInit() {
  // STA mode only; disconnect to force clean scan behavior after reboot.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  return true;
}

int scannerRadioStartAsyncScan(uint8_t /*band*/, uint8_t channel, bool passive, int dwell_ms) {
  // band is currently implicit via channel on this backend.
  const int ret = WiFi.scanNetworks(true, true, passive, dwell_ms, channel);
  if (ret == WIFI_SCAN_FAILED) {
    return SCANNER_RADIO_SCAN_FAILED;
  }
  return ret;
}

int scannerRadioPollAsyncScan() {
  // scanComplete() returns WIFI_SCAN_RUNNING while active, >=0 when complete.
  const int ret = WiFi.scanComplete();
  if (ret == WIFI_SCAN_RUNNING) {
    return SCANNER_RADIO_SCAN_RUNNING;
  }
  if (ret < 0) {
    return SCANNER_RADIO_SCAN_FAILED;
  }
  return ret;
}

void scannerRadioDeleteScanResults() {
  WiFi.scanDelete();
}

bool scannerRadioReadResult(int index, WiFiResult& out) {
  if (index < 0) {
    return false;
  }

  memset(&out, 0, sizeof(out));
  String ssid = WiFi.SSID(index);
  ssid.toCharArray(out.ssid, sizeof(out.ssid));
  WiFi.BSSID(index, out.bssid);
  out.rssi = static_cast<int8_t>(WiFi.RSSI(index));
  out.channel = static_cast<uint8_t>(WiFi.channel(index));
  out.band = wifiBandFromChannel(out.channel);
  out.capabilities = scannerCapabilitiesFromAuthMode(WiFi.encryptionType(index));
  return true;
}
