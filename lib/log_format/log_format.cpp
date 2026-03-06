#include "log_format.h"

#include <stdio.h>
#include <string.h>

#include "spi_protocol_shared.h"

namespace pico_logging {
namespace {

void appendAuthToken(char* out, size_t out_len, const char* token) {
  if (!out || out_len == 0 || !token || token[0] == '\0') {
    return;
  }

  const size_t used = strlen(out);
  if (used >= (out_len - 1)) {
    return;
  }

  const size_t remaining = out_len - used - 1;
  strncat(out, token, remaining);
  out[out_len - 1] = '\0';
}

void appendProtocolAuthToken(char* out,
                             size_t out_len,
                             const char* proto,
                             bool psk,
                             bool eap,
                             bool tkip,
                             bool ccmp) {
  char token[48] = {};
  size_t pos = 0;
  pos += (size_t)snprintf(token + pos, sizeof(token) - pos, "[%s", proto);

  if (psk) {
    pos += (size_t)snprintf(token + pos, sizeof(token) - pos, "-PSK");
  } else if (eap) {
    pos += (size_t)snprintf(token + pos, sizeof(token) - pos, "-EAP");
  }

  if (tkip && ccmp) {
    pos += (size_t)snprintf(token + pos, sizeof(token) - pos, "-CCMP+TKIP");
  } else if (ccmp) {
    pos += (size_t)snprintf(token + pos, sizeof(token) - pos, "-CCMP");
  } else if (tkip) {
    pos += (size_t)snprintf(token + pos, sizeof(token) - pos, "-TKIP");
  }

  (void)snprintf(token + pos, sizeof(token) - pos, "]");
  appendAuthToken(out, out_len, token);
}

}  // namespace

void formatBssid(const uint8_t* bssid, char* out, size_t out_len) {
  snprintf(out, out_len, "%02X:%02X:%02X:%02X:%02X:%02X",
           bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
}

void csvEscape(const char* in, char* out, size_t out_len) {
  size_t j = 0;
  if (out_len == 0) return;
  out[j++] = '"';
  for (size_t i = 0; in[i] != '\0' && j + 2 < out_len; i++) {
    if (in[i] == '"') {
      if (j + 2 >= out_len) break;
      out[j++] = '"';
      out[j++] = '"';
    } else {
      out[j++] = in[i];
    }
  }
  out[j++] = '"';
  out[j] = '\0';
}

void buildAndroidCapabilitiesString(uint16_t caps, char* out, size_t out_len) {
  if (!out || out_len == 0) {
    return;
  }

  out[0] = '\0';

  const bool has_wep = (caps & CAP_WEP) != 0;
  const bool has_wpa = (caps & CAP_WPA) != 0;
  const bool has_wpa2 = (caps & CAP_WPA2) != 0;
  const bool has_wpa3 = (caps & CAP_WPA3) != 0;
  const bool has_psk = (caps & CAP_PSK) != 0;
  const bool has_eap = (caps & CAP_EAP) != 0;
  const bool has_tkip = (caps & CAP_TKIP) != 0;
  const bool has_ccmp = (caps & CAP_CCMP) != 0;

  if (has_wep) {
    appendAuthToken(out, out_len, "[WEP]");
  }
  if (has_wpa) {
    appendProtocolAuthToken(out, out_len, "WPA", has_psk, has_eap, has_tkip, has_ccmp);
  }
  if (has_wpa2) {
    appendProtocolAuthToken(out, out_len, "WPA2", has_psk, has_eap, has_tkip, has_ccmp);
  }
  if (has_wpa3) {
    appendProtocolAuthToken(out, out_len, "RSN", has_psk, has_eap, has_tkip, has_ccmp);
  }

  if (out[0] == '\0') {
    appendAuthToken(out, out_len, "[OPEN]");
  }
  if ((caps & CAP_ESS) != 0) {
    appendAuthToken(out, out_len, "[ESS]");
  }
}

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
                     size_t out_len) {
  if (!out || out_len == 0) {
    return false;
  }

  const uint8_t zero_bssid[6] = {0, 0, 0, 0, 0, 0};
  if (!bssid) {
    bssid = zero_bssid;
  }

  char mac[18] = {};
  char ssid_csv[80] = {};
  char auth_mode[96] = {};

  formatBssid(bssid, mac, sizeof(mac));
  csvEscape(ssid ? ssid : "", ssid_csv, sizeof(ssid_csv));
  buildAndroidCapabilitiesString(capabilities, auth_mode, sizeof(auth_mode));

  const int written = snprintf(out, out_len, "%s,%s,%s,%s,%u,%d,%s,%s,%s,%s,WIFI\n",
                               mac,
                               ssid_csv,
                               auth_mode,
                               first_seen ? first_seen : "",
                               (unsigned)channel,
                               (int)rssi,
                               latitude ? latitude : "",
                               longitude ? longitude : "",
                               altitude_meters ? altitude_meters : "",
                               accuracy_meters ? accuracy_meters : "");
  if (written < 0) {
    out[0] = '\0';
    return false;
  }
  if ((size_t)written >= out_len) {
    out[out_len - 1] = '\0';
    return false;
  }
  return true;
}

}  // namespace pico_logging

