// wifi_result.h
#pragma once
#include <stdint.h>

#define WIFI_SSID_MAX_LEN 32

typedef struct __attribute__((packed)) {
    char     ssid[WIFI_SSID_MAX_LEN + 1];
    uint8_t  bssid[6];
    int8_t   rssi;
    uint8_t  channel;
    uint8_t  band;
    uint16_t capabilities;
} WiFiResult;