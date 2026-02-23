// wifi_result_utils.h
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "../../include/spi_protocol_shared.h"

static constexpr uint8_t WIFI_BAND_24_GHZ = 2;
static constexpr uint8_t WIFI_BAND_5_GHZ = 5;

uint8_t wifiBandFromChannel(uint8_t channel);
bool wifiBandChannelValid(uint8_t band, uint8_t channel);

bool wifiBssidIsZero(const uint8_t* bssid, size_t len = 6);
bool wifiBssidIsBroadcast(const uint8_t* bssid, size_t len = 6);

bool wifiResultHasTerminatedSsid(const WiFiResult& result);
bool wifiResultIsValidForDedupe(const WiFiResult& result);

