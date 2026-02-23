// wifi_dedupe.h
#pragma once

#include <stddef.h>
#include <stdint.h>
#include "../../include/spi_protocol_shared.h"

#ifndef WIFI_DEDUPE_HASH_BITS
#define WIFI_DEDUPE_HASH_BITS 64
#endif

#if WIFI_DEDUPE_HASH_BITS == 32
static constexpr size_t WIFI_DEDUPE_HASH_SIZE = 4;
#elif WIFI_DEDUPE_HASH_BITS == 64
static constexpr size_t WIFI_DEDUPE_HASH_SIZE = 8;
#elif WIFI_DEDUPE_HASH_BITS == 128
static constexpr size_t WIFI_DEDUPE_HASH_SIZE = 16;
#else
#error "WIFI_DEDUPE_HASH_BITS must be one of: 32, 64, 128"
#endif

static constexpr uint16_t WIFI_DEDUPE_TABLE_CAPACITY = 4000;

struct WiFiDedupeHash {
  uint8_t bytes[WIFI_DEDUPE_HASH_SIZE];
};

struct WiFiDedupeTable {
  WiFiDedupeHash* entries;
  uint16_t capacity;
  uint16_t count;
  uint16_t write_index;
};

// xxHash of WiFiResult fields except RSSI:
// ssid[33], bssid[6], channel, band, capabilities.
void wifiDedupeHashFromResult(const WiFiResult& result, WiFiDedupeHash& out);

bool wifiDedupeHashEquals(const WiFiDedupeHash& a, const WiFiDedupeHash& b);

void wifiDedupeTableInit(WiFiDedupeTable* table, WiFiDedupeHash* entries, uint16_t capacity);
void wifiDedupeTableReset(WiFiDedupeTable* table);
bool wifiDedupeTableContains(const WiFiDedupeTable* table, const WiFiDedupeHash* hash);

// Returns true when inserted as new, false when duplicate (or invalid table).
bool wifiDedupeTableRemember(WiFiDedupeTable* table, const WiFiDedupeHash* hash);
