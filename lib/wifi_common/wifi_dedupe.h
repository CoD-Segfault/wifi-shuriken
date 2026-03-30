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

// Maximum number of live entries retained at any time. Sized for spatial
// dedup density: large enough to suppress re-logging the same AP as you drive
// past, small enough that APs fall off as you move away so re-observation from
// a different direction still produces a new log entry.
static constexpr uint16_t WIFI_DEDUPE_TABLE_CAPACITY = 4000;

// Open-addressing hash table slot count. Must be a power of 2 and at least
// 2x WIFI_DEDUPE_TABLE_CAPACITY to keep the load factor ≤ 50%.
static constexpr uint16_t WIFI_DEDUPE_TABLE_SIZE = 8192;

struct WiFiDedupeHash {
  uint8_t bytes[WIFI_DEDUPE_HASH_SIZE];
};

// Open-addressing hash table with FIFO eviction.
//
// - slots[table_size]: the hash table itself; all-zeros is the empty sentinel.
// - fifo[capacity]:    eviction queue storing hash values in insertion order.
//
// Lookup is O(1) average (~1-2 probes at 49% load). Eviction pops the oldest
// hash from the FIFO, locates its current slot via a fresh O(1) probe, and
// removes it using backward-shift deletion (no tombstones).
struct WiFiDedupeTable {
  WiFiDedupeHash* slots;      // open-addressing table, table_size entries
  WiFiDedupeHash* fifo;       // eviction queue, capacity entries
  uint16_t table_size;        // power-of-2, must be >= 2 * capacity
  uint16_t capacity;          // max live entries
  uint16_t count;             // current live entries
  uint16_t fifo_head;         // oldest entry (next to evict)
  uint16_t fifo_tail;         // next write position in fifo
};

// xxHash of WiFiResult fields except RSSI:
// ssid[33], bssid[6], channel, band, capabilities.
void wifiDedupeHashFromResult(const WiFiResult& result, WiFiDedupeHash& out);

bool wifiDedupeHashEquals(const WiFiDedupeHash& a, const WiFiDedupeHash& b);

void wifiDedupeTableInit(WiFiDedupeTable* table,
                         WiFiDedupeHash* slots, uint16_t table_size,
                         WiFiDedupeHash* fifo, uint16_t capacity);
void wifiDedupeTableReset(WiFiDedupeTable* table);
bool wifiDedupeTableContains(const WiFiDedupeTable* table, const WiFiDedupeHash* hash);

// Returns true when inserted as new, false when duplicate (or invalid table).
bool wifiDedupeTableRemember(WiFiDedupeTable* table, const WiFiDedupeHash* hash);
