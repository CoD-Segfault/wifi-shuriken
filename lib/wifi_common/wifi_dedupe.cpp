// wifi_dedupe.cpp
#include "wifi_dedupe.h"

#include <string.h>
#include <xxhash.h>

void wifiDedupeHashFromResult(const WiFiResult& result, WiFiDedupeHash& out) {
  uint8_t material[43] = {};
  size_t offset = 0;

  memcpy(material + offset, result.ssid, sizeof(result.ssid));
  offset += sizeof(result.ssid);

  memcpy(material + offset, result.bssid, sizeof(result.bssid));
  offset += sizeof(result.bssid);

  material[offset++] = result.channel;
  material[offset++] = result.band;
  material[offset++] = (uint8_t)(result.capabilities & 0xFF);
  material[offset++] = (uint8_t)((result.capabilities >> 8) & 0xFF);

#if WIFI_DEDUPE_HASH_BITS == 32
  const XXH32_hash_t hash = XXH32(material, sizeof(material), 0);
  XXH32_canonical_t canonical = {};
  XXH32_canonicalFromHash(&canonical, hash);
  memcpy(out.bytes, canonical.digest, WIFI_DEDUPE_HASH_SIZE);
#elif WIFI_DEDUPE_HASH_BITS == 64
  const XXH64_hash_t hash = XXH3_64bits(material, sizeof(material));
  XXH64_canonical_t canonical = {};
  XXH64_canonicalFromHash(&canonical, hash);
  memcpy(out.bytes, canonical.digest, WIFI_DEDUPE_HASH_SIZE);
#elif WIFI_DEDUPE_HASH_BITS == 128
  const XXH128_hash_t hash = XXH3_128bits(material, sizeof(material));
  XXH128_canonical_t canonical = {};
  XXH128_canonicalFromHash(&canonical, hash);
  memcpy(out.bytes, canonical.digest, WIFI_DEDUPE_HASH_SIZE);
#endif
}

bool wifiDedupeHashEquals(const WiFiDedupeHash& a, const WiFiDedupeHash& b) {
  return memcmp(a.bytes, b.bytes, WIFI_DEDUPE_HASH_SIZE) == 0;
}

// Returns true if the slot contains the empty sentinel (all-zero bytes).
static bool slotIsEmpty(const WiFiDedupeHash* h) {
  for (size_t i = 0; i < WIFI_DEDUPE_HASH_SIZE; i++) {
    if (h->bytes[i] != 0) return false;
  }
  return true;
}

// Maps a hash to its initial probe slot using the first two bytes as an index.
// table_size must be a power of 2.
static uint16_t hashToSlot(const WiFiDedupeHash* h, uint16_t table_size) {
  uint16_t idx = 0;
  memcpy(&idx, h->bytes, sizeof(idx));
  return (uint16_t)(idx & (uint16_t)(table_size - 1u));
}

// Removes the entry at `removed` and shifts subsequent entries in the same
// probe chain back toward their natural positions. Leaves no tombstones.
static void backwardShiftDelete(WiFiDedupeHash* slots,
                                uint16_t table_size,
                                uint16_t removed) {
  const uint16_t mask = (uint16_t)(table_size - 1u);
  memset(&slots[removed], 0, sizeof(WiFiDedupeHash));
  uint16_t gap = removed;
  uint16_t scan = (uint16_t)((removed + 1u) & mask);
  while (!slotIsEmpty(&slots[scan])) {
    const uint16_t natural = hashToSlot(&slots[scan], table_size);
    // Move entry from scan to gap if doing so keeps it at or closer to its
    // natural position (i.e. gap is within the probe chain from natural to scan).
    const uint16_t dist_to_gap  = (uint16_t)((gap  - natural + table_size) & mask);
    const uint16_t dist_to_scan = (uint16_t)((scan - natural + table_size) & mask);
    if (dist_to_gap <= dist_to_scan) {
      slots[gap] = slots[scan];
      memset(&slots[scan], 0, sizeof(WiFiDedupeHash));
      gap = scan;
    }
    scan = (uint16_t)((scan + 1u) & mask);
  }
}

void wifiDedupeTableInit(WiFiDedupeTable* table,
                         WiFiDedupeHash* slots, uint16_t table_size,
                         WiFiDedupeHash* fifo, uint16_t capacity) {
  if (!table) return;
  table->slots = slots;
  table->fifo = fifo;
  table->table_size = table_size;
  table->capacity = capacity;
  table->count = 0;
  table->fifo_head = 0;
  table->fifo_tail = 0;
  if (slots && table_size > 0) {
    memset(slots, 0, (size_t)table_size * sizeof(WiFiDedupeHash));
  }
}

void wifiDedupeTableReset(WiFiDedupeTable* table) {
  if (!table) return;
  table->count = 0;
  table->fifo_head = 0;
  table->fifo_tail = 0;
  if (table->slots && table->table_size > 0) {
    memset(table->slots, 0, (size_t)table->table_size * sizeof(WiFiDedupeHash));
  }
}

bool wifiDedupeTableContains(const WiFiDedupeTable* table,
                              const WiFiDedupeHash* hash) {
  if (!table || !hash || !table->slots || table->table_size == 0) return false;
  // All-zero hashes are used as the empty sentinel and can never be stored.
  if (slotIsEmpty(hash)) return false;
  const uint16_t mask = (uint16_t)(table->table_size - 1u);
  uint16_t slot = hashToSlot(hash, table->table_size);
  while (!slotIsEmpty(&table->slots[slot])) {
    if (memcmp(table->slots[slot].bytes, hash->bytes, WIFI_DEDUPE_HASH_SIZE) == 0) {
      return true;
    }
    slot = (uint16_t)((slot + 1u) & mask);
  }
  return false;
}

bool wifiDedupeTableRemember(WiFiDedupeTable* table, const WiFiDedupeHash* hash) {
  if (!table || !hash || !table->slots || !table->fifo ||
      table->table_size == 0 || table->capacity == 0) {
    return false;
  }
  // All-zero hashes collide with the empty sentinel and cannot be stored.
  if (slotIsEmpty(hash)) return false;

  // Corruption guard: clamp out-of-range metadata before any access.
  if (table->count > table->capacity) table->count = table->capacity;
  if (table->fifo_head >= table->capacity) table->fifo_head = 0;
  if (table->fifo_tail >= table->capacity) table->fifo_tail = 0;

  if (wifiDedupeTableContains(table, hash)) return false;

  const uint16_t mask = (uint16_t)(table->table_size - 1u);

  if (table->count == table->capacity) {
    bool evicted = false;
    for (uint16_t attempts = 0; attempts < table->capacity; attempts++) {
      // Evict the oldest entry: retrieve its hash from the FIFO, locate its
      // current slot via a fresh probe (O(1) expected), then backward-shift delete.
      const WiFiDedupeHash evict_hash = table->fifo[table->fifo_head];
      table->fifo_head = (uint16_t)((table->fifo_head + 1u) % table->capacity);

      if (slotIsEmpty(&evict_hash)) {
        continue;
      }

      uint16_t evict_slot = hashToSlot(&evict_hash, table->table_size);
      while (!slotIsEmpty(&table->slots[evict_slot])) {
        if (memcmp(table->slots[evict_slot].bytes,
                   evict_hash.bytes,
                   WIFI_DEDUPE_HASH_SIZE) == 0) {
          backwardShiftDelete(table->slots, table->table_size, evict_slot);
          table->count--;
          evicted = true;
          break;
        }
        evict_slot = (uint16_t)((evict_slot + 1u) & mask);
      }

      if (evicted) {
        break;
      }
    }

    if (!evicted) {
      // FIFO metadata no longer references any live table entry. Drop stale
      // dedupe state so the table can recover to a known-good configuration.
      wifiDedupeTableReset(table);
    }
  }

  // Find the first empty slot from the natural position.
  uint16_t insert_slot = hashToSlot(hash, table->table_size);
  while (!slotIsEmpty(&table->slots[insert_slot])) {
    insert_slot = (uint16_t)((insert_slot + 1u) & mask);
  }

  table->slots[insert_slot] = *hash;
  table->fifo[table->fifo_tail] = *hash;
  table->fifo_tail = (uint16_t)((table->fifo_tail + 1u) % table->capacity);
  table->count++;
  return true;
}
