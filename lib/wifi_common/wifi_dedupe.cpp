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

void wifiDedupeTableInit(WiFiDedupeTable* table, WiFiDedupeHash* entries, uint16_t capacity) {
  if (!table) {
    return;
  }
  table->entries = entries;
  table->capacity = capacity;
  table->count = 0;
  table->write_index = 0;
  if (table->entries && table->capacity > 0) {
    memset(table->entries, 0, (size_t)table->capacity * sizeof(WiFiDedupeHash));
  }
}

void wifiDedupeTableReset(WiFiDedupeTable* table) {
  if (!table) {
    return;
  }
  table->count = 0;
  table->write_index = 0;
  if (table->entries && table->capacity > 0) {
    memset(table->entries, 0, (size_t)table->capacity * sizeof(WiFiDedupeHash));
  }
}

bool wifiDedupeTableContains(const WiFiDedupeTable* table, const WiFiDedupeHash* hash) {
  if (!table || !hash || !table->entries || table->capacity == 0) {
    return false;
  }
  const uint16_t count = (table->count <= table->capacity) ? table->count : table->capacity;
  for (uint16_t i = 0; i < count; i++) {
    if (memcmp(table->entries[i].bytes, hash->bytes, WIFI_DEDUPE_HASH_SIZE) == 0) {
      return true;
    }
  }
  return false;
}

bool wifiDedupeTableRemember(WiFiDedupeTable* table, const WiFiDedupeHash* hash) {
  if (!table || !hash || !table->entries || table->capacity == 0) {
    return false;
  }

  // Guard against corrupted metadata to avoid out-of-bounds access.
  if (table->count > table->capacity) {
    table->count = table->capacity;
  }
  if (table->count == table->capacity && table->write_index >= table->capacity) {
    table->write_index = 0;
  }

  if (wifiDedupeTableContains(table, hash)) {
    return false;
  }

  if (table->count < table->capacity) {
    table->entries[table->count++] = *hash;
    if (table->count == table->capacity) {
      table->write_index = 0;
    }
  } else {
    table->entries[table->write_index] = *hash;
    table->write_index = (uint16_t)((table->write_index + 1) % table->capacity);
  }
  return true;
}
