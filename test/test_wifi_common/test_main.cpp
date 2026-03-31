#include <string.h>

#include <unity.h>

#include "wifi_dedupe.h"
#include "wifi_result_utils.h"

void setUp() {}
void tearDown() {}

static WiFiResult makeValidResult() {
  WiFiResult r = {};
  strncpy(r.ssid, "TestAP", sizeof(r.ssid) - 1);
  const uint8_t bssid[6] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60};
  memcpy(r.bssid, bssid, sizeof(r.bssid));
  r.rssi = -42;
  r.channel = 36;
  r.band = WIFI_BAND_5_GHZ;
  r.capabilities = CAP_ESS | CAP_WPA2 | CAP_PSK | CAP_CCMP;
  return r;
}

static WiFiResult makeValid24gResult() {
  WiFiResult r = {};
  strncpy(r.ssid, "TwoFour", sizeof(r.ssid) - 1);
  const uint8_t bssid[6] = {0x61, 0x52, 0x43, 0x34, 0x25, 0x16};
  memcpy(r.bssid, bssid, sizeof(r.bssid));
  r.rssi = -55;
  r.channel = 11;
  r.band = WIFI_BAND_24_GHZ;
  r.capabilities = CAP_ESS | CAP_WPA | CAP_PSK | CAP_TKIP;
  return r;
}

static WiFiDedupeHash makeHash(uint8_t seed) {
  WiFiDedupeHash hash = {};
  for (size_t i = 0; i < WIFI_DEDUPE_HASH_SIZE; i++) {
    hash.bytes[i] = (uint8_t)(seed + i);
  }
  return hash;
}

// Helper: initialise a small table for testing. table_size must be a power of
// two and at least 2x capacity; slots and fifo must have table_size and
// capacity entries respectively.
static void initSmallTable(WiFiDedupeTable* table,
                           WiFiDedupeHash* slots, uint16_t table_size,
                           WiFiDedupeHash* fifo, uint16_t capacity) {
  wifiDedupeTableInit(table, slots, table_size, fifo, capacity);
}

void test_wifiBandFromChannel_boundaries() {
  TEST_ASSERT_EQUAL_UINT8(WIFI_BAND_24_GHZ, wifiBandFromChannel(1));
  TEST_ASSERT_EQUAL_UINT8(WIFI_BAND_24_GHZ, wifiBandFromChannel(14));
  TEST_ASSERT_EQUAL_UINT8(WIFI_BAND_5_GHZ, wifiBandFromChannel(36));
  TEST_ASSERT_EQUAL_UINT8(WIFI_BAND_5_GHZ, wifiBandFromChannel(165));
  TEST_ASSERT_EQUAL_UINT8(0, wifiBandFromChannel(0));
  TEST_ASSERT_EQUAL_UINT8(0, wifiBandFromChannel(35));
  TEST_ASSERT_EQUAL_UINT8(0, wifiBandFromChannel(166));
}

void test_wifiBandChannelValid_by_band() {
  TEST_ASSERT_TRUE(wifiBandChannelValid(WIFI_BAND_24_GHZ, 1));
  TEST_ASSERT_TRUE(wifiBandChannelValid(WIFI_BAND_24_GHZ, 14));
  TEST_ASSERT_FALSE(wifiBandChannelValid(WIFI_BAND_24_GHZ, 15));

  TEST_ASSERT_TRUE(wifiBandChannelValid(WIFI_BAND_5_GHZ, 36));
  TEST_ASSERT_TRUE(wifiBandChannelValid(WIFI_BAND_5_GHZ, 165));
  TEST_ASSERT_FALSE(wifiBandChannelValid(WIFI_BAND_5_GHZ, 35));
  TEST_ASSERT_FALSE(wifiBandChannelValid(WIFI_BAND_5_GHZ, 166));

  TEST_ASSERT_FALSE(wifiBandChannelValid(0, 36));
}

void test_wifiBssid_helpers() {
  const uint8_t zero_bssid[6] = {0, 0, 0, 0, 0, 0};
  const uint8_t ff_bssid[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  const uint8_t mixed_bssid[6] = {0x10, 0x20, 0x30, 0, 0, 0};

  TEST_ASSERT_TRUE(wifiBssidIsZero(zero_bssid));
  TEST_ASSERT_FALSE(wifiBssidIsZero(mixed_bssid));

  TEST_ASSERT_TRUE(wifiBssidIsBroadcast(ff_bssid));
  TEST_ASSERT_FALSE(wifiBssidIsBroadcast(mixed_bssid));
}

void test_wifiBssid_helpers_null_and_zero_length() {
  const uint8_t sample[1] = {0x00};
  TEST_ASSERT_TRUE(wifiBssidIsZero(nullptr));
  TEST_ASSERT_TRUE(wifiBssidIsZero(sample, 0));
  TEST_ASSERT_FALSE(wifiBssidIsBroadcast(nullptr));
  TEST_ASSERT_FALSE(wifiBssidIsBroadcast(sample, 0));
}

void test_wifiResult_validation_rules() {
  WiFiResult valid = makeValidResult();
  TEST_ASSERT_TRUE(wifiResultHasTerminatedSsid(valid));
  TEST_ASSERT_TRUE(wifiResultIsValidForDedupe(valid));

  WiFiResult valid_24g = makeValid24gResult();
  TEST_ASSERT_TRUE(wifiResultHasTerminatedSsid(valid_24g));
  TEST_ASSERT_TRUE(wifiResultIsValidForDedupe(valid_24g));

  WiFiResult bad_band = valid;
  bad_band.band = WIFI_BAND_24_GHZ;
  TEST_ASSERT_FALSE(wifiResultIsValidForDedupe(bad_band));

  WiFiResult zero_bssid = valid;
  memset(zero_bssid.bssid, 0, sizeof(zero_bssid.bssid));
  TEST_ASSERT_FALSE(wifiResultIsValidForDedupe(zero_bssid));

  WiFiResult broadcast_bssid = valid;
  memset(broadcast_bssid.bssid, 0xFF, sizeof(broadcast_bssid.bssid));
  TEST_ASSERT_FALSE(wifiResultIsValidForDedupe(broadcast_bssid));

  WiFiResult no_null_ssid = valid;
  memset(no_null_ssid.ssid, 'A', sizeof(no_null_ssid.ssid));
  TEST_ASSERT_FALSE(wifiResultHasTerminatedSsid(no_null_ssid));
  TEST_ASSERT_FALSE(wifiResultIsValidForDedupe(no_null_ssid));
}

void test_wifiDedupeHash_is_deterministic() {
  WiFiResult r = makeValidResult();
  WiFiDedupeHash a = {};
  WiFiDedupeHash b = {};

  wifiDedupeHashFromResult(r, a);
  wifiDedupeHashFromResult(r, b);

  TEST_ASSERT_TRUE(wifiDedupeHashEquals(a, b));
  b.bytes[WIFI_DEDUPE_HASH_SIZE - 1] ^= 0xA5;
  TEST_ASSERT_FALSE(wifiDedupeHashEquals(a, b));
}

void test_wifiDedupeHash_ignores_rssi_field() {
  WiFiResult a = makeValidResult();
  WiFiResult b = a;
  b.rssi = -5;

  WiFiDedupeHash ha = {};
  WiFiDedupeHash hb = {};
  wifiDedupeHashFromResult(a, ha);
  wifiDedupeHashFromResult(b, hb);
  TEST_ASSERT_TRUE(wifiDedupeHashEquals(ha, hb));

  WiFiDedupeHash slots[16] = {};
  WiFiDedupeHash fifo[8] = {};
  WiFiDedupeTable table = {};
  initSmallTable(&table, slots, 16, fifo, 8);

  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &ha));
  TEST_ASSERT_FALSE(wifiDedupeTableRemember(&table, &hb));
}

void test_wifiDedupeHash_changes_when_identity_fields_change() {
  const WiFiResult base = makeValidResult();
  WiFiDedupeHash h_base = {};
  wifiDedupeHashFromResult(base, h_base);

  WiFiResult changed_ssid = base;
  changed_ssid.ssid[0] = (changed_ssid.ssid[0] == 'Z') ? 'Y' : 'Z';
  WiFiDedupeHash h_ssid = {};
  wifiDedupeHashFromResult(changed_ssid, h_ssid);
  TEST_ASSERT_FALSE(wifiDedupeHashEquals(h_base, h_ssid));

  WiFiResult changed_bssid = base;
  changed_bssid.bssid[5] ^= 0x01;
  WiFiDedupeHash h_bssid = {};
  wifiDedupeHashFromResult(changed_bssid, h_bssid);
  TEST_ASSERT_FALSE(wifiDedupeHashEquals(h_base, h_bssid));

  WiFiResult changed_channel = base;
  changed_channel.channel = 44;
  changed_channel.band = WIFI_BAND_5_GHZ;
  WiFiDedupeHash h_channel = {};
  wifiDedupeHashFromResult(changed_channel, h_channel);
  TEST_ASSERT_FALSE(wifiDedupeHashEquals(h_base, h_channel));

  WiFiResult changed_caps = base;
  changed_caps.capabilities ^= CAP_EAP;
  WiFiDedupeHash h_caps = {};
  wifiDedupeHashFromResult(changed_caps, h_caps);
  TEST_ASSERT_FALSE(wifiDedupeHashEquals(h_base, h_caps));
}

void test_wifiDedupeTable_remember_contains_and_duplicate() {
  WiFiDedupeHash slots[8] = {};
  WiFiDedupeHash fifo[4] = {};
  WiFiDedupeTable table = {};
  initSmallTable(&table, slots, 8, fifo, 4);

  const WiFiDedupeHash h1 = makeHash(1);
  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h1));
  TEST_ASSERT_TRUE(wifiDedupeTableContains(&table, &h1));
  TEST_ASSERT_EQUAL_UINT16(1, table.count);

  TEST_ASSERT_FALSE(wifiDedupeTableRemember(&table, &h1));
  TEST_ASSERT_EQUAL_UINT16(1, table.count);
}

void test_wifiDedupeTable_ring_overwrite_when_full() {
  WiFiDedupeHash slots[4] = {};
  WiFiDedupeHash fifo[2] = {};
  WiFiDedupeTable table = {};
  initSmallTable(&table, slots, 4, fifo, 2);

  const WiFiDedupeHash h1 = makeHash(10);
  const WiFiDedupeHash h2 = makeHash(30);
  const WiFiDedupeHash h3 = makeHash(50);

  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h1));
  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h2));
  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h3));

  TEST_ASSERT_FALSE(wifiDedupeTableContains(&table, &h1));
  TEST_ASSERT_TRUE(wifiDedupeTableContains(&table, &h2));
  TEST_ASSERT_TRUE(wifiDedupeTableContains(&table, &h3));
  TEST_ASSERT_EQUAL_UINT16(2, table.count);
}

void test_wifiDedupeTable_corrupt_metadata_guard() {
  WiFiDedupeHash slots[4] = {};
  WiFiDedupeHash fifo[2] = {};
  WiFiDedupeTable table = {};
  initSmallTable(&table, slots, 4, fifo, 2);

  // Corrupt both count and FIFO indices beyond capacity.
  table.count = 99;
  table.fifo_head = 99;
  table.fifo_tail = 99;

  const WiFiDedupeHash h = makeHash(90);
  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h));
  TEST_ASSERT_TRUE(wifiDedupeTableContains(&table, &h));
}

void test_wifiDedupeTable_fifo_desync_skips_stale_entries() {
  WiFiDedupeHash slots[8] = {};
  WiFiDedupeHash fifo[3] = {};
  WiFiDedupeTable table = {};
  initSmallTable(&table, slots, 8, fifo, 3);

  const WiFiDedupeHash h1 = makeHash(1);
  const WiFiDedupeHash h2 = makeHash(9);
  const WiFiDedupeHash h3 = makeHash(17);
  const WiFiDedupeHash ghost = makeHash(90);
  const WiFiDedupeHash h4 = makeHash(25);

  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h1));
  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h2));
  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h3));
  TEST_ASSERT_EQUAL_UINT16(3, table.count);

  // Corrupt only the oldest FIFO entry. Insertion should skip the stale hash
  // and evict the next valid FIFO-backed entry instead of drifting count.
  table.fifo[table.fifo_head] = ghost;

  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h4));
  TEST_ASSERT_EQUAL_UINT16(3, table.count);
  TEST_ASSERT_TRUE(wifiDedupeTableContains(&table, &h1));
  TEST_ASSERT_FALSE(wifiDedupeTableContains(&table, &h2));
  TEST_ASSERT_TRUE(wifiDedupeTableContains(&table, &h3));
  TEST_ASSERT_TRUE(wifiDedupeTableContains(&table, &h4));
}

void test_wifiDedupeTable_fifo_total_desync_resets_table() {
  WiFiDedupeHash slots[8] = {};
  WiFiDedupeHash fifo[3] = {};
  WiFiDedupeTable table = {};
  initSmallTable(&table, slots, 8, fifo, 3);

  const WiFiDedupeHash h1 = makeHash(3);
  const WiFiDedupeHash h2 = makeHash(11);
  const WiFiDedupeHash h3 = makeHash(19);
  const WiFiDedupeHash ghost1 = makeHash(90);
  const WiFiDedupeHash ghost2 = makeHash(91);
  const WiFiDedupeHash ghost3 = makeHash(92);
  const WiFiDedupeHash h4 = makeHash(27);

  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h1));
  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h2));
  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h3));
  TEST_ASSERT_EQUAL_UINT16(3, table.count);

  // If the FIFO no longer points at any live slot, the corruption guard should
  // reset the table rather than underflow count or leave orphaned entries.
  table.fifo[0] = ghost1;
  table.fifo[1] = ghost2;
  table.fifo[2] = ghost3;

  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h4));
  TEST_ASSERT_EQUAL_UINT16(1, table.count);
  TEST_ASSERT_FALSE(wifiDedupeTableContains(&table, &h1));
  TEST_ASSERT_FALSE(wifiDedupeTableContains(&table, &h2));
  TEST_ASSERT_FALSE(wifiDedupeTableContains(&table, &h3));
  TEST_ASSERT_TRUE(wifiDedupeTableContains(&table, &h4));
}

void test_wifiDedupeTable_reset_clears_state() {
  WiFiDedupeHash slots[8] = {};
  WiFiDedupeHash fifo[3] = {};
  WiFiDedupeTable table = {};
  initSmallTable(&table, slots, 8, fifo, 3);

  const WiFiDedupeHash h1 = makeHash(7);
  const WiFiDedupeHash h2 = makeHash(11);
  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h1));
  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h2));
  TEST_ASSERT_EQUAL_UINT16(2, table.count);

  wifiDedupeTableReset(&table);
  TEST_ASSERT_EQUAL_UINT16(0, table.count);
  TEST_ASSERT_EQUAL_UINT16(0, table.fifo_head);
  TEST_ASSERT_EQUAL_UINT16(0, table.fifo_tail);
  TEST_ASSERT_FALSE(wifiDedupeTableContains(&table, &h1));
  TEST_ASSERT_FALSE(wifiDedupeTableContains(&table, &h2));
}

void test_wifiDedupeTable_invalid_inputs() {
  const WiFiDedupeHash h = makeHash(33);
  WiFiDedupeTable invalid = {};

  TEST_ASSERT_FALSE(wifiDedupeTableContains(nullptr, &h));
  TEST_ASSERT_FALSE(wifiDedupeTableContains(&invalid, &h));
  TEST_ASSERT_FALSE(wifiDedupeTableRemember(nullptr, &h));
  TEST_ASSERT_FALSE(wifiDedupeTableRemember(&invalid, &h));
}

void test_wifiDedupeTable_zero_hash_uses_reserved_empty_sentinel() {
  WiFiDedupeHash slots[8] = {};
  WiFiDedupeHash fifo[4] = {};
  WiFiDedupeTable table = {};
  initSmallTable(&table, slots, 8, fifo, 4);

  const WiFiDedupeHash zero = {};
  TEST_ASSERT_FALSE(wifiDedupeTableContains(&table, &zero));
  TEST_ASSERT_FALSE(wifiDedupeTableRemember(&table, &zero));
  TEST_ASSERT_EQUAL_UINT16(0, table.count);
}

// makeHash(seed) produces bytes {seed, seed+1, seed+2, ...}. On little-endian
// hosts the first two bytes are read as (seed + (seed+1)*256). For seed values
// 0, 8, 16, 24 this index is divisible by 8, so all four map to slot 0 in an
// 8-slot table (mask=7). This exercises linear probing and backward-shift
// deletion with a fully colliding probe chain.
void test_wifiDedupeTable_collision_probe_chain() {
  // All four hashes map to slot 0 in an 8-slot table: on little-endian hosts
  // hashToSlot reads the first two bytes as a uint16_t. For makeHash(N),
  // bytes[0..1] = {N, N+1}, so idx = N + (N+1)*256. For N = 0,8,16,24 this
  // value is always divisible by 8 (mask=7 → slot 0).
  //
  // Capacity=3 means the 4th insert triggers eviction of h1, exercising
  // backward-shift deletion across a fully colliding probe chain.
  WiFiDedupeHash slots[8] = {};
  WiFiDedupeHash fifo[3] = {};
  WiFiDedupeTable table = {};
  initSmallTable(&table, slots, 8, fifo, 3);

  const WiFiDedupeHash h1 = makeHash(0);   // natural slot 0
  const WiFiDedupeHash h2 = makeHash(8);   // natural slot 0 (collides)
  const WiFiDedupeHash h3 = makeHash(16);  // natural slot 0 (collides)

  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h1));
  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h2));
  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h3));
  TEST_ASSERT_EQUAL_UINT16(3, table.count);

  // h4 triggers eviction of h1 (count == capacity). Backward-shift deletion
  // must repair the probe chain so h2 and h3 remain findable.
  const WiFiDedupeHash h4 = makeHash(24);  // natural slot 0 (collides)
  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h4));
  TEST_ASSERT_FALSE(wifiDedupeTableContains(&table, &h1));
  TEST_ASSERT_TRUE(wifiDedupeTableContains(&table, &h2));
  TEST_ASSERT_TRUE(wifiDedupeTableContains(&table, &h3));
  TEST_ASSERT_TRUE(wifiDedupeTableContains(&table, &h4));
  TEST_ASSERT_EQUAL_UINT16(3, table.count);
}

void test_wifiDedupeTable_many_insertions_keep_only_most_recent_entries() {
  WiFiDedupeHash slots[8] = {};
  WiFiDedupeHash fifo[3] = {};
  WiFiDedupeTable table = {};
  initSmallTable(&table, slots, 8, fifo, 3);

  const WiFiDedupeHash h1 = makeHash(1);
  const WiFiDedupeHash h2 = makeHash(2);
  const WiFiDedupeHash h3 = makeHash(3);
  const WiFiDedupeHash h4 = makeHash(4);
  const WiFiDedupeHash h5 = makeHash(5);

  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h1));
  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h2));
  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h3));
  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h4));
  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h5));

  TEST_ASSERT_EQUAL_UINT16(3, table.count);
  TEST_ASSERT_FALSE(wifiDedupeTableContains(&table, &h1));
  TEST_ASSERT_FALSE(wifiDedupeTableContains(&table, &h2));
  TEST_ASSERT_TRUE(wifiDedupeTableContains(&table, &h3));
  TEST_ASSERT_TRUE(wifiDedupeTableContains(&table, &h4));
  TEST_ASSERT_TRUE(wifiDedupeTableContains(&table, &h5));
}

void test_wifiDedupeTable_evicted_entry_can_be_reinserted() {
  WiFiDedupeHash slots[8] = {};
  WiFiDedupeHash fifo[3] = {};
  WiFiDedupeTable table = {};
  initSmallTable(&table, slots, 8, fifo, 3);

  const WiFiDedupeHash h1 = makeHash(10);
  const WiFiDedupeHash h2 = makeHash(11);
  const WiFiDedupeHash h3 = makeHash(12);
  const WiFiDedupeHash h4 = makeHash(13);

  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h1));
  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h2));
  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h3));
  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h4));
  TEST_ASSERT_FALSE(wifiDedupeTableContains(&table, &h1));

  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h1));
  TEST_ASSERT_EQUAL_UINT16(3, table.count);
  TEST_ASSERT_FALSE(wifiDedupeTableContains(&table, &h2));
  TEST_ASSERT_TRUE(wifiDedupeTableContains(&table, &h3));
  TEST_ASSERT_TRUE(wifiDedupeTableContains(&table, &h4));
  TEST_ASSERT_TRUE(wifiDedupeTableContains(&table, &h1));
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  UNITY_BEGIN();
  RUN_TEST(test_wifiBandFromChannel_boundaries);
  RUN_TEST(test_wifiBandChannelValid_by_band);
  RUN_TEST(test_wifiBssid_helpers);
  RUN_TEST(test_wifiBssid_helpers_null_and_zero_length);
  RUN_TEST(test_wifiResult_validation_rules);
  RUN_TEST(test_wifiDedupeHash_is_deterministic);
  RUN_TEST(test_wifiDedupeHash_ignores_rssi_field);
  RUN_TEST(test_wifiDedupeHash_changes_when_identity_fields_change);
  RUN_TEST(test_wifiDedupeTable_remember_contains_and_duplicate);
  RUN_TEST(test_wifiDedupeTable_ring_overwrite_when_full);
  RUN_TEST(test_wifiDedupeTable_corrupt_metadata_guard);
  RUN_TEST(test_wifiDedupeTable_fifo_desync_skips_stale_entries);
  RUN_TEST(test_wifiDedupeTable_fifo_total_desync_resets_table);
  RUN_TEST(test_wifiDedupeTable_reset_clears_state);
  RUN_TEST(test_wifiDedupeTable_invalid_inputs);
  RUN_TEST(test_wifiDedupeTable_zero_hash_uses_reserved_empty_sentinel);
  RUN_TEST(test_wifiDedupeTable_collision_probe_chain);
  RUN_TEST(test_wifiDedupeTable_many_insertions_keep_only_most_recent_entries);
  RUN_TEST(test_wifiDedupeTable_evicted_entry_can_be_reinserted);
  return UNITY_END();
}
