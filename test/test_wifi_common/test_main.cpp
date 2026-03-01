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

static WiFiDedupeHash makeHash(uint8_t seed) {
  WiFiDedupeHash hash = {};
  for (size_t i = 0; i < WIFI_DEDUPE_HASH_SIZE; i++) {
    hash.bytes[i] = (uint8_t)(seed + i);
  }
  return hash;
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

  WiFiResult bad_band = valid;
  bad_band.band = WIFI_BAND_24_GHZ;
  TEST_ASSERT_FALSE(wifiResultIsValidForDedupe(bad_band));

  WiFiResult zero_bssid = valid;
  memset(zero_bssid.bssid, 0, sizeof(zero_bssid.bssid));
  TEST_ASSERT_FALSE(wifiResultIsValidForDedupe(zero_bssid));

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

  WiFiDedupeHash storage[8] = {};
  WiFiDedupeTable table = {};
  wifiDedupeTableInit(&table, storage, 8);

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
  WiFiDedupeHash storage[4] = {};
  WiFiDedupeTable table = {};
  wifiDedupeTableInit(&table, storage, 4);

  const WiFiDedupeHash h1 = makeHash(1);
  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h1));
  TEST_ASSERT_TRUE(wifiDedupeTableContains(&table, &h1));
  TEST_ASSERT_EQUAL_UINT16(1, table.count);

  TEST_ASSERT_FALSE(wifiDedupeTableRemember(&table, &h1));
  TEST_ASSERT_EQUAL_UINT16(1, table.count);
}

void test_wifiDedupeTable_ring_overwrite_when_full() {
  WiFiDedupeHash storage[2] = {};
  WiFiDedupeTable table = {};
  wifiDedupeTableInit(&table, storage, 2);

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
  WiFiDedupeHash storage[2] = {};
  WiFiDedupeTable table = {};
  wifiDedupeTableInit(&table, storage, 2);

  table.count = 99;
  table.write_index = 99;

  const WiFiDedupeHash h = makeHash(90);
  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h));
  TEST_ASSERT_TRUE(wifiDedupeTableContains(&table, &h));
}

void test_wifiDedupeTable_reset_clears_state() {
  WiFiDedupeHash storage[3] = {};
  WiFiDedupeTable table = {};
  wifiDedupeTableInit(&table, storage, 3);

  const WiFiDedupeHash h1 = makeHash(7);
  const WiFiDedupeHash h2 = makeHash(11);
  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h1));
  TEST_ASSERT_TRUE(wifiDedupeTableRemember(&table, &h2));
  TEST_ASSERT_EQUAL_UINT16(2, table.count);

  wifiDedupeTableReset(&table);
  TEST_ASSERT_EQUAL_UINT16(0, table.count);
  TEST_ASSERT_EQUAL_UINT16(0, table.write_index);
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
  RUN_TEST(test_wifiDedupeTable_reset_clears_state);
  RUN_TEST(test_wifiDedupeTable_invalid_inputs);
  return UNITY_END();
}
