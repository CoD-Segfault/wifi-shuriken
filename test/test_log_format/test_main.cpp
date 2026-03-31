#include <string.h>

#include <unity.h>

#include "log_format.h"
#include "spi_protocol_shared.h"

void setUp() {}
void tearDown() {}

void test_formatBssid_outputs_uppercase_mac() {
  const uint8_t bssid[6] = {0x00, 0x12, 0xAB, 0xCD, 0xEF, 0x34};
  char out[18] = {};
  pico_logging::formatBssid(bssid, out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("00:12:AB:CD:EF:34", out);
}

void test_csvEscape_quotes_and_escapes_quotes() {
  char out[64] = {};
  pico_logging::csvEscape("My\"SSID", out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("\"My\"\"SSID\"", out);
}

void test_csvEscape_truncates_safely() {
  char out[8] = {};
  pico_logging::csvEscape("abcdefghijk", out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("\"abcde\"", out);
}

void test_buildAndroidCapabilitiesString_open_and_ess() {
  char out[96] = {};
  pico_logging::buildAndroidCapabilitiesString(0, out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("[OPEN]", out);

  pico_logging::buildAndroidCapabilitiesString(CAP_ESS, out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("[OPEN][ESS]", out);
}

void test_buildAndroidCapabilitiesString_wep_and_ess() {
  char out[96] = {};
  pico_logging::buildAndroidCapabilitiesString((uint16_t)(CAP_WEP | CAP_ESS), out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("[WEP][ESS]", out);
}

void test_buildAndroidCapabilitiesString_wpa2_psk_ccmp() {
  char out[96] = {};
  const uint16_t caps = (uint16_t)(CAP_WPA2 | CAP_PSK | CAP_CCMP | CAP_ESS);
  pico_logging::buildAndroidCapabilitiesString(caps, out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("[WPA2-PSK-CCMP][ESS]", out);
}

void test_buildAndroidCapabilitiesString_wpa3_psk_ccmp_maps_to_rsn() {
  char out[96] = {};
  const uint16_t caps = (uint16_t)(CAP_WPA3 | CAP_PSK | CAP_CCMP | CAP_ESS);
  pico_logging::buildAndroidCapabilitiesString(caps, out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("[RSN-PSK-CCMP][ESS]", out);
}

void test_buildAndroidCapabilitiesString_multi_protocol_eap_combo() {
  char out[96] = {};
  const uint16_t caps = (uint16_t)(CAP_WPA | CAP_WPA2 | CAP_EAP | CAP_CCMP | CAP_TKIP);
  pico_logging::buildAndroidCapabilitiesString(caps, out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("[WPA-EAP-CCMP+TKIP][WPA2-EAP-CCMP+TKIP]", out);
}

void test_buildCsvWifiRow_full_line() {
  const uint8_t bssid[6] = {0x00, 0x12, 0xAB, 0xCD, 0xEF, 0x34};
  char out[384] = {};
  const bool ok = pico_logging::buildCsvWifiRow(
      bssid,
      "My\"SSID",
      (uint16_t)(CAP_WPA2 | CAP_PSK | CAP_CCMP | CAP_ESS),
      "2026-02-28 12:34:56",
      36,
      -42,
      "37.1234567",
      "-122.7654321",
      "12.34",
      "4.56",
      out,
      sizeof(out));

  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_EQUAL_STRING(
      "00:12:AB:CD:EF:34,\"My\"\"SSID\",[WPA2-PSK-CCMP][ESS],2026-02-28 12:34:56,36,-42,37.1234567,-122.7654321,12.34,4.56,WIFI\n",
      out);
}

void test_buildCsvWifiRow_blank_optional_fields() {
  const uint8_t bssid[6] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05};
  char out[384] = {};
  const bool ok = pico_logging::buildCsvWifiRow(
      bssid, "NoFix", 0, "", 1, -90, "", "", "", "", out, sizeof(out));

  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_EQUAL_STRING(
      "00:01:02:03:04:05,\"NoFix\",[OPEN],,1,-90,,,,,WIFI\n",
      out);
}

void test_buildCsvWifiRow_null_inputs_are_safely_defaulted() {
  char out[384] = {};
  const bool ok = pico_logging::buildCsvWifiRow(
      nullptr, nullptr, 0, nullptr, 6, -1, nullptr, nullptr, nullptr, nullptr, out, sizeof(out));

  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_EQUAL_STRING(
      "00:00:00:00:00:00,\"\",[OPEN],,6,-1,,,,,WIFI\n",
      out);
}

void test_buildCsvWifiRow_reports_overflow() {
  const uint8_t bssid[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
  char out[24] = {};
  const bool ok = pico_logging::buildCsvWifiRow(
      bssid, "SSID", CAP_ESS, "2026-02-28 12:34:56", 149, -20, "1.0", "2.0", "3.0", "4.0", out, sizeof(out));

  TEST_ASSERT_FALSE(ok);
  TEST_ASSERT_EQUAL_CHAR('\0', out[sizeof(out) - 1]);
}

void test_buildCsvWifiRow_rejects_missing_output_buffer() {
  const uint8_t bssid[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
  char out[8] = {};

  TEST_ASSERT_FALSE(pico_logging::buildCsvWifiRow(
      bssid, "SSID", CAP_ESS, "2026-02-28 12:34:56", 149, -20, "1.0", "2.0", "3.0", "4.0", nullptr, sizeof(out)));
  TEST_ASSERT_FALSE(pico_logging::buildCsvWifiRow(
      bssid, "SSID", CAP_ESS, "2026-02-28 12:34:56", 149, -20, "1.0", "2.0", "3.0", "4.0", out, 0));
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  UNITY_BEGIN();
  RUN_TEST(test_formatBssid_outputs_uppercase_mac);
  RUN_TEST(test_csvEscape_quotes_and_escapes_quotes);
  RUN_TEST(test_csvEscape_truncates_safely);
  RUN_TEST(test_buildAndroidCapabilitiesString_open_and_ess);
  RUN_TEST(test_buildAndroidCapabilitiesString_wep_and_ess);
  RUN_TEST(test_buildAndroidCapabilitiesString_wpa2_psk_ccmp);
  RUN_TEST(test_buildAndroidCapabilitiesString_wpa3_psk_ccmp_maps_to_rsn);
  RUN_TEST(test_buildAndroidCapabilitiesString_multi_protocol_eap_combo);
  RUN_TEST(test_buildCsvWifiRow_full_line);
  RUN_TEST(test_buildCsvWifiRow_blank_optional_fields);
  RUN_TEST(test_buildCsvWifiRow_null_inputs_are_safely_defaulted);
  RUN_TEST(test_buildCsvWifiRow_reports_overflow);
  RUN_TEST(test_buildCsvWifiRow_rejects_missing_output_buffer);
  return UNITY_END();
}
