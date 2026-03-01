#include <limits.h>
#include <stddef.h>

#include <unity.h>

#include "spi_protocol_shared.h"

void setUp() {}
void tearDown() {}

void test_protocol_command_values_are_stable() {
  TEST_ASSERT_EQUAL_UINT8(0x00, CMD_NOP);
  TEST_ASSERT_EQUAL_UINT8(0x01, CMD_ID);
  TEST_ASSERT_EQUAL_UINT8(0x02, CMD_SCAN);
  TEST_ASSERT_EQUAL_UINT8(0x03, CMD_RESULT_COUNT);
  TEST_ASSERT_EQUAL_UINT8(0x04, CMD_RESULT_GET);
  TEST_ASSERT_EQUAL_UINT8(0x05, CMD_DEDUPE_RESET);
}

void test_protocol_result_type_values_are_stable() {
  TEST_ASSERT_EQUAL_UINT8(0x00, RESULT_WIFI);
  TEST_ASSERT_EQUAL_UINT8(0xFD, RESULT_BUSY);
  TEST_ASSERT_EQUAL_UINT8(0xFF, RESULT_END);
}

void test_protocol_status_values_are_stable() {
  TEST_ASSERT_EQUAL_INT8(0, SCANNER_STATUS_OK);
  TEST_ASSERT_EQUAL_INT8(-1, SCANNER_STATUS_BUSY);
  TEST_ASSERT_EQUAL_INT8(-2, SCANNER_STATUS_INVALID);
  TEST_ASSERT_EQUAL_INT8(-3, SCANNER_STATUS_START_FAILED);
}

void test_protocol_frame_contract() {
  TEST_ASSERT_EQUAL_UINT32(64, SPI_FRAME_SIZE);
  TEST_ASSERT_EQUAL_UINT32(SPI_FRAME_SIZE - 1, SPI_FRAME_TAG_INDEX);
}

void test_protocol_struct_layout_and_limits() {
  TEST_ASSERT_EQUAL_UINT32(48, sizeof(WiFiResultPacket));
  TEST_ASSERT_EQUAL_UINT32(8, sizeof(DeviceIdReply));
  TEST_ASSERT_TRUE(PROTO_MAX_RESULTS <= INT8_MAX);
}

void test_protocol_struct_offsets_are_wire_stable() {
  TEST_ASSERT_EQUAL_UINT32(0, offsetof(WiFiResult, ssid));
  TEST_ASSERT_EQUAL_UINT32(33, offsetof(WiFiResult, bssid));
  TEST_ASSERT_EQUAL_UINT32(39, offsetof(WiFiResult, rssi));
  TEST_ASSERT_EQUAL_UINT32(40, offsetof(WiFiResult, channel));
  TEST_ASSERT_EQUAL_UINT32(41, offsetof(WiFiResult, band));
  TEST_ASSERT_EQUAL_UINT32(42, offsetof(WiFiResult, capabilities));

  TEST_ASSERT_EQUAL_UINT32(0, offsetof(WiFiResultPacket, result_type));
  TEST_ASSERT_EQUAL_UINT32(1, offsetof(WiFiResultPacket, result));
  TEST_ASSERT_EQUAL_UINT32(45, offsetof(WiFiResultPacket, _pad));

  TEST_ASSERT_EQUAL_UINT32(0, offsetof(ScanCommand, cmd));
  TEST_ASSERT_EQUAL_UINT32(1, offsetof(ScanCommand, band));
  TEST_ASSERT_EQUAL_UINT32(2, offsetof(ScanCommand, channel));
  TEST_ASSERT_EQUAL_UINT32(3, offsetof(ScanCommand, reserved));

  TEST_ASSERT_EQUAL_UINT32(0, offsetof(DeviceIdReply, proto_version));
  TEST_ASSERT_EQUAL_UINT32(1, offsetof(DeviceIdReply, scanner_type));
  TEST_ASSERT_EQUAL_UINT32(2, offsetof(DeviceIdReply, capabilities));
  TEST_ASSERT_EQUAL_UINT32(3, offsetof(DeviceIdReply, max_results));
  TEST_ASSERT_EQUAL_UINT32(4, offsetof(DeviceIdReply, reserved));
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  UNITY_BEGIN();
  RUN_TEST(test_protocol_command_values_are_stable);
  RUN_TEST(test_protocol_result_type_values_are_stable);
  RUN_TEST(test_protocol_status_values_are_stable);
  RUN_TEST(test_protocol_frame_contract);
  RUN_TEST(test_protocol_struct_layout_and_limits);
  RUN_TEST(test_protocol_struct_offsets_are_wire_stable);
  return UNITY_END();
}
