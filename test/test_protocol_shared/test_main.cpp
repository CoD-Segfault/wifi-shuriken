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
  TEST_ASSERT_EQUAL_UINT8(0x10, CMD_OTA_BEGIN);
  TEST_ASSERT_EQUAL_UINT8(0x11, CMD_OTA_DATA);
  TEST_ASSERT_EQUAL_UINT8(0x12, CMD_OTA_END);
  TEST_ASSERT_EQUAL_UINT8(0x13, CMD_OTA_ABORT);
}

void test_protocol_version_and_capability_values_are_stable() {
  TEST_ASSERT_EQUAL_UINT8(1, PROTO_VERSION);
  TEST_ASSERT_EQUAL_UINT8(1, SCANNER_TYPE_ESP32_C5);
  TEST_ASSERT_EQUAL_UINT8(0x01, CAP_BAND_24GHZ);
  TEST_ASSERT_EQUAL_UINT8(0x02, CAP_BAND_5GHZ);
  TEST_ASSERT_EQUAL_UINT8(0x04, CAP_BAND_6GHZ);
  TEST_ASSERT_EQUAL_UINT8(0x80, CAP_OTA_UPDATE);
  TEST_ASSERT_EQUAL_UINT8(127, PROTO_MAX_RESULTS);
  TEST_ASSERT_EQUAL_UINT8(48, OTA_DATA_BYTES_PER_FRAME);
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

void test_protocol_ota_status_values_are_stable() {
  TEST_ASSERT_EQUAL_INT8(0, OTA_STATUS_OK);
  TEST_ASSERT_EQUAL_INT8(-1, OTA_STATUS_BUSY);
  TEST_ASSERT_EQUAL_INT8(-2, OTA_STATUS_INVALID);
  TEST_ASSERT_EQUAL_INT8(-3, OTA_STATUS_NOT_ACTIVE);
  TEST_ASSERT_EQUAL_INT8(-4, OTA_STATUS_SEQUENCE_MISMATCH);
  TEST_ASSERT_EQUAL_INT8(-5, OTA_STATUS_CRC_MISMATCH);
  TEST_ASSERT_EQUAL_INT8(-6, OTA_STATUS_WRITE_FAILED);
  TEST_ASSERT_EQUAL_INT8(-7, OTA_STATUS_SIZE_MISMATCH);
  TEST_ASSERT_EQUAL_INT8(-8, OTA_STATUS_HASH_MISMATCH);
  TEST_ASSERT_EQUAL_INT8(-9, OTA_STATUS_BEGIN_FAILED);
  TEST_ASSERT_EQUAL_INT8(-10, OTA_STATUS_END_FAILED);
}

void test_protocol_frame_contract() {
  TEST_ASSERT_EQUAL_UINT32(64, SPI_FRAME_SIZE);
  TEST_ASSERT_EQUAL_UINT32(SPI_FRAME_SIZE - 1, SPI_FRAME_TAG_INDEX);
}

void test_protocol_struct_layout_and_limits() {
  TEST_ASSERT_EQUAL_UINT32(44, sizeof(WiFiResult));
  TEST_ASSERT_EQUAL_UINT32(48, sizeof(WiFiResultPacket));
  TEST_ASSERT_EQUAL_UINT32(4, sizeof(ScanCommand));
  TEST_ASSERT_EQUAL_UINT32(8, sizeof(DeviceIdReply));
  TEST_ASSERT_EQUAL_UINT32(64, sizeof(OtaBeginCommand));
  TEST_ASSERT_EQUAL_UINT32(64, sizeof(OtaDataCommand));
  TEST_ASSERT_EQUAL_UINT32(64, sizeof(OtaEndCommand));
  TEST_ASSERT_EQUAL_UINT32(20, sizeof(OtaAck));
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

  TEST_ASSERT_EQUAL_UINT32(0, offsetof(OtaBeginCommand, cmd));
  TEST_ASSERT_EQUAL_UINT32(1, offsetof(OtaBeginCommand, packet_data_bytes));
  TEST_ASSERT_EQUAL_UINT32(4, offsetof(OtaBeginCommand, image_size));
  TEST_ASSERT_EQUAL_UINT32(8, offsetof(OtaBeginCommand, image_hash));

  TEST_ASSERT_EQUAL_UINT32(0, offsetof(OtaDataCommand, cmd));
  TEST_ASSERT_EQUAL_UINT32(1, offsetof(OtaDataCommand, length));
  TEST_ASSERT_EQUAL_UINT32(2, offsetof(OtaDataCommand, sequence));
  TEST_ASSERT_EQUAL_UINT32(4, offsetof(OtaDataCommand, offset));
  TEST_ASSERT_EQUAL_UINT32(8, offsetof(OtaDataCommand, crc32));
  TEST_ASSERT_EQUAL_UINT32(12, offsetof(OtaDataCommand, data));
  TEST_ASSERT_EQUAL_UINT32(60, offsetof(OtaDataCommand, reserved));

  TEST_ASSERT_EQUAL_UINT32(0, offsetof(OtaEndCommand, cmd));
  TEST_ASSERT_EQUAL_UINT32(4, offsetof(OtaEndCommand, image_size));
  TEST_ASSERT_EQUAL_UINT32(8, offsetof(OtaEndCommand, image_hash));

  TEST_ASSERT_EQUAL_UINT32(0, offsetof(OtaAck, status));
  TEST_ASSERT_EQUAL_UINT32(1, offsetof(OtaAck, active));
  TEST_ASSERT_EQUAL_UINT32(2, offsetof(OtaAck, next_sequence));
  TEST_ASSERT_EQUAL_UINT32(4, offsetof(OtaAck, offset));
  TEST_ASSERT_EQUAL_UINT32(8, offsetof(OtaAck, detail));
  TEST_ASSERT_EQUAL_UINT32(12, offsetof(OtaAck, image_hash));
}

void test_protocol_ota_integrity_helpers_are_stable() {
  const uint8_t data[] = {'1','2','3','4','5','6','7','8','9'};
  TEST_ASSERT_EQUAL_UINT32(0xcbf43926UL, otaCrc32(data, sizeof(data)));
  TEST_ASSERT_EQUAL_UINT64(0x06d5573923c6cdfcULL,
                           otaHash64Update(OTA_HASH64_INIT, data, sizeof(data)));
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  UNITY_BEGIN();
  RUN_TEST(test_protocol_command_values_are_stable);
  RUN_TEST(test_protocol_version_and_capability_values_are_stable);
  RUN_TEST(test_protocol_result_type_values_are_stable);
  RUN_TEST(test_protocol_status_values_are_stable);
  RUN_TEST(test_protocol_ota_status_values_are_stable);
  RUN_TEST(test_protocol_frame_contract);
  RUN_TEST(test_protocol_struct_layout_and_limits);
  RUN_TEST(test_protocol_struct_offsets_are_wire_stable);
  RUN_TEST(test_protocol_ota_integrity_helpers_are_stable);
  return UNITY_END();
}
