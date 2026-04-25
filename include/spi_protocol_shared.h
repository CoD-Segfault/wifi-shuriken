#pragma once

#include <stddef.h>
#include <stdint.h>

// Shared SPI protocol between Pico master and ESP32 slave.
// Keep this file as the single source of truth for on-wire framing.
static constexpr uint8_t PROTO_VERSION = 1;
static constexpr uint8_t SCANNER_TYPE_ESP32_C5 = 1;
static constexpr uint8_t CAP_BAND_24GHZ = 0x01;
static constexpr uint8_t CAP_BAND_5GHZ = 0x02;
static constexpr uint8_t CAP_BAND_6GHZ = 0x04;
static constexpr uint8_t CAP_OTA_UPDATE = 0x80;
static constexpr uint8_t PROTO_MAX_RESULTS = 127;
static constexpr uint8_t OTA_DATA_BYTES_PER_FRAME = 48;

// WiFiResult.capabilities bit flags (AuthMode/ciphers/AKM).
#define CAP_ESS              (1u << 0)
#define CAP_WEP              (1u << 1)
#define CAP_WPA              (1u << 2)
#define CAP_WPA2             (1u << 3)
#define CAP_WPA3             (1u << 4)
#define CAP_PSK              (1u << 5)
#define CAP_EAP              (1u << 6)
#define CAP_TKIP             (1u << 7)
#define CAP_CCMP             (1u << 8)

#ifndef LOG_DEDUPE_HITS
#define LOG_DEDUPE_HITS 0
#endif

#ifndef LOG_DEDUPE_HITS_MAX_PER_SCAN
#define LOG_DEDUPE_HITS_MAX_PER_SCAN 8
#endif

enum SpiCommand : uint8_t {
  CMD_NOP          = 0x00,
  CMD_ID           = 0x01,
  CMD_SCAN         = 0x02,
  CMD_RESULT_COUNT = 0x03,
  CMD_RESULT_GET   = 0x04,
  CMD_DEDUPE_RESET = 0x05,
  CMD_OTA_BEGIN    = 0x10,
  CMD_OTA_DATA     = 0x11,
  CMD_OTA_END      = 0x12,
  CMD_OTA_ABORT    = 0x13,
};

enum ResultType : uint8_t {
  RESULT_WIFI = 0x00,
  RESULT_BUSY = 0xFD,
  RESULT_END  = 0xFF,
};

// On-wire scan/status codes returned by CMD_SCAN and CMD_RESULT_COUNT.
enum ScannerStatus : int8_t {
  SCANNER_STATUS_OK = 0,
  SCANNER_STATUS_BUSY = -1,
  SCANNER_STATUS_INVALID = -2,
  SCANNER_STATUS_START_FAILED = -3,
};

enum OtaStatus : int8_t {
  OTA_STATUS_OK = 0,
  OTA_STATUS_BUSY = -1,
  OTA_STATUS_INVALID = -2,
  OTA_STATUS_NOT_ACTIVE = -3,
  OTA_STATUS_SEQUENCE_MISMATCH = -4,
  OTA_STATUS_CRC_MISMATCH = -5,
  OTA_STATUS_WRITE_FAILED = -6,
  OTA_STATUS_SIZE_MISMATCH = -7,
  OTA_STATUS_HASH_MISMATCH = -8,
  OTA_STATUS_BEGIN_FAILED = -9,
  OTA_STATUS_END_FAILED = -10,
};

static constexpr size_t SPI_FRAME_SIZE = 64;
static constexpr size_t SPI_FRAME_TAG_INDEX = SPI_FRAME_SIZE - 1;

struct __attribute__((packed)) WiFiResult {
  char ssid[33];
  uint8_t bssid[6];
  int8_t rssi;
  uint8_t channel;
  uint8_t band;
  uint16_t capabilities;
};

struct __attribute__((packed)) WiFiResultPacket {
  uint8_t result_type;
  WiFiResult result;
  uint8_t _pad[3];
};

struct __attribute__((packed)) ScanCommand {
  uint8_t cmd;
  uint8_t band;
  uint8_t channel;
  uint8_t reserved;
};

struct __attribute__((packed)) DeviceIdReply {
  uint8_t proto_version;
  uint8_t scanner_type;
  uint8_t capabilities;
  uint8_t max_results;
  uint8_t reserved[4];
};

struct __attribute__((packed)) OtaBeginCommand {
  uint8_t cmd;
  uint8_t packet_data_bytes;
  uint16_t reserved0;
  uint32_t image_size;
  uint64_t image_hash;
  uint8_t reserved1[48];
};

struct __attribute__((packed)) OtaDataCommand {
  uint8_t cmd;
  uint8_t length;
  uint16_t sequence;
  uint32_t offset;
  uint32_t crc32;
  uint8_t data[OTA_DATA_BYTES_PER_FRAME];
  uint8_t reserved[4];
};

struct __attribute__((packed)) OtaEndCommand {
  uint8_t cmd;
  uint8_t reserved0;
  uint16_t reserved1;
  uint32_t image_size;
  uint64_t image_hash;
  uint8_t reserved2[48];
};

struct __attribute__((packed)) OtaAck {
  int8_t status;
  uint8_t active;
  uint16_t next_sequence;
  uint32_t offset;
  uint32_t detail;
  uint64_t image_hash;
};

static constexpr uint32_t OTA_CRC32_INIT = 0xffffffffUL;
static constexpr uint64_t OTA_HASH64_INIT = 14695981039346656037ULL;
static constexpr uint64_t OTA_HASH64_PRIME = 1099511628211ULL;

static inline uint32_t otaCrc32Update(uint32_t crc, const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 1U) {
        crc = (crc >> 1) ^ 0xedb88320UL;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

static inline uint32_t otaCrc32(const uint8_t* data, size_t len) {
  return ~otaCrc32Update(OTA_CRC32_INIT, data, len);
}

static inline uint64_t otaHash64Update(uint64_t hash, const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    hash ^= data[i];
    hash *= OTA_HASH64_PRIME;
  }
  return hash;
}

static_assert(sizeof(WiFiResultPacket) == 48, "WiFiResultPacket must be 48 bytes");
static_assert(sizeof(DeviceIdReply) == 8, "DeviceIdReply must be 8 bytes");
static_assert(sizeof(OtaBeginCommand) == SPI_FRAME_SIZE, "OtaBeginCommand must be one SPI frame");
static_assert(sizeof(OtaDataCommand) == SPI_FRAME_SIZE, "OtaDataCommand must be one SPI frame");
static_assert(sizeof(OtaEndCommand) == SPI_FRAME_SIZE, "OtaEndCommand must be one SPI frame");
static_assert(sizeof(OtaAck) == 20, "OtaAck must be 20 bytes");
