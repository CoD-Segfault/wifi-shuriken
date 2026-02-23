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
static constexpr uint8_t PROTO_MAX_RESULTS = 127;

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
};

enum ResultType : uint8_t {
  RESULT_WIFI = 0x00,
  RESULT_BUSY = 0xFD,
  RESULT_END  = 0xFF,
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

static_assert(sizeof(WiFiResultPacket) == 48, "WiFiResultPacket must be 48 bytes");
static_assert(sizeof(DeviceIdReply) == 8, "DeviceIdReply must be 8 bytes");
