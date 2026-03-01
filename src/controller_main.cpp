#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <TinyGPSPlus.h>
#include <SdFat.h>
#if defined(USE_TINYUSB)
#include <Adafruit_TinyUSB.h>
#endif
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "hardware/watchdog.h"
#include "hardware/gpio.h"
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>
#include <stdarg.h>
#include "pico_logging.h"
#include "spi_protocol_shared.h"
#include "wifi_dedupe.h"
#include "wifi_result_utils.h"
#include "Configuration.h"

static constexpr uint32_t GNSS_TARGET_BAUD = 115200;

SdFat sd;
FsFile logFile;
Adafruit_NeoPixel pixels(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);
TinyGPSPlus gps;
#if defined(USE_TINYUSB)
static Adafruit_USBD_CDC nmea_passthrough_cdc;
#endif
using QueuedScanResult = pico_logging::QueuedScanResult;

// Cross-core queues:
// - core1 (SPI/scanner) produces scan rows + diagnostic lines.
// - core0 (main loop) consumes and performs serial output + SD writes.
static queue_t scan_result_queue;
struct QueuedSerialMsg {
  char text[192];
};
static queue_t serial_msg_queue;
static uint32_t scan_queue_drops = 0;
static uint32_t serial_msg_drops = 0;
static uint32_t dedupe_drops = 0;
static constexpr uint16_t GPS_FIELD_MAX_AGE_MS = 3000;

static constexpr uint16_t MASTER_DEDUPE_CAPACITY = WIFI_DEDUPE_TABLE_CAPACITY;
static WiFiDedupeHash master_dedupe_storage[MASTER_DEDUPE_CAPACITY];
static WiFiDedupeTable master_dedupe_table = {};
static constexpr uint8_t SD_INIT_MAX_ATTEMPTS = 5;
static constexpr uint16_t SD_INIT_RETRY_DELAY_MS = 500;
static constexpr uint16_t SD_RETRY_BACKGROUND_MS = 30000;
static constexpr uint32_t SD_SPI_CLOCKS_HZ[] = {SD_SPI_CLOCK, 4000000, 1000000, 400000};
static constexpr uint16_t GNSS_BOOT_TIMESTAMP_WAIT_MS = 6000;
static constexpr uint8_t GNSS_BOOT_TIMESTAMP_READS = 5;
static constexpr uint8_t RESET_BUTTON_DEBOUNCE_MS = 50;
static constexpr uint16_t CSV_LOG_TIME_WAIT_RETRY_MS = 1000;
static constexpr uint16_t CSV_LOG_FLUSH_INTERVAL_MS = 5000;
static constexpr uint16_t GNSS_MIN_VALID_YEAR = 2026;

static pico_logging::State logging_state = {};
static const pico_logging::Config logging_config = {
  SD_PIN_CS,
  SD_SPI_CLOCKS_HZ,
  sizeof(SD_SPI_CLOCKS_HZ) / sizeof(SD_SPI_CLOCKS_HZ[0]),
  SD_INIT_RETRY_DELAY_MS,
  SD_RETRY_BACKGROUND_MS,
  GNSS_BOOT_TIMESTAMP_WAIT_MS,
  GNSS_BOOT_TIMESTAMP_READS,
  GPS_FIELD_MAX_AGE_MS,
  CSV_LOG_TIME_WAIT_RETRY_MS,
  CSV_LOG_FLUSH_INTERVAL_MS,
  GNSS_MIN_VALID_YEAR
};
static pico_logging::Logger logging(sd, logFile, gps, GNSS_UART, Serial, logging_config, logging_state);

static void serialQueueTry(const char* text) {
  QueuedSerialMsg msg = {};
  strncpy(msg.text, text, sizeof(msg.text) - 1);
  if (!queue_try_add(&serial_msg_queue, &msg)) {
    serial_msg_drops++;
  }
}

static void serialPrintlnTry(const char* s) {
#if !CORE1_SERIAL_LOG
  (void)s;
  return;
#else
  char buf[192];
  snprintf(buf, sizeof(buf), "%s\n", s);
  serialQueueTry(buf);
#endif
}

static void serialPrintfTry(const char* fmt, ...) {
#if !CORE1_SERIAL_LOG
  (void)fmt;
  return;
#else
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  serialQueueTry(buf);
#endif
}

static inline void nmeaPassthroughWriteChar(char c) {
#if !NMEA_PASSTHROUGH
  (void)c;
#elif defined(USE_TINYUSB)
  if (nmea_passthrough_cdc) {
    nmea_passthrough_cdc.write(static_cast<uint8_t>(c));
  }
#else
  Serial.write(static_cast<uint8_t>(c));
#endif
}

static void updateGpsFixLed(bool usable_fix) {
  static int8_t last_state = -1;
  const int8_t state = usable_fix ? 1 : 0;
  if (state == last_state) {
    return;
  }
  last_state = state;

  const uint32_t color = usable_fix ? pixels.Color(0, 96, 0) : pixels.Color(75, 20, 0);
  pixels.setPixelColor(0, color);
  pixels.show();
}

// ---------- Scanner SPI transport ----------
static constexpr size_t SCANNER_FRAME_SIZE = SPI_FRAME_SIZE;
static constexpr uint32_t SCANNER_SPI_HZ = 20000000;
static constexpr size_t SCANNER_FRAME_TAG_INDEX = SPI_FRAME_TAG_INDEX;
static constexpr uint8_t SCANNER_SCAN_RESPONSE_PULLS = 24;
static constexpr uint8_t SCANNER_RESULT_RESPONSE_PULLS = 32;
static constexpr uint8_t SCANNER_SCAN_CMD_RETRIES = 3;
static constexpr uint8_t SCANNER_QUERY_CMD_RETRIES = 2;
// Scanner transport hardware config (mapped from legacy config macro names).
static constexpr int SCANNER_PIN_SCK = ESP_PIN_SCK;
static constexpr int SCANNER_PIN_MOSI = ESP_PIN_MOSI;
static constexpr int SCANNER_PIN_MISO = ESP_PIN_MISO;
static constexpr int SCANNER_PIN_CS = ESP_PIN_CS;
static constexpr uint32_t SCANNER_INTERFRAME_US = ESP_INTERFRAME_US;
static constexpr uint8_t SCANNER_DEVICE_TYPE = SCANNER_TYPE_ESP32_C5;
// Local transport status (not on-wire protocol).
static constexpr int8_t SCANNER_STATUS_TRANSPORT_TIMEOUT = -4;
static constexpr uint16_t SCANNER_SLOT_REPROBE_MS = 2000;
static constexpr uint8_t SCANNER_SLOT_TIMEOUT_REPROBE_THRESHOLD = 8;
#if SCANNER_USE_SHIFTREG_CS
static constexpr uint8_t SCANNER_SLOT_COUNT = SCANNER_SHIFTREG_OUTPUTS;
#else
static constexpr uint8_t SCANNER_SLOT_COUNT = 1;
#endif
static uint8_t scanner_cs_shiftreg_state = 0xFF;
static uint8_t scanner_active_slot = SCANNER_INITIAL_SLOT;

struct ScannerSlotState {
  // Cached identity state for this physical slot.
  bool id_valid;
  // One-shot log suppressors while the slot remains in same state.
  bool miss_logged;
  bool unsupported_logged;
  DeviceIdReply id;
  // Re-probe scheduling so absent slots do not get hammered every loop.
  uint32_t next_probe_ms;
  // Transport timeout streak for targeted stale-frame draining.
  uint8_t transport_timeouts;
};

static const uint8_t CHANNELS_24G[] = {
  1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14
};

static const uint8_t CHANNELS_5G[] = {
  36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, 149, 153, 157, 161, 165
};

// band_index: 0=2.4GHz list, 1=5GHz list.
// channel_index indexes into the current band's channel array.
static bool advanceSweepChannel(size_t& band_index, size_t& channel_index) {
  channel_index++;
  if (band_index == 0 && channel_index >= (sizeof(CHANNELS_24G) / sizeof(CHANNELS_24G[0]))) {
    band_index = 1;
    channel_index = 0;
    return false;
  }
  if (band_index == 1 && channel_index >= (sizeof(CHANNELS_5G) / sizeof(CHANNELS_5G[0]))) {
    band_index = 0;
    channel_index = 0;
    return true;
  }
  return false;
}

static inline void scannerSetActiveSlot(uint8_t slot) {
#if SCANNER_USE_SHIFTREG_CS
  scanner_active_slot = (uint8_t)(slot % SCANNER_SLOT_COUNT);
#else
  (void)slot;
  scanner_active_slot = 0;
#endif
}

#if SCANNER_USE_SHIFTREG_CS
static inline void scannerShiftRegWriteCsState(uint8_t state, bool force) {
  if (!force && scanner_cs_shiftreg_state == state) {
    return;
  }
  scanner_cs_shiftreg_state = state;
  SPI1.beginTransaction(SPISettings(SCANNER_SHIFTREG_SPI_HZ, MSBFIRST, SPI_MODE0));
  SPI1.transfer(scanner_cs_shiftreg_state);
  SPI1.endTransaction();
  digitalWrite(SCANNER_SHIFTREG_LATCH_PIN, HIGH);
  digitalWrite(SCANNER_SHIFTREG_LATCH_PIN, LOW);
}
#endif

static inline void advanceSweepChannelAndLog(size_t& band_index, size_t& channel_index) {
  if (advanceSweepChannel(band_index, channel_index)) {
    serialPrintlnTry("Completed full 2.4G + 5G sweep");
  }
}

static void scannerTransferFrame(const uint8_t* tx, uint8_t* rx) {
#if SCANNER_USE_SHIFTREG_CS
  // Active-low CS via shift-register output slot.
  const uint8_t assert_state = (uint8_t)(0xFFu & ~(1u << scanner_active_slot));
  scannerShiftRegWriteCsState(assert_state, false);
#else
  digitalWrite(SCANNER_PIN_CS, LOW);
#endif
  SPI1.beginTransaction(SPISettings(SCANNER_SPI_HZ, MSBFIRST, SPI_MODE0));

  for (size_t i = 0; i < SCANNER_FRAME_SIZE; i++) {
    const uint8_t t = tx ? tx[i] : 0;
    const uint8_t r = SPI1.transfer(t);
    if (rx) {
      rx[i] = r;
    }
  }

  SPI1.endTransaction();
#if SCANNER_USE_SHIFTREG_CS
  // Deassert all scanner CS outputs.
  const uint8_t deassert_state = 0xFFu;
  scannerShiftRegWriteCsState(deassert_state, false);
#else
  digitalWrite(SCANNER_PIN_CS, HIGH);
#endif
}

static inline void scannerInterframeWait() {
  if (SCANNER_INTERFRAME_US > 0) {
    delayMicroseconds(SCANNER_INTERFRAME_US);
  }
}

// Issue command frame, then pull NOP frames until response tag matches expected cmd.
static bool scannerCommandWithResponse(const uint8_t* cmd_frame,
                                   uint8_t expected_cmd_tag,
                                   uint8_t* response_frame,
                                   int max_pulls = 8) {
  uint8_t rx[SCANNER_FRAME_SIZE] = {0};
  uint8_t nop[SCANNER_FRAME_SIZE] = {0};
  nop[0] = CMD_NOP;
  scannerTransferFrame(cmd_frame, rx);
  scannerInterframeWait();

  for (int i = 0; i < max_pulls; i++) {
    scannerTransferFrame(nop, response_frame);
    if (response_frame[SCANNER_FRAME_TAG_INDEX] == expected_cmd_tag) {
      return true;
    }
    scannerInterframeWait();
  }
  return false;
}

static void scannerDrainStaleFrames(int pulls = 4) {
  uint8_t nop[SCANNER_FRAME_SIZE] = {0};
  uint8_t rx[SCANNER_FRAME_SIZE] = {0};
  nop[0] = CMD_NOP;
  for (int i = 0; i < pulls; i++) {
    scannerTransferFrame(nop, rx);
    scannerInterframeWait();
  }
}

static void scannerInitBus() {
#if SCANNER_USE_SHIFTREG_CS
  pinMode(SCANNER_SHIFTREG_LATCH_PIN, OUTPUT);
  pinMode(SCANNER_SHIFTREG_OE_PIN, OUTPUT);
  digitalWrite(SCANNER_SHIFTREG_LATCH_PIN, LOW);
  // Active-low OE: disable outputs while shift register state is initialized.
  digitalWrite(SCANNER_SHIFTREG_OE_PIN, HIGH);
#else
  digitalWrite(SCANNER_PIN_CS, HIGH);
  pinMode(SCANNER_PIN_CS, OUTPUT);
#endif
  SPI1.setSCK(SCANNER_PIN_SCK);
  SPI1.setTX(SCANNER_PIN_MOSI);
  SPI1.setRX(SCANNER_PIN_MISO);
  SPI1.begin();
#if SCANNER_USE_SHIFTREG_CS
  scannerSetActiveSlot(SCANNER_INITIAL_SLOT);
  scannerShiftRegWriteCsState(0xFFu, true);
  // Enable outputs once all scanner CS lines are safely deasserted.
  digitalWrite(SCANNER_SHIFTREG_OE_PIN, LOW);
#endif
}

static bool scannerGetDeviceId(DeviceIdReply& out) {
  uint8_t tx[SCANNER_FRAME_SIZE] = {0};
  uint8_t rx[SCANNER_FRAME_SIZE] = {0};

  for (int attempt = 0; attempt < 8; attempt++) {
    tx[0] = CMD_ID;
    if (scannerCommandWithResponse(tx, CMD_ID, rx, 10)) {
      memcpy(&out, rx, sizeof(out));
      if (out.proto_version == PROTO_VERSION) {
        return true;
      }
    }
    delay(1);
  }
  return false;
}

static bool ensureScannerSlotIdentity(uint8_t slot, ScannerSlotState& state) {
  if (state.id_valid) {
    return true;
  }

  const uint32_t now = millis();
  if ((int32_t)(now - state.next_probe_ms) < 0) {
    return false;
  }

  DeviceIdReply id = {};
  if (!scannerGetDeviceId(id)) {
    state.id_valid = false;
    state.transport_timeouts = 0;
    state.next_probe_ms = now + SCANNER_SLOT_REPROBE_MS;
    if (!state.miss_logged) {
      serialPrintfTry("S%u no valid scanner ID; skipping slot\n", (unsigned)slot);
      state.miss_logged = true;
    }
    return false;
  }

  // Only accept slots that speak our protocol and report sane limits.
  state.id = id;
  state.miss_logged = false;

  const bool proto_ok = (id.proto_version == PROTO_VERSION);
  const bool type_ok = (id.scanner_type == SCANNER_DEVICE_TYPE);
  const bool max_ok = (id.max_results > 0 && id.max_results <= PROTO_MAX_RESULTS);
  if (!(proto_ok && type_ok && max_ok)) {
    state.id_valid = false;
    state.transport_timeouts = 0;
    state.next_probe_ms = now + SCANNER_SLOT_REPROBE_MS;
    if (!state.unsupported_logged) {
      serialPrintfTry("S%u unsupported SPI device: proto=%u type=%u caps=0x%02X max=%u; skipping slot\n",
                      (unsigned)slot,
                      id.proto_version, id.scanner_type, id.capabilities, id.max_results);
      state.unsupported_logged = true;
    }
    return false;
  }

  state.id_valid = true;
  state.transport_timeouts = 0;
  state.next_probe_ms = 0;
  state.unsupported_logged = false;
  serialPrintfTry("S%u scanner ready: proto=%u type=%u caps=0x%02X max=%u\n",
                  (unsigned)slot,
                  id.proto_version, id.scanner_type, id.capabilities, id.max_results);
  scannerDrainStaleFrames(1);
  return true;
}

static int8_t scannerQueryStatusWithRetry(uint8_t cmd,
                                          uint8_t arg1,
                                          uint8_t arg2,
                                          uint8_t response_pulls,
                                          uint8_t retries,
                                          int8_t min_valid_status,
                                          int8_t max_valid_status) {
  for (uint8_t attempt = 0; attempt < retries; attempt++) {
    uint8_t tx[SCANNER_FRAME_SIZE] = {0};
    uint8_t rx[SCANNER_FRAME_SIZE] = {0};
    tx[0] = cmd;
    tx[1] = arg1;
    tx[2] = arg2;
    if (scannerCommandWithResponse(tx, cmd, rx, response_pulls)) {
      const int8_t status = (int8_t)rx[0];
      if (status >= min_valid_status && status <= max_valid_status) {
        return status;
      }
      // Corrupt/unknown status payload; retry in-place before giving up.
      continue;
    }
    // Lost/misaligned response; retry in-place before giving up.
  }
  return SCANNER_STATUS_TRANSPORT_TIMEOUT;
}

static bool scannerGetResult(WiFiResultPacket& pkt) {
  uint8_t tx[SCANNER_FRAME_SIZE] = {0};
  uint8_t rx[SCANNER_FRAME_SIZE] = {0};
  uint8_t nop[SCANNER_FRAME_SIZE] = {0};
  tx[0] = CMD_RESULT_GET;
  nop[0] = CMD_NOP;

  for (int attempt = 0; attempt < SCANNER_QUERY_CMD_RETRIES; attempt++) {
    scannerTransferFrame(tx, rx);
    scannerInterframeWait();

    for (int pull = 0; pull < SCANNER_RESULT_RESPONSE_PULLS; pull++) {
      scannerTransferFrame(nop, rx);
      if (rx[SCANNER_FRAME_TAG_INDEX] != CMD_RESULT_GET) {
        scannerInterframeWait();
        continue;
      }

      WiFiResultPacket candidate = {};
      memcpy(&candidate, rx, sizeof(candidate));
      if (candidate.result_type == RESULT_WIFI ||
          candidate.result_type == RESULT_BUSY ||
          candidate.result_type == RESULT_END) {
        pkt = candidate;
        return true;
      }

      // Valid tag but invalid payload; keep pulling to find a sane packet.
      scannerInterframeWait();
    }

    // Retry full RESULT_GET transaction before failing this record.
  }
  return false;
}

void sendPMTKCommand(const char* cmd) {
  GNSS_UART.println(cmd);
  Serial.print("Sent PMTK Command: ");
  Serial.println(cmd);
  delay(100); // Short delay to ensure command is processed
}

static void gnssUartBeginWithConfiguredBuffer(uint32_t baud) {
  static bool fifo_warned = false;
  if (!GNSS_UART.setFIFOSize(SERIAL_BUFFER_SIZE) && !fifo_warned) {
    Serial.print("Warning: GNSS UART FIFO resize failed, keeping default. requested=");
    Serial.println((unsigned)SERIAL_BUFFER_SIZE);
    fifo_warned = true;
  }
  GNSS_UART.begin(baud);
}

static void configureGnssBaudToTarget() {
  static const uint32_t probe_bauds[] = {115200, 9600, 38400};

  // Try the baud-rate switch command at common existing rates so legacy and
  // newer modules both converge to GNSS_TARGET_BAUD.
  for (size_t i = 0; i < (sizeof(probe_bauds) / sizeof(probe_bauds[0])); i++) {
    GNSS_UART.end();
    gnssUartBeginWithConfiguredBuffer(probe_bauds[i]);
    delay(30);
    GNSS_UART.println("$PMTK251,115200*1F");
    GNSS_UART.flush();
    delay(80);
  }

  GNSS_UART.end();
  gnssUartBeginWithConfiguredBuffer(GNSS_TARGET_BAUD);
  Serial.print("GNSS UART configured to ");
  Serial.print(GNSS_TARGET_BAUD);
  Serial.print(" baud (FIFO=");
  Serial.print((unsigned)SERIAL_BUFFER_SIZE);
  Serial.println(").");
}

static void handleResetButton() {
  static bool pressed_latched = false;
  static uint32_t pressed_ms = 0;
  const bool pressed = digitalRead(RESET_BUTTON_PIN) == LOW;

  if (!pressed) {
    pressed_latched = false;
    return;
  }

  if (!pressed_latched) {
    pressed_latched = true;
    pressed_ms = millis();
    return;
  }

  if ((millis() - pressed_ms) < RESET_BUTTON_DEBOUNCE_MS) {
    return;
  }

  Serial.println("Reset button pressed, rebooting Pico...");
  if (logFile) {
    logFile.flush();
  }
  Serial.flush();
  delay(20);
  watchdog_reboot(0, 0, 0);
  while (1) {
    delay(1);
  }
}

static void startScanForCurrentChannel(uint8_t slot,
                                       size_t& band_index,
                                       size_t& channel_index) {
  const uint8_t band = (band_index == 0) ? 2 : 5;
  const uint8_t channel = (band_index == 0)
    ? CHANNELS_24G[channel_index]
    : CHANNELS_5G[channel_index];

  const int8_t status = scannerQueryStatusWithRetry(CMD_SCAN,
                                                    band,
                                                    channel,
                                                    SCANNER_SCAN_RESPONSE_PULLS,
                                                    SCANNER_SCAN_CMD_RETRIES,
                                                    SCANNER_STATUS_START_FAILED,
                                                    SCANNER_STATUS_OK);
  if (status == SCANNER_STATUS_OK) {
    serialPrintfTry("S%u Scan started (band=%u ch=%u)\n",
                    (unsigned)slot, band, channel);
    advanceSweepChannelAndLog(band_index, channel_index);
    return;
  }
  if (status == SCANNER_STATUS_BUSY) {
    // If RESULT_COUNT said idle but SCAN says busy, the prior start likely latched
    // and we missed/garbled the ACK. Advance to avoid re-requesting same channel.
    serialPrintfTry("S%u Scan already active (band=%u ch=%u); assuming prior start latched\n",
                    (unsigned)slot, band, channel);
    advanceSweepChannelAndLog(band_index, channel_index);
    return;
  }
  if (status == SCANNER_STATUS_TRANSPORT_TIMEOUT) {
    // If SCAN ACK was lost but scanner is now busy, treat scan as started.
    const int8_t c = scannerQueryStatusWithRetry(CMD_RESULT_COUNT,
                                                 0,
                                                 0,
                                                 SCANNER_RESULT_RESPONSE_PULLS,
                                                 SCANNER_QUERY_CMD_RETRIES,
                                                 SCANNER_STATUS_BUSY,
                                                 static_cast<int8_t>(PROTO_MAX_RESULTS));
    if (c == SCANNER_STATUS_BUSY) {
      serialPrintfTry("S%u Scan started (band=%u ch=%u) via BUSY confirm\n",
                      (unsigned)slot, band, channel);
      advanceSweepChannelAndLog(band_index, channel_index);
      return;
    }
    // Timeout with no BUSY confirm: move on to avoid getting pinned on one channel.
    serialPrintfTry("S%u SCAN timeout (band=%u ch=%u); moving on\n",
                    (unsigned)slot, band, channel);
    advanceSweepChannelAndLog(band_index, channel_index);
    return;
  }

  // Unexpected scan status: move on to the next channel.
  serialPrintfTry("S%u SCAN unexpected status=%d (band=%u ch=%u); moving on\n",
                  (unsigned)slot, status, band, channel);
  advanceSweepChannelAndLog(band_index, channel_index);
}

// Per-slot scan pump:
// 1) Poll RESULT_COUNT.
// 2) If busy -> return.
// 3) If zero -> request scan on current channel.
// 4) If >0 -> drain that many packets and enqueue uniques.
static void processScannerSlot(uint8_t slot,
                               ScannerSlotState& slot_state,
                               size_t& band_index,
                               size_t& channel_index) {
  scannerSetActiveSlot(slot);
  const int8_t count = scannerQueryStatusWithRetry(CMD_RESULT_COUNT,
                                                   0,
                                                   0,
                                                   SCANNER_RESULT_RESPONSE_PULLS,
                                                   SCANNER_QUERY_CMD_RETRIES,
                                                   SCANNER_STATUS_BUSY,
                                                   static_cast<int8_t>(PROTO_MAX_RESULTS));

  // Active scan on slave; poll this slot again next round.
  if (count == SCANNER_STATUS_BUSY) {
    slot_state.transport_timeouts = 0;
    return;
  }

  if (count == SCANNER_STATUS_TRANSPORT_TIMEOUT) {
    if (slot_state.transport_timeouts < 0xFF) {
      slot_state.transport_timeouts++;
    }
    if (slot_state.transport_timeouts >= SCANNER_SLOT_TIMEOUT_REPROBE_THRESHOLD) {
      serialPrintfTry("S%u RESULT_COUNT timeout x%u; draining and continuing\n",
                      (unsigned)slot, (unsigned)slot_state.transport_timeouts);
      slot_state.transport_timeouts = 0;
      scannerDrainStaleFrames(4);
    }
    return;
  }
  slot_state.transport_timeouts = 0;

  if (count < 0) {
    // Unexpected status: move this slot's channel pointer.
    serialPrintfTry("S%u RESULT_COUNT unexpected=%d; moving on\n",
                    (unsigned)slot, count);
    advanceSweepChannelAndLog(band_index, channel_index);
    return;
  }

  // No results pending: request next scan for this slot.
  if (count == 0) {
    startScanForCurrentChannel(slot, band_index, channel_index);
    return;
  }

  // Results pending: drain up to reported count for this slot.
  bool scanner_busy_during_drain = false;
  uint16_t batch_dedupe_hits = 0;
  uint16_t batch_dedupe_logs = 0;
  for (int i = 0; i < count; i++) {
    WiFiResultPacket pkt = {};
    if (!scannerGetResult(pkt)) {
      break;
    }

    if (pkt.result_type == RESULT_WIFI) {
      if (!wifiResultIsValidForDedupe(pkt.result)) {
        const bool zero_bssid = wifiBssidIsZero(pkt.result.bssid);
        const bool ff_bssid = wifiBssidIsBroadcast(pkt.result.bssid);
        serialPrintfTry("S%u Dropped invalid WIFI result: ch=%u band=%u rssi=%d ssid0=0x%02X bssid_zero=%u bssid_ff=%u\n",
                        (unsigned)slot,
                        pkt.result.channel,
                        pkt.result.band,
                        pkt.result.rssi,
                        (uint8_t)pkt.result.ssid[0],
                        zero_bssid ? 1 : 0,
                        ff_bssid ? 1 : 0);
        continue;
      }

      WiFiDedupeHash hash = {};
      wifiDedupeHashFromResult(pkt.result, hash);
      if (!wifiDedupeTableRemember(&master_dedupe_table, &hash)) {
        dedupe_drops++;
        batch_dedupe_hits++;
#if LOG_DEDUPE_HITS
        if (batch_dedupe_logs < LOG_DEDUPE_HITS_MAX_PER_SCAN) {
          char mac[18] = {};
          pico_logging::formatBssid(pkt.result.bssid, mac, sizeof(mac));
          serialPrintfTry("[MASTER S%u] dedupe hit ssid='%s' bssid=%s ch=%u band=%u rssi=%d\n",
                          (unsigned)slot,
                          pkt.result.ssid,
                          mac,
                          pkt.result.channel,
                          pkt.result.band,
                          pkt.result.rssi);
          batch_dedupe_logs++;
        }
#endif
        continue;
      }

      QueuedScanResult q = {};
      strncpy(q.ssid, pkt.result.ssid, sizeof(q.ssid) - 1);
      memcpy(q.bssid, pkt.result.bssid, sizeof(q.bssid));
      q.rssi = pkt.result.rssi;
      q.channel = pkt.result.channel;
      q.band = pkt.result.band;
      q.capabilities = pkt.result.capabilities;
      if (!queue_try_add(&scan_result_queue, &q)) {
        scan_queue_drops++;
      }
      continue;
    }

    if (pkt.result_type == RESULT_END) {
      break;
    }
    if (pkt.result_type == RESULT_BUSY) {
      scanner_busy_during_drain = true;
      break;
    }

    serialPrintfTry("S%u Unknown result type: 0x%02X; draining stale frames\n",
                    (unsigned)slot, pkt.result_type);
    scannerDrainStaleFrames(2);
    break;
  }

#if LOG_DEDUPE_HITS
  if (batch_dedupe_hits > batch_dedupe_logs) {
    serialPrintfTry("[MASTER S%u] dedupe hits suppressed: %u (total=%u)\n",
                    (unsigned)slot,
                    (unsigned)(batch_dedupe_hits - batch_dedupe_logs),
                    (unsigned)batch_dedupe_hits);
  }
#endif

  if (scanner_busy_during_drain) {
    return;
  }
}

static void serialPrintRuntimeStatus() {
  Serial.printf("Queue drops=%lu serial_drop=%lu dedupe_drop=%lu logged=%lu blank_gps=%lu\n",
                (unsigned long)scan_queue_drops,
                (unsigned long)serial_msg_drops,
                (unsigned long)dedupe_drops,
                (unsigned long)logging_state.csv_rows,
                (unsigned long)logging_state.csv_rows_blank_gps);
}

static void serialPrintGpsStatus(bool usable_fix) {
  Serial.printf("GPS: usable=%s loc_valid=%s updated=%s lat=%.7f lon=%.7f hdop=%.2f sats=%u age=%lu chars=%lu fix=%lu cksum_fail=%lu\n",
                usable_fix ? "YES" : "NO",
                gps.location.isValid() ? "YES" : "NO",
                gps.location.isUpdated() ? "YES" : "NO",
                gps.location.isValid() ? gps.location.lat() : 0.0,
                gps.location.isValid() ? gps.location.lng() : 0.0,
                gps.hdop.isValid() ? gps.hdop.hdop() : 0.0,
                gps.satellites.isValid() ? gps.satellites.value() : 0,
                (unsigned long)gps.location.age(),
                (unsigned long)gps.charsProcessed(),
                (unsigned long)gps.sentencesWithFix(),
                (unsigned long)gps.failedChecksum());
}

static void printPeriodicStatus(bool usable_fix) {
  static uint32_t lastStatMs = 0;
  const uint32_t now = millis();
  if ((now - lastStatMs) <= 10000) {
    return;
  }
  lastStatMs = now;
  serialPrintRuntimeStatus();
  serialPrintGpsStatus(usable_fix);
}

void loop2() {
  static ScannerSlotState slot_state[SCANNER_SLOT_COUNT] = {};
  static size_t band_index[SCANNER_SLOT_COUNT] = {};
  static size_t channel_index[SCANNER_SLOT_COUNT] = {};
  static uint8_t slot = SCANNER_INITIAL_SLOT;

  serialPrintlnTry("Core1 SPI loop started");
#if SCANNER_USE_SHIFTREG_CS
  for (uint8_t s = 0; s < SCANNER_SLOT_COUNT; s++) {
    scannerSetActiveSlot(s);
    scannerDrainStaleFrames(1);
  }
  scannerSetActiveSlot(slot);
#else
  scannerDrainStaleFrames();
#endif

  while (true) {
    // Each iteration services exactly one slot; round-robin when shift-reg CS is enabled.
    scannerSetActiveSlot(slot);
    if (ensureScannerSlotIdentity(slot, slot_state[slot])) {
      processScannerSlot(slot, slot_state[slot], band_index[slot], channel_index[slot]);
    }
#if SCANNER_USE_SHIFTREG_CS
    slot = (uint8_t)((slot + 1u) % SCANNER_SLOT_COUNT);
#endif
  }
}

void setup() {
  // Bring up status LED first so bare-board bring-up has immediate feedback.
#if RGB_POWER_ENABLED
  pinMode(RGB_POWER_PIN, OUTPUT);
  digitalWrite(RGB_POWER_PIN, HIGH);
  delay(10);  // Let WS2812 power rail settle before first data frame.
#endif
  pixels.begin();
  pixels.setPixelColor(0, pixels.Color(128, 0, 0));
  pixels.show();

#if defined(USE_TINYUSB)
  TinyUSBDevice.setManufacturerDescriptor(USB_DEVICE_MANUFACTURER_NAME);
  TinyUSBDevice.setProductDescriptor(USB_DEVICE_PRODUCT_NAME);
  if (USB_DEVICE_SERIAL_OVERRIDE[0] != '\0') {
    TinyUSBDevice.setSerialDescriptor(USB_DEVICE_SERIAL_OVERRIDE);
  }
#endif
  Serial.begin(115200);
#if defined(USE_TINYUSB)
  nmea_passthrough_cdc.begin(115200);
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }
#endif
  delay(2000);

  Serial.println("WiFi Shuriken Startup");
  Serial.printf("RGB config: data=%d power_en=%d power_pin=%d\n",
                RGB_PIN, RGB_POWER_ENABLED, RGB_POWER_PIN);
  logging.initUtcTimezone();
#if defined(USE_TINYUSB)
  Serial.printf("USB CDC IDs: CDC0='%s' CDC1='%s'\n",
                USB_CDC0_IFACE_NAME, USB_CDC1_IFACE_NAME);
#endif
  logging.registerSdDateTimeCallback();
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  Serial.println("Reset button enabled on GPIO17");
  // Setup SD Card on SPI0
  SPI.setSCK(SD_PIN_SCK);
  SPI.setTX(SD_PIN_MOSI);
  SPI.setRX(SD_PIN_MISO);
#if SD_MISO_PULLUP
  gpio_pull_up(SD_PIN_MISO);
#endif
  pinMode(SD_PIN_CS, OUTPUT);
  digitalWrite(SD_PIN_CS, HIGH);
  SPI.begin();
#if SD_MISO_PULLUP
  gpio_pull_up(SD_PIN_MISO);
  Serial.println("SD MISO internal pull-up enabled");
#endif

  // Initialize SD card with retries and fallback clocks.
  logging_state.sd_ready = logging.mountSdWithRetries(SD_INIT_MAX_ATTEMPTS);
  if (!logging_state.sd_ready) {
    Serial.println("SD initialization failed after retries; continuing without SD logging.");
    logging_state.sd_next_retry_ms = millis() + SD_RETRY_BACKGROUND_MS;
  } else {
    Serial.println("SD initialization done");
  }
  queue_init(&scan_result_queue, sizeof(QueuedScanResult), 128);
  queue_init(&serial_msg_queue, sizeof(QueuedSerialMsg), 64);
  wifiDedupeTableInit(&master_dedupe_table, master_dedupe_storage, MASTER_DEDUPE_CAPACITY);
  wifiDedupeTableReset(&master_dedupe_table);

  // Initialize SPI1 for scanner communication
  scannerInitBus();

#if SCANNER_USE_SHIFTREG_CS
  Serial.printf("SPI1 initialized (shift-register scanner CS enabled, slots=%u).\n",
                (unsigned)SCANNER_SLOT_COUNT);
#else
  Serial.println("SPI1 initialized.");
#endif
  // Initialize GNSS UART
  GNSS_UART.setTX(GNSS_TX);
  GNSS_UART.setRX(GNSS_RX);
  configureGnssBaudToTarget();

  // Configure GNSS module for 5Hz update rate and disable unnecessary NMEA sentences
  sendPMTKCommand("$PMTK220,200*2C"); // Set update rate to 5Hz
  sendPMTKCommand("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"); // Enable GGA and RMC sentences only

  // Capture RTC-backed GNSS date/time once per boot for the log filename.
  logging.captureBootTimestampFromGnss();

  if (logging_state.sd_ready) {
    if (!logging.ensureBootTimestampFromClock()) {
      Serial.println("Waiting for valid GNSS time before creating CSV log file");
      logging_state.sd_next_retry_ms = millis() + CSV_LOG_TIME_WAIT_RETRY_MS;
    } else if (!logging.selectCsvLogPath()) {
      Serial.println("CSV log path selection failed");
      logging_state.sd_next_retry_ms = millis() + SD_RETRY_BACKGROUND_MS;
    } else {
      logging_state.csv_ready = logging.initCsvLog(logging_state.csv_log_path);
      if (logging_state.csv_ready) {
        Serial.print("CSV log ready: ");
        Serial.println(logging_state.csv_log_path);
      } else {
        Serial.print("CSV log open failed: ");
        Serial.println(logging_state.csv_log_path);
        logging_state.sd_next_retry_ms = millis() + SD_RETRY_BACKGROUND_MS;
      }
    }
  }

  // Set up second core loop for SPI communication with scanner
  multicore_launch_core1(loop2);

}

void loop() {
  handleResetButton();

  // Read GNSS data and update GPS info.
  while (GNSS_UART.available()) {
    char c = GNSS_UART.read();
    gps.encode(c);

#if NMEA_PASSTHROUGH
    nmeaPassthroughWriteChar(c);
#endif
  }
  logging.syncMasterClockFromGnss();

  const bool usable_fix =
      gps.location.isValid() &&
      gps.location.age() < GPS_FIELD_MAX_AGE_MS &&
      gps.satellites.isValid() &&
      gps.satellites.value() >= 3;
  updateGpsFixLed(usable_fix);

  // Drain core1 serial messages via queue (caps work per iteration).
  QueuedSerialMsg sm = {};
  int serial_drained = 0;
  while (serial_drained < 16 && queue_try_remove(&serial_msg_queue, &sm)) {
    Serial.print(sm.text);
    serial_drained++;
  }

  // Drain AP results produced by core1 and handle logging on core0.
  QueuedScanResult q = {};
  // Limit per-iteration SD writes so GNSS parsing is serviced frequently.
  int drained = 0;
  while (drained < 16 && queue_try_remove(&scan_result_queue, &q)) {
    logging.appendCsvRow(q);
    drained++;
  }
  // Time-based flush closes the durability gap between row-count flushes.
  logging.flushCsvIfDue();

  if (!logging_state.csv_ready) {
    logging.tryRecoverSdLogging();
  }

  printPeriodicStatus(usable_fix);
}
