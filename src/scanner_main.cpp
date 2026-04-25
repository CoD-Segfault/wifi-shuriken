#include <Arduino.h>
#include "spi_protocol_shared.h"
#include "wifi_dedupe.h"
#include "wifi_result_utils.h"
#include "scanner_platform_esp32_radio.h"
#include "scanner_platform_esp32_spi.h"
#include "ScannerConfiguration.h"
#include <esp_system.h>
#include <Update.h>

/*
  ===== SPI RESPONSE TIMING (IMPORTANT) =====
  With SPI slave + spi_slave_transmit(), the slave must have tx_buf filled BEFORE the master clocks.
  That means the response to a command is available on the *next* SPI transaction.

  Recommended master pattern (simple):
    1) TX CMD_xxxxx (ignore RX)
    2) TX CMD_NOP   (read RX = response to CMD_xxxxx)

  Or pipeline (more efficient):
    - Each transaction sends the next command while receiving the previous command's response.
*/

#ifndef LED_BUILTIN
#define LED_BUILTIN -1
#endif

#if SCANNER_SERIAL_LOG
#define SCANNER_LOG_PRINTF(...) Serial.printf(__VA_ARGS__)
#define SCANNER_LOG_PRINTLN(x) Serial.println(x)
#else
#define SCANNER_LOG_PRINTF(...)
#define SCANNER_LOG_PRINTLN(x)
#endif

#if DEBUG_SPI_PROTOCOL
#define DBG_PRINTF(...) Serial.printf(__VA_ARGS__)
#define DBG_PRINTLN(x) Serial.println(x)
#else
#define DBG_PRINTF(...)
#define DBG_PRINTLN(x)
#endif

// ---------- Scan tuning ----------
static constexpr uint16_t ACTIVE_DWELL_MS  = 110;
static constexpr uint16_t PASSIVE_DWELL_MS = 210;
static constexpr uint8_t BAND_5_GHZ = 5;
static constexpr uint16_t SPI_RX_TIMEOUT_MS = 50;
static constexpr uint8_t SPI_TRANSACTION_QUEUE_SIZE = 4;
static constexpr uint8_t SCAN_RESULT_PROCESS_BUDGET = 16;

// ---------- Fixed SPI frame size ----------
// Keep a fixed frame size for all transactions (simplifies master & DMA alignment).
static constexpr size_t FRAME_SIZE = SPI_FRAME_SIZE;
static constexpr size_t FRAME_TAG_INDEX = SPI_FRAME_TAG_INDEX;

static uint8_t rx_buf[FRAME_SIZE] __attribute__((aligned(4)));
static uint8_t tx_buf[FRAME_SIZE] __attribute__((aligned(4)));

// Boot diagnostics are emitted repeatedly for a short window so they remain visible
// even when USB CDC enumeration lags behind firmware startup.
static esp_reset_reason_t boot_reset_reason = ESP_RST_UNKNOWN;
static uint32_t boot_diag_next_ms = 0;
static uint8_t boot_diag_emits = 0;
static constexpr uint8_t BOOT_DIAG_MAX_EMITS = 30;      // 15s at 500ms cadence
static constexpr uint16_t BOOT_DIAG_PERIOD_MS = 500;

// ---------- Scan state ----------
// Two-stage pipeline:
// 1) Radio scan snapshot lifecycle (scan_phase / scan_count / scan_index).
// 2) SPI-visible deduped buffer lifecycle (spi_results / spi_result_count / spi_result_index / spi_status).
enum class ScanPhase : uint8_t {
  Idle = 0,
  Scanning = 1,
  Processing = 2,
};

static ScanPhase scan_phase = ScanPhase::Idle;
static int scan_count = 0;     // total results in current scan snapshot
static int scan_index = 0;     // next raw result index to process
static WiFiResult spi_results[PROTO_MAX_RESULTS] = {};
static uint8_t spi_result_count = 0;
static uint8_t spi_result_index = 0;
// Master-visible status: 0=idle, -1=busy (scan/processing), >0=buffered results available.
static int8_t spi_status = SCANNER_STATUS_OK;
static uint8_t last_scan_band = 0;
static uint8_t last_scan_channel = 0;
static WiFiDedupeHash scan_dedupe_slots[WIFI_DEDUPE_TABLE_SIZE];
static WiFiDedupeHash scan_dedupe_fifo[WIFI_DEDUPE_TABLE_CAPACITY];
static WiFiDedupeTable scan_dedupe_table = {};
#if LOG_DEDUPE_HITS
static uint32_t scan_dedupe_hits = 0;
static uint32_t scan_dedupe_logs = 0;
#endif

// Optional: immediate status for CMD_SCAN response (returned next transaction)
static int8_t last_scan_status = SCANNER_STATUS_OK;

struct ScannerOtaState {
  bool active;
  uint32_t expected_size;
  uint32_t received_size;
  uint64_t expected_hash;
  uint64_t running_hash;
  uint16_t next_sequence;
};

static ScannerOtaState ota_state = {};
static bool ota_restart_pending = false;
static uint32_t ota_diag_nop_polls = 0;
static uint32_t ota_diag_data_frames = 0;
static uint32_t ota_diag_other_frames = 0;
static OtaAck ota_pending_ack = {};
static uint8_t ota_pending_ack_tag = CMD_NOP;
static bool ota_pending_ack_valid = false;
static constexpr uint16_t OTA_RESTART_DELAY_MS = 100;
static constexpr uint32_t OTA_DEBUG_SECTOR_BYTES = SCANNER_OTA_DEBUG_SECTOR_BYTES;
static constexpr uint32_t OTA_DEBUG_EDGE_BYTES = SCANNER_OTA_DEBUG_EDGE_BYTES;
static constexpr uint32_t OTA_DEBUG_SLOW_WRITE_MS = SCANNER_OTA_DEBUG_SLOW_WRITE_MS;

// ---------- Helpers ----------
static void scannerStatusLedSet(bool on) {
  if (!SCANNER_STATUS_LED_ENABLED || ((int)SCANNER_STATUS_LED_PIN < 0)) {
    return;
  }
  const bool drive_high = SCANNER_STATUS_LED_ACTIVE_LOW ? !on : on;
  digitalWrite(SCANNER_STATUS_LED_PIN, drive_high ? HIGH : LOW);
}

static const char* cmd_to_str(uint8_t cmd) {
  switch (static_cast<SpiCommand>(cmd)) {
    case CMD_NOP: return "NOP";
    case CMD_ID: return "ID";
    case CMD_SCAN: return "SCAN";
    case CMD_RESULT_COUNT: return "RESULT_COUNT";
    case CMD_RESULT_GET: return "RESULT_GET";
    case CMD_DEDUPE_RESET: return "DEDUPE_RESET";
    case CMD_OTA_BEGIN: return "OTA_BEGIN";
    case CMD_OTA_DATA: return "OTA_DATA";
    case CMD_OTA_END: return "OTA_END";
    case CMD_OTA_ABORT: return "OTA_ABORT";
    default: return "UNKNOWN";
  }
}

static const char* reset_reason_to_str(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_UNKNOWN: return "UNKNOWN";
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXT";
    case ESP_RST_SW: return "SW";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    case ESP_RST_USB: return "USB";
    case ESP_RST_JTAG: return "JTAG";
    case ESP_RST_EFUSE: return "EFUSE";
    case ESP_RST_PWR_GLITCH: return "PWR_GLITCH";
    case ESP_RST_CPU_LOCKUP: return "CPU_LOCKUP";
    default: return "OTHER";
  }
}

static void emitBootDiagnosticsIfDue() {
#if !SCANNER_SERIAL_LOG
  return;
#else
  if (boot_diag_emits >= BOOT_DIAG_MAX_EMITS) {
    return;
  }
  const uint32_t now = millis();
  if ((int32_t)(now - boot_diag_next_ms) < 0) {
    return;
  }
  boot_diag_next_ms = now + BOOT_DIAG_PERIOD_MS;
  boot_diag_emits++;
  SCANNER_LOG_PRINTF("Boot reset reason: %d (%s)\n",
                     (int)boot_reset_reason,
                     reset_reason_to_str(boot_reset_reason));
#endif
}

static bool is_dfs_channel(uint8_t band, uint8_t channel) {
  // DFS definition for your use case: 5GHz channels 52-144
  return (band == BAND_5_GHZ) && (channel >= 52) && (channel <= 144);
}

static void clear_scan_results() {
  if (scan_count > 0) {
    // Release backend snapshot memory once we have consumed/aborted it.
    scannerRadioDeleteScanResults();
  }
  scan_count = 0;
  scan_index = 0;
#if LOG_DEDUPE_HITS
  scan_dedupe_hits = 0;
  scan_dedupe_logs = 0;
#endif
}

static void clear_spi_result_buffer() {
  spi_result_count = 0;
  spi_result_index = 0;
}

static inline void setResultBufferIdle() {
  spi_status = SCANNER_STATUS_OK;
  clear_spi_result_buffer();
}

static inline void setScanEngineIdle() {
  scan_phase = ScanPhase::Idle;
  spi_status = SCANNER_STATUS_OK;
  scannerStatusLedSet(false);
}

static inline void abortScanAndSetIdle() {
  clear_scan_results();
  clear_spi_result_buffer();
  setScanEngineIdle();
}

static void resetDedupeAndBuffers() {
  abortScanAndSetIdle();
  wifiDedupeTableReset(&scan_dedupe_table);
  last_scan_status = SCANNER_STATUS_OK;
}

static void write_status_response(int8_t value) {
  memcpy(tx_buf, &value, 1);
}

static void clearOtaState() {
  ota_state = {};
  ota_diag_nop_polls = 0;
  ota_diag_data_frames = 0;
  ota_diag_other_frames = 0;
}

static void clearOtaPendingAck() {
  ota_pending_ack = {};
  ota_pending_ack_tag = CMD_NOP;
  ota_pending_ack_valid = false;
}

static void abortOtaState() {
  if (ota_state.active) {
    Update.abort();
  }
  clearOtaState();
  clearOtaPendingAck();
  ota_restart_pending = false;
}

static void write_ota_ack(OtaStatus status, uint32_t detail = 0) {
  OtaAck ack = {};
  ack.status = static_cast<int8_t>(status);
  ack.active = ota_state.active ? 1 : 0;
  ack.next_sequence = ota_state.next_sequence;
  ack.offset = ota_state.received_size;
  ack.detail = detail;
  ack.image_hash = ota_state.running_hash;
  memcpy(tx_buf, &ack, sizeof(ack));
  ota_pending_ack = ack;
  ota_pending_ack_tag = tx_buf[FRAME_TAG_INDEX];
  ota_pending_ack_valid = true;
}

static bool otaShouldLogPacket(uint32_t offset, uint8_t length) {
#if !SCANNER_OTA_DEBUG_LOG
  (void)offset;
  (void)length;
  return false;
#else
  if (offset < 512) {
    return true;
  }
  if (OTA_DEBUG_SECTOR_BYTES == 0) {
    return false;
  }
  const uint32_t sector_offset = offset % OTA_DEBUG_SECTOR_BYTES;
  const uint32_t end_offset = (offset + length) % OTA_DEBUG_SECTOR_BYTES;
  if (sector_offset < OTA_DEBUG_EDGE_BYTES ||
      sector_offset >= (OTA_DEBUG_SECTOR_BYTES - OTA_DEBUG_EDGE_BYTES) ||
      end_offset < OTA_DEBUG_EDGE_BYTES) {
    return true;
  }
  return false;
#endif
}

static void otaLogAck(const char* reason, OtaStatus status, uint32_t detail = 0) {
#if !SCANNER_OTA_DEBUG_LOG
  (void)reason;
  (void)status;
  (void)detail;
  return;
#else
  SCANNER_LOG_PRINTF("Scanner OTA ack %s: status=%d offset=%lu next_seq=%u detail=0x%08lx active=%u\n",
                     reason,
                     (int)status,
                     (unsigned long)ota_state.received_size,
                     (unsigned)ota_state.next_sequence,
                     (unsigned long)detail,
                     ota_state.active ? 1u : 0u);
#endif
}

static void otaLogRawFrame(uint8_t cmd) {
#if !SCANNER_OTA_DEBUG_LOG
  (void)cmd;
  return;
#else
  if (!ota_state.active) {
    return;
  }

  if (cmd == CMD_NOP) {
    ota_diag_nop_polls++;
    if (ota_diag_nop_polls == 1 || ota_diag_nop_polls == 10 ||
        ota_diag_nop_polls == 100 || ota_diag_nop_polls == 1000 ||
        (ota_diag_nop_polls % 10000UL) == 0) {
      SCANNER_LOG_PRINTF("Scanner OTA raw NOP poll count=%lu tx_tag=0x%02X offset=%lu next_seq=%u\n",
                         (unsigned long)ota_diag_nop_polls,
                         tx_buf[FRAME_TAG_INDEX],
                         (unsigned long)ota_state.received_size,
                         (unsigned)ota_state.next_sequence);
    }
    return;
  }

  if (cmd == CMD_OTA_DATA) {
    ota_diag_data_frames++;
    OtaDataCommand data = {};
    memcpy(&data, rx_buf, sizeof(data));
    if (!otaShouldLogPacket(data.offset, data.length)) {
      return;
    }
  } else {
    ota_diag_other_frames++;
  }

  SCANNER_LOG_PRINTF("Scanner OTA raw cmd=0x%02X data_frames=%lu other=%lu nops=%lu hdr=%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X tag=0x%02X offset=%lu next_seq=%u\n",
                     cmd,
                     (unsigned long)ota_diag_data_frames,
                     (unsigned long)ota_diag_other_frames,
                     (unsigned long)ota_diag_nop_polls,
                     rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3],
                     rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7],
                     rx_buf[8], rx_buf[9], rx_buf[10], rx_buf[11],
                     rx_buf[FRAME_TAG_INDEX],
                     (unsigned long)ota_state.received_size,
                     (unsigned)ota_state.next_sequence);
  Serial.flush();
#endif
}

#if LOG_DEDUPE_HITS
static void noteDedupeHit(int index, const WiFiResult& r) {
  scan_dedupe_hits++;
  if (scan_dedupe_logs < LOG_DEDUPE_HITS_MAX_PER_SCAN) {
    SCANNER_LOG_PRINTF("[SLAVE] dedupe hit idx=%d ssid='%s' ch=%u band=%u rssi=%d\n",
                       index, r.ssid, r.channel, r.band, r.rssi);
    scan_dedupe_logs++;
  }
}

static void logDedupeSuppressedSummary() {
  if (scan_dedupe_hits > scan_dedupe_logs) {
    SCANNER_LOG_PRINTF("[SLAVE] dedupe hits suppressed: %lu (total=%lu)\n",
                       (unsigned long)(scan_dedupe_hits - scan_dedupe_logs),
                       (unsigned long)scan_dedupe_hits);
  }
}
#endif

static void process_scan_results_into_spi_buffer(int budget = SCAN_RESULT_PROCESS_BUDGET) {
  if (scan_phase != ScanPhase::Processing) {
    return;
  }

  // Budgeted draining keeps loop latency bounded when scans are dense.
  int processed = 0;
  while (scan_index < scan_count &&
         spi_result_count < PROTO_MAX_RESULTS &&
         processed < budget) {
    const int raw_index = scan_index++;
    processed++;

    WiFiResult r = {};
    if (!scannerRadioReadResult(raw_index, r)) {
      // Skip unreadable raw entries but keep progressing the snapshot cursor.
      continue;
    }

    if (!wifiResultIsValidForDedupe(r)) {
      // Ignore invalid radio results so they don't poison dedupe state.
      continue;
    }

    WiFiDedupeHash hash = {};
    wifiDedupeHashFromResult(r, hash);
    if (!wifiDedupeTableRemember(&scan_dedupe_table, &hash)) {
#if LOG_DEDUPE_HITS
      noteDedupeHit(raw_index, r);
#endif
      continue;
    }

    spi_results[spi_result_count++] = r;
  }

  // When all raw entries are consumed (or protocol buffer is full),
  // publish the final count to the master as the new status.
  if (scan_index >= scan_count || spi_result_count >= PROTO_MAX_RESULTS) {
#if LOG_DEDUPE_HITS
    logDedupeSuppressedSummary();
#endif
    clear_scan_results();
    scan_phase = ScanPhase::Idle;
    spi_status = static_cast<int8_t>(spi_result_count);
    // Snapshot fully processed; scanner is now idle from RF perspective.
    scannerStatusLedSet(false);
    SCANNER_LOG_PRINTF("Buffered %u unique results (band %u ch %u)\n",
                       (unsigned)spi_result_count, last_scan_band, last_scan_channel);
  }
}

static void start_channel_scan(uint8_t band, uint8_t channel) {
  // Discard any previous snapshot before starting a new scan.
  clear_scan_results();
  setResultBufferIdle();

  // DFS channels should be passive to avoid radar CAC delays, 210ms dwell
  const bool passive = is_dfs_channel(band, channel);
  const int dwell = passive ? PASSIVE_DWELL_MS : ACTIVE_DWELL_MS;

  // Save scan metadata for completion logs.
  last_scan_band = band;
  last_scan_channel = channel;

  // Start the WiFi scan on the specified channel (async)
  // scanNetworks(bool async, bool show_hidden, bool passive, uint32_t max_ms_per_chan, uint8_t channel)
  const int n = scannerRadioStartAsyncScan(band, channel, passive, dwell);
  DBG_PRINTF("[SCAN] start request band=%u ch=%u passive=%d dwell=%d ret=%d\n",
             band, channel, passive ? 1 : 0, dwell, n);

  if (n == SCANNER_RADIO_SCAN_FAILED) {
    SCANNER_LOG_PRINTF("Scan start failed on band %u ch %u (ret=%d)\n", band, channel, n);
    setScanEngineIdle();
    last_scan_status = SCANNER_STATUS_START_FAILED;
    return;
  }

  scan_phase = ScanPhase::Scanning;
  last_scan_status = SCANNER_STATUS_OK;
  spi_status = SCANNER_STATUS_BUSY;
  scannerStatusLedSet(true);
  SCANNER_LOG_PRINTF("Started scan band %u ch %u (%s, %dms, ret=%d)\n",
                     band, channel, passive ? "passive" : "active", dwell, n);
}

static void update_scan_state() {
  // Phase 1: poll radio async state until scan snapshot is complete.
  if (scan_phase == ScanPhase::Scanning) {
    const int n = scannerRadioPollAsyncScan();

    if (n == SCANNER_RADIO_SCAN_RUNNING) {
      return;
    }

    if (n < 0) {
      // Failed or aborted.
      abortScanAndSetIdle();
      SCANNER_LOG_PRINTLN("Scan failed/aborted");
      return;
    }

    // Scan completed: keep status busy while we process and dedupe.
    scan_phase = ScanPhase::Processing;
    scan_index = 0;
    spi_status = SCANNER_STATUS_BUSY;

    // Safety: enforce protocol max.
    if (n > PROTO_MAX_RESULTS) {
      SCANNER_LOG_PRINTF("Warning: scan returned %d results; clamping to %u\n",
                         n, (unsigned)PROTO_MAX_RESULTS);
      scan_count = PROTO_MAX_RESULTS;
    } else {
      scan_count = n;
    }

    SCANNER_LOG_PRINTF("Scan complete: %d results (band %u ch %u)\n",
                       scan_count, last_scan_band, last_scan_channel);
  }

  // Phase 2: post-process raw snapshot into deduped SPI buffer.
  if (scan_phase == ScanPhase::Processing) {
    process_scan_results_into_spi_buffer();
  }
}

// Prepare a WiFiResultPacket into tx_buf (first 48 bytes).
static void write_result_packet(ResultType type, const WiFiResult* r = nullptr) {
  WiFiResultPacket pkt = {};
  pkt.result_type = static_cast<uint8_t>(type);
  if (r) {
    pkt.result = *r;
  }
  // pkt padded automatically by zero init
  memcpy(tx_buf, &pkt, sizeof(pkt));
}

static void handleCmdNop() {
  if (ota_pending_ack_valid) {
    memcpy(tx_buf, &ota_pending_ack, sizeof(ota_pending_ack));
    tx_buf[FRAME_TAG_INDEX] = ota_pending_ack_tag;
#if SCANNER_OTA_DEBUG_LOG
    if (ota_state.active && (ota_diag_nop_polls % 50000UL) == 0) {
      SCANNER_LOG_PRINTF("[SPI] rsp NOP => repeat OTA ack tag=0x%02X status=%d offset=%lu next_seq=%u\n",
                         ota_pending_ack_tag,
                         (int)ota_pending_ack.status,
                         (unsigned long)ota_pending_ack.offset,
                         (unsigned)ota_pending_ack.next_sequence);
    }
#endif
    return;
  }

  // Return current scanner status/count semantics as a convenience.
  const int8_t c = spi_status;
  write_status_response(c);
  DBG_PRINTF("[SPI] rsp NOP => count=%d\n", c);
}

static void handleCmdId() {
  DeviceIdReply id = {};
  id.proto_version = PROTO_VERSION;
  id.scanner_type  = SCANNER_TYPE_ESP32_C5;
  id.capabilities  = (CAP_BAND_24GHZ | CAP_BAND_5GHZ | CAP_OTA_UPDATE);
  id.max_results   = PROTO_MAX_RESULTS;
  memcpy(tx_buf, &id, sizeof(id));
  DBG_PRINTF("[SPI] rsp ID => proto=%u type=%u caps=0x%02X max=%u\n",
             id.proto_version, id.scanner_type, id.capabilities, id.max_results);
}

static void handleCmdScan() {
  // Note: response comes next transaction: we return last_scan_status (int8_t)
  ScanCommand sc = {};
  memcpy(&sc, rx_buf, sizeof(sc));

  // Reject new scans while a previous scan/snapshot/buffer is still in flight.
  if (scan_phase != ScanPhase::Idle ||
      spi_status != SCANNER_STATUS_OK) {
    last_scan_status = SCANNER_STATUS_BUSY;
  } else if (!wifiBandChannelValid(sc.band, sc.channel)) {
    last_scan_status = SCANNER_STATUS_INVALID;
    DBG_PRINTF("[SCAN] invalid request raw=%02X,%02X,%02X,%02X\n",
               rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
  } else {
    last_scan_status = SCANNER_STATUS_OK;
    start_channel_scan(sc.band, sc.channel);
  }

  write_status_response(last_scan_status);
  DBG_PRINTF("[SPI] rsp SCAN => status=%d (band=%u ch=%u)\n",
             last_scan_status, sc.band, sc.channel);
}

static void handleCmdResultCount() {
  const int8_t c = spi_status;
  write_status_response(c);
  DBG_PRINTF("[SPI] rsp RESULT_COUNT => %d (phase=%u total=%d idx=%d)\n",
             c, (unsigned)scan_phase, scan_count, scan_index);
}

static void handleCmdResultGet() {
  if (spi_status == SCANNER_STATUS_BUSY) {
    write_result_packet(RESULT_BUSY);
    DBG_PRINTLN("[SPI] rsp RESULT_GET => BUSY");
    return;
  }

  // No buffered results ready -> end marker.
  if (spi_status <= 0 || spi_result_count == 0 || spi_result_index >= spi_result_count) {
    write_result_packet(RESULT_END);
    DBG_PRINTLN("[SPI] rsp RESULT_GET => END");
    // END always collapses transfer state back to idle.
    setResultBufferIdle();
    return;
  }

  const WiFiResult& r = spi_results[spi_result_index++];
  write_result_packet(RESULT_WIFI, &r);

  const int remaining = (int)spi_result_count - (int)spi_result_index;
  if (remaining > 0) {
    spi_status = static_cast<int8_t>(remaining);
  } else {
    setResultBufferIdle();
  }

  DBG_PRINTF("[SPI] rsp RESULT_GET => WIFI out=%u/%u ssid='%s' rssi=%d ch=%u band=%u\n",
             (unsigned)spi_result_index, (unsigned)spi_result_count,
             r.ssid, r.rssi, r.channel, r.band);
}

static void handleCmdDedupeReset() {
  // Reset is allowed only when not actively scanning/processing.
  if (scan_phase != ScanPhase::Idle) {
    write_status_response(SCANNER_STATUS_BUSY);
    DBG_PRINTLN("[SPI] rsp DEDUPE_RESET => BUSY");
    return;
  }

  resetDedupeAndBuffers();
  write_status_response(SCANNER_STATUS_OK);
  DBG_PRINTLN("[SPI] rsp DEDUPE_RESET => OK");
}

static void handleCmdOtaBegin() {
  OtaBeginCommand begin = {};
  memcpy(&begin, rx_buf, sizeof(begin));

  if (scan_phase != ScanPhase::Idle || spi_status != SCANNER_STATUS_OK) {
    write_ota_ack(OTA_STATUS_BUSY);
    DBG_PRINTLN("[SPI] rsp OTA_BEGIN => BUSY");
    return;
  }

  if (begin.image_size == 0 ||
      begin.packet_data_bytes == 0 ||
      begin.packet_data_bytes > OTA_DATA_BYTES_PER_FRAME) {
    write_ota_ack(OTA_STATUS_INVALID);
    DBG_PRINTLN("[SPI] rsp OTA_BEGIN => INVALID");
    return;
  }

  abortOtaState();
  clear_spi_result_buffer();
  scannerStatusLedSet(true);

  if (!Update.begin(begin.image_size, U_FLASH)) {
    clearOtaState();
    scannerStatusLedSet(false);
    write_ota_ack(OTA_STATUS_BEGIN_FAILED);
    SCANNER_LOG_PRINTLN("Scanner OTA begin failed");
    return;
  }

  ota_state.active = true;
  ota_state.expected_size = begin.image_size;
  ota_state.received_size = 0;
  ota_state.expected_hash = begin.image_hash;
  ota_state.running_hash = OTA_HASH64_INIT;
  ota_state.next_sequence = 0;
  write_ota_ack(OTA_STATUS_OK);
  SCANNER_LOG_PRINTF("Scanner OTA begin: %lu bytes hash=0x%08lx%08lx\n",
                     (unsigned long)ota_state.expected_size,
                     (unsigned long)(ota_state.expected_hash >> 32),
                     (unsigned long)(ota_state.expected_hash & 0xffffffffUL));
}

static void handleCmdOtaData() {
  OtaDataCommand data = {};
  memcpy(&data, rx_buf, sizeof(data));

  if (!ota_state.active) {
    otaLogAck("data-not-active", OTA_STATUS_NOT_ACTIVE);
    write_ota_ack(OTA_STATUS_NOT_ACTIVE);
    return;
  }
  if (data.length == 0 || data.length > OTA_DATA_BYTES_PER_FRAME ||
      data.offset > ota_state.expected_size ||
      data.length > (ota_state.expected_size - data.offset)) {
#if SCANNER_OTA_DEBUG_LOG
    SCANNER_LOG_PRINTF("Scanner OTA invalid data: seq=%u expected_seq=%u offset=%lu received=%lu len=%u size=%lu\n",
                       (unsigned)data.sequence,
                       (unsigned)ota_state.next_sequence,
                       (unsigned long)data.offset,
                       (unsigned long)ota_state.received_size,
                       (unsigned)data.length,
                       (unsigned long)ota_state.expected_size);
#endif
    otaLogAck("data-invalid", OTA_STATUS_INVALID);
    write_ota_ack(OTA_STATUS_INVALID);
    return;
  }
  if (data.sequence != ota_state.next_sequence || data.offset != ota_state.received_size) {
#if SCANNER_OTA_DEBUG_LOG
    SCANNER_LOG_PRINTF("Scanner OTA sequence mismatch: seq=%u expected_seq=%u offset=%lu received=%lu len=%u\n",
                       (unsigned)data.sequence,
                       (unsigned)ota_state.next_sequence,
                       (unsigned long)data.offset,
                       (unsigned long)ota_state.received_size,
                       (unsigned)data.length);
#endif
    otaLogAck("data-sequence", OTA_STATUS_SEQUENCE_MISMATCH,
              static_cast<uint32_t>(ota_state.next_sequence));
    write_ota_ack(OTA_STATUS_SEQUENCE_MISMATCH,
                  static_cast<uint32_t>(ota_state.next_sequence));
    return;
  }

  const uint32_t actual_crc = otaCrc32(data.data, data.length);
  if (actual_crc != data.crc32) {
#if SCANNER_OTA_DEBUG_LOG
    SCANNER_LOG_PRINTF("Scanner OTA CRC mismatch: seq=%u offset=%lu len=%u rx=0x%08lx actual=0x%08lx\n",
                       (unsigned)data.sequence,
                       (unsigned long)data.offset,
                       (unsigned)data.length,
                       (unsigned long)data.crc32,
                       (unsigned long)actual_crc);
#endif
    otaLogAck("data-crc", OTA_STATUS_CRC_MISMATCH, actual_crc);
    write_ota_ack(OTA_STATUS_CRC_MISMATCH, actual_crc);
    return;
  }

  const bool log_packet = otaShouldLogPacket(data.offset, data.length);
  if (log_packet) {
    SCANNER_LOG_PRINTF("Scanner OTA write begin: seq=%u offset=%lu len=%u sector_off=%lu crc=0x%08lx received=%lu\n",
                       (unsigned)data.sequence,
                       (unsigned long)data.offset,
                       (unsigned)data.length,
                       (unsigned long)(data.offset % OTA_DEBUG_SECTOR_BYTES),
                       (unsigned long)data.crc32,
                       (unsigned long)ota_state.received_size);
#if SCANNER_SERIAL_LOG
    Serial.flush();
#endif
  }

  const uint32_t write_start_ms = millis();
  const size_t written = Update.write(data.data, data.length);
  const uint32_t write_elapsed_ms = millis() - write_start_ms;
  if (SCANNER_OTA_DEBUG_LOG &&
      (log_packet || write_elapsed_ms >= OTA_DEBUG_SLOW_WRITE_MS)) {
    SCANNER_LOG_PRINTF("Scanner OTA write end: seq=%u offset=%lu len=%u written=%u elapsed=%lums\n",
                       (unsigned)data.sequence,
                       (unsigned long)data.offset,
                       (unsigned)data.length,
                       (unsigned)written,
                       (unsigned long)write_elapsed_ms);
  }

  if (written != data.length) {
    Update.abort();
    clearOtaState();
    scannerStatusLedSet(false);
    otaLogAck("data-write-failed", OTA_STATUS_WRITE_FAILED,
              static_cast<uint32_t>(written));
    write_ota_ack(OTA_STATUS_WRITE_FAILED, static_cast<uint32_t>(written));
    SCANNER_LOG_PRINTLN("Scanner OTA flash write failed");
    return;
  }

  ota_state.running_hash = otaHash64Update(ota_state.running_hash,
                                           data.data,
                                           data.length);
  ota_state.received_size += data.length;
  ota_state.next_sequence++;
  if (log_packet) {
    SCANNER_LOG_PRINTF("Scanner OTA data accepted: new_offset=%lu next_seq=%u hash=0x%08lx%08lx\n",
                       (unsigned long)ota_state.received_size,
                       (unsigned)ota_state.next_sequence,
                       (unsigned long)(ota_state.running_hash >> 32),
                       (unsigned long)(ota_state.running_hash & 0xffffffffUL));
  }
  write_ota_ack(OTA_STATUS_OK);
}

static void handleCmdOtaEnd() {
  OtaEndCommand end = {};
  memcpy(&end, rx_buf, sizeof(end));

  if (!ota_state.active) {
    write_ota_ack(OTA_STATUS_NOT_ACTIVE);
    return;
  }
  if (end.image_size != ota_state.expected_size ||
      end.image_hash != ota_state.expected_hash ||
      ota_state.received_size != ota_state.expected_size) {
    Update.abort();
    clearOtaState();
    scannerStatusLedSet(false);
    write_ota_ack(OTA_STATUS_SIZE_MISMATCH);
    SCANNER_LOG_PRINTLN("Scanner OTA size mismatch; aborted");
    return;
  }
  if (ota_state.running_hash != ota_state.expected_hash) {
    const uint64_t actual_hash = ota_state.running_hash;
    Update.abort();
    clearOtaState();
    scannerStatusLedSet(false);
    write_ota_ack(OTA_STATUS_HASH_MISMATCH,
                  static_cast<uint32_t>(actual_hash & 0xffffffffUL));
    SCANNER_LOG_PRINTF("Scanner OTA hash mismatch: actual=0x%08lx%08lx expected=0x%08lx%08lx\n",
                       (unsigned long)(actual_hash >> 32),
                       (unsigned long)(actual_hash & 0xffffffffUL),
                       (unsigned long)(end.image_hash >> 32),
                       (unsigned long)(end.image_hash & 0xffffffffUL));
    return;
  }

  const uint64_t final_hash = ota_state.running_hash;
  if (!Update.end(true)) {
    Update.abort();
    clearOtaState();
    scannerStatusLedSet(false);
    write_ota_ack(OTA_STATUS_END_FAILED);
    SCANNER_LOG_PRINTLN("Scanner OTA commit failed");
    return;
  }

  write_ota_ack(OTA_STATUS_OK);
  SCANNER_LOG_PRINTF("Scanner OTA committed: %lu bytes hash=0x%08lx%08lx; restarting\n",
                     (unsigned long)ota_state.received_size,
                     (unsigned long)(final_hash >> 32),
                     (unsigned long)(final_hash & 0xffffffffUL));
  clearOtaState();
  ota_restart_pending = true;
}

static void handleCmdOtaAbort() {
#if SCANNER_OTA_DEBUG_LOG
  SCANNER_LOG_PRINTF("Scanner OTA abort summary before reset: offset=%lu next_seq=%u data_frames=%lu other=%lu nops=%lu\n",
                     (unsigned long)ota_state.received_size,
                     (unsigned)ota_state.next_sequence,
                     (unsigned long)ota_diag_data_frames,
                     (unsigned long)ota_diag_other_frames,
                     (unsigned long)ota_diag_nop_polls);
#endif
  abortOtaState();
  scannerStatusLedSet(false);
  write_ota_ack(OTA_STATUS_OK);
  SCANNER_LOG_PRINTLN("Scanner OTA aborted by controller");
}

// ---------- Command handling ----------
// This fills tx_buf with the *next* response (to be clocked out on the next transaction).
static void handle_command(uint8_t cmd_byte) {
  // Clear tx_buf each time so short responses don't leak old data.
  memset(tx_buf, 0, FRAME_SIZE);
  // Tag the frame with the command this response corresponds to.
  tx_buf[FRAME_TAG_INDEX] = cmd_byte;
  if (cmd_byte != CMD_NOP) {
    clearOtaPendingAck();
  }

  const SpiCommand cmd = static_cast<SpiCommand>(cmd_byte);
  DBG_PRINTF("[SPI] cmd=0x%02X (%s) rx[1..3]=%u,%u,%u\n",
             cmd_byte, cmd_to_str(cmd_byte), rx_buf[1], rx_buf[2], rx_buf[3]);

  switch (cmd) {
    case CMD_NOP: handleCmdNop(); break;
    case CMD_ID: handleCmdId(); break;
    case CMD_SCAN: handleCmdScan(); break;
    case CMD_RESULT_COUNT: handleCmdResultCount(); break;
    case CMD_RESULT_GET: handleCmdResultGet(); break;
    case CMD_DEDUPE_RESET: handleCmdDedupeReset(); break;
    case CMD_OTA_BEGIN: handleCmdOtaBegin(); break;
    case CMD_OTA_DATA: handleCmdOtaData(); break;
    case CMD_OTA_END: handleCmdOtaEnd(); break;
    case CMD_OTA_ABORT: handleCmdOtaAbort(); break;

    default:
      // Unknown command: return 0
      DBG_PRINTF("[SPI] rsp UNKNOWN cmd=0x%02X\n", cmd_byte);
      break;
  }
}

void setup() {
#if SCANNER_SERIAL_LOG || DEBUG_SPI_PROTOCOL
  Serial.begin(115200);
#endif
  boot_reset_reason = esp_reset_reason();
  boot_diag_next_ms = 0;
  boot_diag_emits = 0;
  if (SCANNER_STATUS_LED_ENABLED && ((int)SCANNER_STATUS_LED_PIN >= 0)) {
    pinMode(SCANNER_STATUS_LED_PIN, OUTPUT);
  }
  scannerStatusLedSet(true);
  // Keep reboot-to-SPI-ready latency low so the Pico can recover quickly.
  delay(100);

  SCANNER_LOG_PRINTLN("ESP32-C5 Scanner (Arduino WiFi + SPI slave, command protocol)");

  if (!scannerRadioInit()) {
    SCANNER_LOG_PRINTLN("scannerRadioInit failed");
    while (1) delay(1000);
  }
  delay(100);

  if (!scannerSpiSlaveInit(PIN_SCK, PIN_MOSI, PIN_MISO, PIN_CS,
                           FRAME_SIZE, SPI_TRANSACTION_QUEUE_SIZE)) {
    SCANNER_LOG_PRINTLN("scannerSpiSlaveInit failed");
    while (1) delay(1000);
  }

  memset(rx_buf, 0, FRAME_SIZE);
  memset(tx_buf, 0, FRAME_SIZE);
  wifiDedupeTableInit(&scan_dedupe_table,
                      scan_dedupe_slots, WIFI_DEDUPE_TABLE_SIZE,
                      scan_dedupe_fifo, WIFI_DEDUPE_TABLE_CAPACITY);
  resetDedupeAndBuffers();

  // Boot complete: scanner is ready/idle.
  scannerStatusLedSet(false);
  SCANNER_LOG_PRINTLN("SPI slave ready");
}

void loop() {
  emitBootDiagnosticsIfDue();
  // Always progress scan state even if the master is quiet.
  update_scan_state();

  // Ensure rx_buf doesn't contain stale data (not strictly required, but helps debugging).
  memset(rx_buf, 0, FRAME_SIZE);

  // Wait for a SPI transaction; timeout keeps async scan polling progressing.
  const int ret = scannerSpiSlaveTransfer(tx_buf, rx_buf, FRAME_SIZE, SPI_RX_TIMEOUT_MS);

  if (ret == SCANNER_SPI_TRANSFER_TIMEOUT) {
    // No SPI activity this slice; keep looping to update scan state.
    return;
  }
  if (ret != SCANNER_SPI_TRANSFER_OK) {
    // SPI error; keep going
    DBG_PRINTF("[SPI] spi_slave_transmit error: %d\n", ret);
    return;
  }

  if (ota_restart_pending) {
    delay(OTA_RESTART_DELAY_MS);
    ESP.restart();
  }

  // Transaction completed: rx_buf now contains the command from the master.
  const uint8_t cmd = rx_buf[0];
  otaLogRawFrame(cmd);

  // Prepare response for the NEXT transaction (pipelined slave semantics).
  // Keep handlers short here: doing extra scan/Wi-Fi work in this critical path
  // delays re-queueing the next slave transaction and can make the master miss
  // the response poll entirely.
  handle_command(cmd);
}
