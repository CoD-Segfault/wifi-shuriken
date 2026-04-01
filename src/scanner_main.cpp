#include <Arduino.h>
#include "spi_protocol_shared.h"
#include "wifi_dedupe.h"
#include "wifi_result_utils.h"
#include "scanner_platform_esp32_radio.h"
#include "scanner_platform_esp32_spi.h"
#include "ScannerConfiguration.h"
#include <esp_system.h>

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
  // Return current scanner status/count semantics as a convenience.
  const int8_t c = spi_status;
  write_status_response(c);
  DBG_PRINTF("[SPI] rsp NOP => count=%d\n", c);
}

static void handleCmdId() {
  DeviceIdReply id = {};
  id.proto_version = PROTO_VERSION;
  id.scanner_type  = SCANNER_TYPE_ESP32_C5;
  id.capabilities  = (CAP_BAND_24GHZ | CAP_BAND_5GHZ);
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

// ---------- Command handling ----------
// This fills tx_buf with the *next* response (to be clocked out on the next transaction).
static void handle_command(uint8_t cmd_byte) {
  // Clear tx_buf each time so short responses don't leak old data.
  memset(tx_buf, 0, FRAME_SIZE);
  // Tag the frame with the command this response corresponds to.
  tx_buf[FRAME_TAG_INDEX] = cmd_byte;

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

  // Transaction completed: rx_buf now contains the command from the master.
  const uint8_t cmd = rx_buf[0];

  // Prepare response for the NEXT transaction (pipelined slave semantics).
  // Keep handlers short here: doing extra scan/Wi-Fi work in this critical path
  // delays re-queueing the next slave transaction and can make the master miss
  // the response poll entirely.
  handle_command(cmd);
}
