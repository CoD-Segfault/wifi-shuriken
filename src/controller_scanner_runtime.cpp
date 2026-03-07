#include "controller_scanner_runtime.h"

#include <Arduino.h>
#include <SPI.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "hardware/gpio.h"
#include "pico_logging.h"
#include "spi_protocol_shared.h"
#include "wifi_result_utils.h"
#include "Configuration.h"

// This module owns the controller-side scanner transport and scheduler that run
// on core1: SPI framing, slot identification, shared sweep progression, and
// controller-wide dedupe before results are handed back to core0.

using QueuedScanResult = pico_logging::QueuedScanResult;

// Transport tuning constants for the Pico<->scanner SPI protocol.
static constexpr size_t SCANNER_FRAME_SIZE = SPI_FRAME_SIZE;
static constexpr uint32_t SCANNER_SPI_HZ = SCANNER_SPI_CLOCK;
static constexpr size_t SCANNER_FRAME_TAG_INDEX = SPI_FRAME_TAG_INDEX;
static constexpr uint8_t SCANNER_SCAN_RESPONSE_PULLS = 24;
// Keep RESULT_GET pulls relatively high; result packets are larger and may need extra drains.
static constexpr uint8_t SCANNER_RESULT_RESPONSE_PULLS = 32;
// Bound per-slot drain work so one scanner cannot monopolize loop time.
static constexpr uint8_t SCANNER_RESULT_DRAIN_BUDGET = 8;
// Fast-fail status polling so one missing slot does not stall the round-robin scheduler.
static constexpr uint8_t SCANNER_RESULT_COUNT_RESPONSE_PULLS = 8;
static constexpr uint8_t SCANNER_SCAN_CMD_RETRIES = 1;
static constexpr uint8_t SCANNER_QUERY_CMD_RETRIES = 2;
static constexpr uint8_t SCANNER_RESULT_COUNT_CMD_RETRIES = 1;
static constexpr uint8_t SCANNER_RESULT_GET_CMD_RETRIES = 1;
// Keep a short guard gap after CMD_SCAN before the first response pull.
static constexpr uint16_t SCANNER_SCAN_FIRST_PULL_US = 3000;
static constexpr uint8_t SCANNER_SCAN_BUSY_CONFIRM_POLLS = 1;
static constexpr uint16_t SCANNER_SCAN_BUSY_CONFIRM_DELAY_US = 2500;
// Scanner transport hardware config (mapped from legacy config macro names).
static constexpr int SCANNER_PIN_SCK = ESP_PIN_SCK;
static constexpr int SCANNER_PIN_MOSI = ESP_PIN_MOSI;
static constexpr int SCANNER_PIN_MISO = ESP_PIN_MISO;
#if defined(ESP_PIN_CS)
static constexpr int SCANNER_PIN_CS = ESP_PIN_CS;
#endif
static constexpr uint32_t SCANNER_INTERFRAME_US = ESP_INTERFRAME_US;
static constexpr uint8_t SCANNER_DEVICE_TYPE = SCANNER_TYPE_ESP32_C5;
// Local transport status (not on-wire protocol).
static constexpr int8_t SCANNER_STATUS_TRANSPORT_TIMEOUT = -4;
static constexpr uint16_t SCANNER_SLOT_REPROBE_MS = 2000;
static constexpr uint8_t SCANNER_SLOT_TIMEOUT_REPROBE_THRESHOLD = 8;
static constexpr uint16_t SCANNER_SLOT_TIMEOUT_BACKOFF_BASE_MS = 10;
static constexpr uint16_t SCANNER_SLOT_TIMEOUT_BACKOFF_MAX_MS = 200;
static constexpr uint16_t SCANNER_DEDUPE_RESET_RETRY_MS = 100;
#if SCANNER_USE_SHIFTREG_CS
static constexpr uint8_t SCANNER_SLOT_COUNT = SCANNER_SHIFTREG_OUTPUTS;
#else
static constexpr uint8_t SCANNER_SLOT_COUNT = 1;
#endif

struct ScannerSlotState {
  // Cached identity state for this physical slot.
  bool id_valid;
  // One-shot log suppressors while the slot remains in same state.
  bool miss_logged;
  bool unsupported_logged;
  DeviceIdReply id;
  // Re-probe scheduling so absent slots do not get hammered every loop.
  uint32_t next_probe_ms;
  // Backoff for temporary transport misses while slot remains identified.
  uint32_t next_service_ms;
  // One-time per-identification scanner dedupe reset handshake.
  bool dedupe_reset_done;
  uint32_t next_dedupe_reset_ms;
  // Transport timeout streak for targeted stale-frame draining.
  uint8_t transport_timeouts;
};

static const uint8_t CHANNELS_24G[] = {
  1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14
};

static const uint8_t CHANNELS_5G[] = {
  36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, 149, 153, 157, 161, 165
};

// Persistent core1 runtime state. controller_main.cpp owns the shared queues and
// counters, but the scanner runtime owns the active slot machine and sweep state.
static ControllerScannerRuntimeContext scanner_runtime_context = {};
static uint8_t scanner_cs_shiftreg_state = 0xFF;
static uint8_t scanner_active_slot = SCANNER_INITIAL_SLOT;
#if SCANNER_USE_SHIFTREG_CS
static bool scanner_shiftreg_outputs_enabled = false;
#endif
static ScannerSlotState scanner_slot_state[SCANNER_SLOT_COUNT] = {};
// A single global sweep order is shared across all scanner slots.
static size_t scanner_sweep_band_index = 0;
static size_t scanner_sweep_channel_index = 0;
static bool scanner_sweep_timing_active = false;
static uint32_t scanner_sweep_started_ms = 0;
static uint8_t scanner_slot = SCANNER_INITIAL_SLOT;

static void scannerSerialQueueTry(const char* text) {
  if (scanner_runtime_context.serial_queue_try != nullptr) {
    scanner_runtime_context.serial_queue_try(text);
  }
}

static void scannerSerialPrintlnTry(const char* s) {
#if !CORE1_SERIAL_LOG
  (void)s;
  return;
#else
  char buf[192];
  snprintf(buf, sizeof(buf), "%s\n", s);
  scannerSerialQueueTry(buf);
#endif
}

static void scannerSerialPrintfTry(const char* fmt, ...) {
#if !CORE1_SERIAL_LOG
  (void)fmt;
  return;
#else
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  scannerSerialQueueTry(buf);
#endif
}

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

// Sweep timing starts when the shared scheduler wraps to channel 1 on 2.4GHz.
static inline void sweepTimingNoteStart(size_t band_index,
                                        size_t channel_index,
                                        bool& timing_active,
                                        uint32_t& started_ms) {
  if (!timing_active && band_index == 0 && channel_index == 0) {
    started_ms = millis();
    timing_active = true;
  }
}

static inline void scannerSetActiveSlot(uint8_t slot) {
#if SCANNER_USE_SHIFTREG_CS
  // The active slot selects which 74HC595 output will assert CS for the next
  // transfer. With direct CS wiring there is only one logical slot.
  scanner_active_slot = (uint8_t)(slot % SCANNER_SLOT_COUNT);
#else
  (void)slot;
  scanner_active_slot = 0;
#endif
}

#if SCANNER_USE_SHIFTREG_CS
static inline void scannerShiftRegSetOutputsEnabled(bool enabled) {
  scanner_shiftreg_outputs_enabled = enabled;
  // 74HC595 OE is active-low.
  digitalWrite(SCANNER_SHIFTREG_OE_PIN, enabled ? LOW : HIGH);
}

static inline void scannerShiftRegWriteCsState(uint8_t state, bool force) {
  // Updating the shift register itself uses SPI1, so temporarily disable the
  // scanner outputs to avoid clocking garbage into a selected slave.
  if (!force && scanner_cs_shiftreg_state == state) {
    return;
  }
  const bool restore_outputs = scanner_shiftreg_outputs_enabled;
  if (restore_outputs) {
    // Prevent SPI clocks for shift-register updates from reaching any selected scanner.
    scannerShiftRegSetOutputsEnabled(false);
  }
  scanner_cs_shiftreg_state = state;
  SPI1.beginTransaction(SPISettings(SCANNER_SHIFTREG_SPI_HZ, MSBFIRST, SPI_MODE0));
  SPI1.transfer(scanner_cs_shiftreg_state);
  SPI1.endTransaction();
  digitalWrite(SCANNER_SHIFTREG_LATCH_PIN, HIGH);
  digitalWrite(SCANNER_SHIFTREG_LATCH_PIN, LOW);
  if (restore_outputs) {
    scannerShiftRegSetOutputsEnabled(true);
  }
}
#endif

static inline void advanceSweepChannelAndLog(size_t& band_index,
                                             size_t& channel_index,
                                             bool& timing_active,
                                             uint32_t& started_ms) {
  // The scheduler is global across all slots, so sweep completion is only
  // recognized when the shared channel cursor wraps from the end of 5GHz back
  // to the start of 2.4GHz.
  if (advanceSweepChannel(band_index, channel_index)) {
    if (timing_active) {
      const uint32_t elapsed_ms = millis() - started_ms;
      *scanner_runtime_context.last_full_sweep_ms = elapsed_ms;
      (*scanner_runtime_context.sweep_cycles_completed)++;
      scannerSerialPrintfTry("Completed full 2.4G + 5G sweep in %lums\n",
                             (unsigned long)elapsed_ms);
      timing_active = false;
      return;
    }
    scannerSerialPrintlnTry("Completed full 2.4G + 5G sweep");
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

  // The slave protocol is fixed-length and full-duplex, so every command and
  // poll is exactly one 64-byte transfer.
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
  // Some scanner boards need a short turnaround gap between command and poll
  // frames to let the response buffer settle.
  if (SCANNER_INTERFRAME_US > 0) {
    delayMicroseconds(SCANNER_INTERFRAME_US);
  }
}

// Issue command frame, then pull NOP frames until response tag matches expected cmd.
static bool scannerCommandWithResponse(const uint8_t* cmd_frame,
                                       uint8_t expected_cmd_tag,
                                       uint8_t* response_frame,
                                       int max_pulls = 8,
                                       uint32_t first_pull_wait_us = SCANNER_INTERFRAME_US) {
  // Most commands are acknowledged asynchronously: send once, then keep pulling
  // NOP frames until the scanner echoes the expected command tag.
  uint8_t rx[SCANNER_FRAME_SIZE] = {0};
  uint8_t nop[SCANNER_FRAME_SIZE] = {0};
  nop[0] = CMD_NOP;
  scannerTransferFrame(cmd_frame, rx);
  if (first_pull_wait_us > 0) {
    delayMicroseconds(first_pull_wait_us);
  }

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
  // Drain leftover reply frames when we switch slots or recover from malformed
  // traffic so a stale response is less likely to be mis-read as the next ACK.
  uint8_t nop[SCANNER_FRAME_SIZE] = {0};
  uint8_t rx[SCANNER_FRAME_SIZE] = {0};
  nop[0] = CMD_NOP;
  for (int i = 0; i < pulls; i++) {
    scannerTransferFrame(nop, rx);
    scannerInterframeWait();
  }
}

void controllerScannerRuntimeInitBus() {
  // Bring the scanner bus to a safe idle state before core1 starts polling.
#if SCANNER_USE_SHIFTREG_CS
  pinMode(SCANNER_SHIFTREG_LATCH_PIN, OUTPUT);
  pinMode(SCANNER_SHIFTREG_OE_PIN, OUTPUT);
  digitalWrite(SCANNER_SHIFTREG_LATCH_PIN, LOW);
  // Active-low OE: keep outputs disabled while shift register state is initialized.
  scannerShiftRegSetOutputsEnabled(false);
#else
  digitalWrite(SCANNER_PIN_CS, HIGH);
  pinMode(SCANNER_PIN_CS, OUTPUT);
#endif
#if SCANNER_MISO_PULLUP
  gpio_pull_up(SCANNER_PIN_MISO);
#endif
  SPI1.setSCK(SCANNER_PIN_SCK);
  SPI1.setTX(SCANNER_PIN_MOSI);
  SPI1.setRX(SCANNER_PIN_MISO);
  SPI1.begin();
#if SCANNER_MISO_PULLUP
  gpio_pull_up(SCANNER_PIN_MISO);
#endif
#if SCANNER_USE_SHIFTREG_CS
  scannerSetActiveSlot(SCANNER_INITIAL_SLOT);
  scannerShiftRegWriteCsState(0xFFu, true);
  // Keep outputs enabled in idle once all scanner CS lines are safely deasserted.
  scannerShiftRegSetOutputsEnabled(true);
#endif
}

static bool scannerGetDeviceId(DeviceIdReply& out) {
  uint8_t tx[SCANNER_FRAME_SIZE] = {0};
  uint8_t rx[SCANNER_FRAME_SIZE] = {0};

  // Empty or still-booting slots can return garbage; probe a few times before
  // deciding the slot is absent.
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
  // Slot identity is cached until transport failures force a re-probe. This
  // keeps empty slots from consuming controller bandwidth every pass.
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
    state.next_service_ms = 0;
    state.dedupe_reset_done = false;
    state.next_dedupe_reset_ms = 0;
    state.transport_timeouts = 0;
    state.next_probe_ms = now + SCANNER_SLOT_REPROBE_MS;
    if (!state.miss_logged) {
      scannerSerialPrintfTry("S%u no valid scanner ID; skipping slot\n", (unsigned)slot);
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
    state.next_service_ms = 0;
    state.dedupe_reset_done = false;
    state.next_dedupe_reset_ms = 0;
    state.transport_timeouts = 0;
    state.next_probe_ms = now + SCANNER_SLOT_REPROBE_MS;
    if (!state.unsupported_logged) {
      scannerSerialPrintfTry("S%u unsupported SPI device: proto=%u type=%u caps=0x%02X max=%u; skipping slot\n",
                             (unsigned)slot,
                             id.proto_version, id.scanner_type, id.capabilities, id.max_results);
      state.unsupported_logged = true;
    }
    return false;
  }

  state.id_valid = true;
  state.transport_timeouts = 0;
  state.next_service_ms = 0;
  state.dedupe_reset_done = false;
  state.next_dedupe_reset_ms = 0;
  state.next_probe_ms = 0;
  state.unsupported_logged = false;
  scannerSerialPrintfTry("S%u scanner ready: proto=%u type=%u caps=0x%02X max=%u\n",
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
  // SCAN needs a longer first wait than lightweight status commands because
  // the scanner must hand work to the Wi-Fi driver before an ACK is ready.
  uint32_t first_pull_wait_us = SCANNER_INTERFRAME_US;
  if (cmd == CMD_SCAN) {
    first_pull_wait_us = SCANNER_SCAN_FIRST_PULL_US;
  }

  for (uint8_t attempt = 0; attempt < retries; attempt++) {
    uint8_t tx[SCANNER_FRAME_SIZE] = {0};
    uint8_t rx[SCANNER_FRAME_SIZE] = {0};
    tx[0] = cmd;
    tx[1] = arg1;
    tx[2] = arg2;
    if (scannerCommandWithResponse(tx, cmd, rx, response_pulls, first_pull_wait_us)) {
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
  // RESULT_GET is a two-phase exchange: request the next packet, then keep
  // pulling until the scanner returns a tagged result payload.
  uint8_t tx[SCANNER_FRAME_SIZE] = {0};
  uint8_t rx[SCANNER_FRAME_SIZE] = {0};
  uint8_t nop[SCANNER_FRAME_SIZE] = {0};
  tx[0] = CMD_RESULT_GET;
  nop[0] = CMD_NOP;

  for (int attempt = 0; attempt < SCANNER_RESULT_GET_CMD_RETRIES; attempt++) {
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

static void startScanForCurrentChannel(uint8_t slot,
                                       size_t& sweep_band_index,
                                       size_t& sweep_channel_index,
                                       bool& sweep_timing_active,
                                       uint32_t& sweep_started_ms) {
  // Every slot draws work from the same shared channel cursor. A slot that
  // successfully starts a scan advances the global sweep for the next slot.
  const uint8_t band = (sweep_band_index == 0) ? 2 : 5;
  const uint8_t channel = (sweep_band_index == 0)
    ? CHANNELS_24G[sweep_channel_index]
    : CHANNELS_5G[sweep_channel_index];
  sweepTimingNoteStart(sweep_band_index,
                       sweep_channel_index,
                       sweep_timing_active,
                       sweep_started_ms);

  const int8_t status = scannerQueryStatusWithRetry(CMD_SCAN,
                                                    band,
                                                    channel,
                                                    SCANNER_SCAN_RESPONSE_PULLS,
                                                    SCANNER_SCAN_CMD_RETRIES,
                                                    SCANNER_STATUS_START_FAILED,
                                                    SCANNER_STATUS_OK);
  if (status == SCANNER_STATUS_OK) {
#if SCAN_LOG_NORMAL_STARTS
    scannerSerialPrintfTry("S%u Scan started (band=%u ch=%u)\n",
                           (unsigned)slot, band, channel);
#endif
    advanceSweepChannelAndLog(sweep_band_index,
                              sweep_channel_index,
                              sweep_timing_active,
                              sweep_started_ms);
    return;
  }
  if (status == SCANNER_STATUS_BUSY) {
    // If RESULT_COUNT said idle but SCAN says busy, the prior start likely latched
    // and we missed/garbled the ACK. Advance to avoid re-requesting same channel.
    scannerSerialPrintfTry("S%u Scan already active (band=%u ch=%u); assuming prior start latched\n",
                           (unsigned)slot, band, channel);
    advanceSweepChannelAndLog(sweep_band_index,
                              sweep_channel_index,
                              sweep_timing_active,
                              sweep_started_ms);
    return;
  }
  if (status == SCANNER_STATUS_TRANSPORT_TIMEOUT) {
    // If SCAN ACK was lost but scanner is now busy, treat scan as started.
    for (uint8_t confirm = 0; confirm < SCANNER_SCAN_BUSY_CONFIRM_POLLS; confirm++) {
      const int8_t c = scannerQueryStatusWithRetry(CMD_RESULT_COUNT,
                                                   0,
                                                   0,
                                                   SCANNER_RESULT_COUNT_RESPONSE_PULLS,
                                                   SCANNER_RESULT_COUNT_CMD_RETRIES,
                                                   SCANNER_STATUS_BUSY,
                                                   static_cast<int8_t>(PROTO_MAX_RESULTS));
      if (c == SCANNER_STATUS_BUSY) {
        scannerSerialPrintfTry("S%u Scan started (band=%u ch=%u) via BUSY confirm\n",
                               (unsigned)slot, band, channel);
        advanceSweepChannelAndLog(sweep_band_index,
                                  sweep_channel_index,
                                  sweep_timing_active,
                                  sweep_started_ms);
        return;
      }
      if (SCANNER_SCAN_BUSY_CONFIRM_DELAY_US > 0) {
        delayMicroseconds(SCANNER_SCAN_BUSY_CONFIRM_DELAY_US);
      }
    }
    // Timeout with no BUSY confirm: move on to avoid getting pinned on one channel.
    scannerSerialPrintfTry("S%u SCAN timeout (band=%u ch=%u); moving on\n",
                           (unsigned)slot, band, channel);
    advanceSweepChannelAndLog(sweep_band_index,
                              sweep_channel_index,
                              sweep_timing_active,
                              sweep_started_ms);
    return;
  }

  // Unexpected scan status: move on to the next channel.
  scannerSerialPrintfTry("S%u SCAN unexpected status=%d (band=%u ch=%u); moving on\n",
                         (unsigned)slot, status, band, channel);
  advanceSweepChannelAndLog(sweep_band_index,
                            sweep_channel_index,
                            sweep_timing_active,
                            sweep_started_ms);
}

// Per-slot scan pump with a shared channel scheduler:
// 1) Poll RESULT_COUNT.
// 2) If busy -> return.
// 3) If zero -> request scan on current channel.
// 4) If >0 -> drain that many packets and enqueue uniques.
static void processScannerSlot(uint8_t slot,
                               ScannerSlotState& slot_state,
                               size_t& sweep_band_index,
                               size_t& sweep_channel_index,
                               bool& sweep_timing_active,
                               uint32_t& sweep_started_ms) {
  // Each service pass handles one slot only. This keeps the round-robin fair
  // across populated scanners even when one slot has a large result backlog.
  const uint32_t now = millis();
  if ((int32_t)(now - slot_state.next_service_ms) < 0) {
    return;
  }

  scannerSetActiveSlot(slot);

  if (!slot_state.dedupe_reset_done) {
    // Reset scanner-local dedupe once per identification cycle so the slave
    // starts each attach with a clean buffer model.
    if ((int32_t)(now - slot_state.next_dedupe_reset_ms) < 0) {
      return;
    }

    const int8_t reset_status = scannerQueryStatusWithRetry(CMD_DEDUPE_RESET,
                                                            0,
                                                            0,
                                                            SCANNER_RESULT_COUNT_RESPONSE_PULLS,
                                                            SCANNER_QUERY_CMD_RETRIES,
                                                            SCANNER_STATUS_BUSY,
                                                            SCANNER_STATUS_OK);
    if (reset_status == SCANNER_STATUS_OK) {
      slot_state.dedupe_reset_done = true;
      slot_state.next_dedupe_reset_ms = 0;
      slot_state.transport_timeouts = 0;
      slot_state.next_service_ms = 0;
      scannerSerialPrintfTry("S%u scanner dedupe reset\n", (unsigned)slot);
      scannerDrainStaleFrames(1);
    } else {
      slot_state.next_dedupe_reset_ms = now + SCANNER_DEDUPE_RESET_RETRY_MS;
      if (reset_status == SCANNER_STATUS_TRANSPORT_TIMEOUT &&
          slot_state.transport_timeouts < 0x0F) {
        slot_state.transport_timeouts++;
      }
      return;
    }
  }

  const int8_t count = scannerQueryStatusWithRetry(CMD_RESULT_COUNT,
                                                   0,
                                                   0,
                                                   SCANNER_RESULT_COUNT_RESPONSE_PULLS,
                                                   SCANNER_RESULT_COUNT_CMD_RETRIES,
                                                   SCANNER_STATUS_BUSY,
                                                   static_cast<int8_t>(PROTO_MAX_RESULTS));

  // Active scan on slave; poll this slot again next round.
  if (count == SCANNER_STATUS_BUSY) {
    slot_state.transport_timeouts = 0;
    slot_state.next_service_ms = 0;
    return;
  }

  if (count == SCANNER_STATUS_TRANSPORT_TIMEOUT) {
    if (slot_state.transport_timeouts < 0x0F) {
      slot_state.transport_timeouts++;
    }
    const uint8_t streak = slot_state.transport_timeouts;
    const uint8_t shift = (streak <= 1) ? 0 : ((streak >= 5) ? 4 : (uint8_t)(streak - 1));
    uint32_t backoff_ms = (uint32_t)SCANNER_SLOT_TIMEOUT_BACKOFF_BASE_MS << shift;
    if (backoff_ms > SCANNER_SLOT_TIMEOUT_BACKOFF_MAX_MS) {
      backoff_ms = SCANNER_SLOT_TIMEOUT_BACKOFF_MAX_MS;
    }
    slot_state.next_service_ms = now + backoff_ms;

    if ((streak == SCANNER_SLOT_TIMEOUT_REPROBE_THRESHOLD) ||
        ((streak % SCANNER_SLOT_TIMEOUT_REPROBE_THRESHOLD) == 0)) {
      scannerSerialPrintfTry("S%u RESULT_COUNT timeout x%u; backing off %lums\n",
                             (unsigned)slot,
                             (unsigned)streak,
                             (unsigned long)backoff_ms);
    }
    return;
  }
  slot_state.transport_timeouts = 0;
  slot_state.next_service_ms = 0;

  if (count < 0) {
    // Unexpected status: advance global scheduler so one bad slot does not stall coverage.
    scannerSerialPrintfTry("S%u RESULT_COUNT unexpected=%d; moving on\n",
                           (unsigned)slot, count);
    advanceSweepChannelAndLog(sweep_band_index,
                              sweep_channel_index,
                              sweep_timing_active,
                              sweep_started_ms);
    return;
  }

  // No results pending: request next scan from the global scheduler.
  if (count == 0) {
    startScanForCurrentChannel(slot,
                               sweep_band_index,
                               sweep_channel_index,
                               sweep_timing_active,
                               sweep_started_ms);
    return;
  }

  // Results pending: drain up to reported count for this slot.
  bool scanner_busy_during_drain = false;
  const int drain_budget = (count > (int)SCANNER_RESULT_DRAIN_BUDGET)
    ? (int)SCANNER_RESULT_DRAIN_BUDGET
    : count;
  uint16_t batch_dedupe_hits = 0;
  uint16_t batch_dedupe_logs = 0;
  for (int i = 0; i < drain_budget; i++) {
    WiFiResultPacket pkt = {};
    if (!scannerGetResult(pkt)) {
      if (slot_state.transport_timeouts < 0x0F) {
        slot_state.transport_timeouts++;
      }
      slot_state.next_service_ms = now + SCANNER_SLOT_TIMEOUT_BACKOFF_BASE_MS;
      break;
    }

    if (pkt.result_type == RESULT_WIFI) {
      if (!wifiResultIsValidForDedupe(pkt.result)) {
        const bool zero_bssid = wifiBssidIsZero(pkt.result.bssid);
        const bool ff_bssid = wifiBssidIsBroadcast(pkt.result.bssid);
        scannerSerialPrintfTry("S%u Dropped invalid WIFI result: ch=%u band=%u rssi=%d ssid0=0x%02X bssid_zero=%u bssid_ff=%u\n",
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
      // Controller-side dedupe suppresses duplicates across all scanners, not
      // just within the reporting slot.
      if (!wifiDedupeTableRemember(scanner_runtime_context.master_dedupe_table, &hash)) {
        (*scanner_runtime_context.dedupe_drops)++;
        batch_dedupe_hits++;
#if LOG_DEDUPE_HITS
#if CONTROLLER_LOG_INDIVIDUAL_DEDUPE_HITS
        if (batch_dedupe_logs < LOG_DEDUPE_HITS_MAX_PER_SCAN) {
          char mac[18] = {};
          pico_logging::formatBssid(pkt.result.bssid, mac, sizeof(mac));
          scannerSerialPrintfTry("[MASTER S%u] dedupe hit ssid='%s' bssid=%s ch=%u band=%u rssi=%d\n",
                                 (unsigned)slot,
                                 pkt.result.ssid,
                                 mac,
                                 pkt.result.channel,
                                 pkt.result.band,
                                 pkt.result.rssi);
          batch_dedupe_logs++;
        }
#endif
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
      if (!queue_try_add(scanner_runtime_context.scan_result_queue, &q)) {
        (*scanner_runtime_context.scan_queue_drops)++;
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

    scannerSerialPrintfTry("S%u Unknown result type: 0x%02X; draining stale frames\n",
                           (unsigned)slot, pkt.result_type);
    scannerDrainStaleFrames(2);
    break;
  }

#if LOG_DEDUPE_HITS
  if (batch_dedupe_hits > batch_dedupe_logs) {
    scannerSerialPrintfTry("[MASTER S%u] dedupe hits suppressed: %u (total=%u)\n",
                           (unsigned)slot,
                           (unsigned)(batch_dedupe_hits - batch_dedupe_logs),
                           (unsigned)batch_dedupe_hits);
  }
#endif

  if (scanner_busy_during_drain) {
    return;
  }
}

void controllerScannerRuntimeRun(const ControllerScannerRuntimeContext& context) {
  // core1 lives here forever after startup. controller_main.cpp only builds the
  // context and launches this loop on the second core.
  scanner_runtime_context = context;
  scanner_slot = SCANNER_INITIAL_SLOT;

  scannerSerialPrintlnTry("Core1 SPI loop started");
#if SCANNER_USE_SHIFTREG_CS
  for (uint8_t s = 0; s < SCANNER_SLOT_COUNT; s++) {
    scannerSetActiveSlot(s);
    scannerDrainStaleFrames(1);
  }
  scannerSetActiveSlot(scanner_slot);
#else
  scannerDrainStaleFrames();
#endif

  while (true) {
    // Each iteration services exactly one slot; round-robin when shift-reg CS is enabled.
    scannerSetActiveSlot(scanner_slot);
    if (ensureScannerSlotIdentity(scanner_slot, scanner_slot_state[scanner_slot])) {
      processScannerSlot(scanner_slot,
                         scanner_slot_state[scanner_slot],
                         scanner_sweep_band_index,
                         scanner_sweep_channel_index,
                         scanner_sweep_timing_active,
                         scanner_sweep_started_ms);
    }
#if SCANNER_USE_SHIFTREG_CS
    scanner_slot = (uint8_t)((scanner_slot + 1u) % SCANNER_SLOT_COUNT);
#endif
  }
}
