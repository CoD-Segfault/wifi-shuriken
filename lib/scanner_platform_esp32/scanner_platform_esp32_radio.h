// scanner_platform_esp32_radio.h
#pragma once

#include <stdint.h>
#include "../../include/spi_protocol_shared.h"

// Generic radio scan status contract used by scanner logic.
static constexpr int8_t SCANNER_RADIO_SCAN_RUNNING = -1;
static constexpr int8_t SCANNER_RADIO_SCAN_FAILED = -2;

bool scannerRadioInit();

// Starts async channel scan. Returns SCANNER_RADIO_SCAN_FAILED on failure.
// On success, backend may return SCANNER_RADIO_SCAN_RUNNING or another
// backend-specific success value.
int scannerRadioStartAsyncScan(uint8_t band, uint8_t channel, bool passive, int dwell_ms);

// Returns SCANNER_RADIO_SCAN_RUNNING while scan is active, >=0 when complete
// (result count), and <0 on failure.
int scannerRadioPollAsyncScan();

void scannerRadioDeleteScanResults();

// Reads scan result by index from backend snapshot.
bool scannerRadioReadResult(int index, WiFiResult& out);
