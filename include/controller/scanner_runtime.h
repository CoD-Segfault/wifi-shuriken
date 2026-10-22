#pragma once

#include <stdint.h>

#include "controller/scan_result.h"
#include "wifi_dedupe.h"

// Cross-core resources owned by the controller frontend and consumed by its
// scanner worker on core1 (both RP2350 and P4).
struct ControllerScannerRuntimeContext {
  // Nonblocking enqueue of unique scan results for the application loop.
  bool (*queue_result_try)(const controller::ScanResult& result);
  // Drop counters updated by the runtime when queueing or dedupe rejects fail.
  uint32_t* scan_queue_drops;
  uint32_t* dedupe_drops;
  // Sweep timing stats surfaced by periodic controller status output.
  volatile uint32_t* sweep_cycles_completed;
  volatile uint32_t* last_full_sweep_ms;
  // Controller-wide dedupe table shared across all scanner slots.
  WiFiDedupeTable* master_dedupe_table;
  // The scanner worker cannot print directly; queue diagnostic lines instead.
  void (*serial_queue_try)(const char* text);
  // Scanner heartbeat in milliseconds; RP2350 uses it to gate watchdog feeds,
  // while P4 reports its age alongside independent task watchdogs.
  volatile uint32_t* wd_core1_last_ms;
  volatile uint32_t* scanner_presence_mask = nullptr;
};

// Initialize the target's scanner SPI bus and CS hardware.
bool controllerScannerRuntimeInitBus();

// Request a controller-wide dedupe reset. The scanner worker applies the reset
// to the master dedupe table and re-arms per-scanner dedupe handshakes.
void controllerScannerRuntimeRequestDedupeReset();

// Run the scanner service loop on its dedicated core forever.
void controllerScannerRuntimeRun(const ControllerScannerRuntimeContext& context);

// Alternative lifecycle for host event loops and deterministic regression tests.
// Call once after InitBus, then poll from the same owner. Poll services one slot
// with the existing bounded retries/drain budgets; it is not a hard-time limit.
void controllerScannerRuntimeStart(const ControllerScannerRuntimeContext& context);
void controllerScannerRuntimePoll();
