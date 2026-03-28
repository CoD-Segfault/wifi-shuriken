#pragma once

#include <stdint.h>

#include "pico/util/queue.h"
#include "wifi_dedupe.h"

// Cross-core resources owned by controller_main.cpp and consumed by the
// scanner runtime on core1.
struct ControllerScannerRuntimeContext {
  // Queue of unique scan results produced on core1 and drained on core0.
  queue_t* scan_result_queue;
  // Drop counters updated by the runtime when queueing or dedupe rejects fail.
  uint32_t* scan_queue_drops;
  uint32_t* dedupe_drops;
  // Sweep timing stats surfaced by periodic controller status output.
  volatile uint32_t* sweep_cycles_completed;
  volatile uint32_t* last_full_sweep_ms;
  // Controller-wide dedupe table shared across all scanner slots.
  WiFiDedupeTable* master_dedupe_table;
  // Core1 cannot print directly; queue diagnostic lines for core0 instead.
  void (*serial_queue_try)(const char* text);
};

// Initialize the SPI1 bus and any scanner CS hardware.
void controllerScannerRuntimeInitBus();

// Request a controller-wide dedupe reset from core0. Core1 applies the reset
// to the master dedupe table and re-arms per-scanner dedupe handshakes.
void controllerScannerRuntimeRequestDedupeReset();

// Run the scanner service loop on core1 forever.
void controllerScannerRuntimeRun(const ControllerScannerRuntimeContext& context);
