#pragma once

#include <stdint.h>

#include "pico/util/queue.h"
#include "wifi_dedupe.h"

struct ControllerScannerRuntimeContext {
  queue_t* scan_result_queue;
  uint32_t* scan_queue_drops;
  uint32_t* dedupe_drops;
  volatile uint32_t* sweep_cycles_completed;
  volatile uint32_t* last_full_sweep_ms;
  WiFiDedupeTable* master_dedupe_table;
  void (*serial_queue_try)(const char* text);
};

void controllerScannerRuntimeInitBus();
void controllerScannerRuntimeRun(const ControllerScannerRuntimeContext& context);
