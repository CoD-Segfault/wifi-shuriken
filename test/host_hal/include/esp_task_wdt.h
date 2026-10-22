#pragma once
#include "Arduino.h"
inline int esp_task_wdt_add(void*) { ++fake::watchdog_adds; return fake::watchdog_ok ? 0 : -1; }
inline int esp_task_wdt_reset() { ++fake::watchdog_feeds; return 0; }
