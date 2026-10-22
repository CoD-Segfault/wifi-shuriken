#pragma once
#include <Arduino.h>
inline int64_t esp_timer_get_time() { return fake::now; }
