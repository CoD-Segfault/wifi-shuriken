#pragma once
#include "Arduino.h"
inline void vTaskDelay(int ticks) { fake::now += ticks * 1000; ++fake::yields; }
