#pragma once
#include "Arduino.h"
using gpio_num_t = int;
#define ESP_OK 0
inline int gpio_pullup_en(int pin) { fake::events.push_back(3000 + pin); return fake::pullup_ok ? 0 : -1; }
