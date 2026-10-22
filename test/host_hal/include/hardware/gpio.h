#pragma once
#include "Arduino.h"
inline void gpio_pull_up(int pin) { fake::events.push_back(3000 + pin); }
