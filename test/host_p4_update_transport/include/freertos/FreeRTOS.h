#pragma once
#include <algorithm>
#include <cstdint>
using TickType_t = uint32_t;
constexpr int pdTRUE = 1, pdFALSE = 0, pdPASS = 1;
constexpr TickType_t portMAX_DELAY = UINT32_MAX;
#define pdMS_TO_TICKS(value) (std::min<TickType_t>((value), 20))
