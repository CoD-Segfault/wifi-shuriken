#pragma once
#include "FreeRTOS.h"
int xTaskCreatePinnedToCore(void (*entry)(void*), const char*, unsigned, void*, unsigned, void*, unsigned core);
TickType_t xTaskGetTickCount();
void vTaskDelay(TickType_t ticks);
void vTaskDelete(void*);
