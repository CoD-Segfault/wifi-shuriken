#pragma once
#include "FreeRTOS.h"
#include <cstddef>
struct FakeQueue;
using QueueHandle_t = FakeQueue*;
QueueHandle_t xQueueCreate(unsigned capacity, size_t size);
int xQueueSend(QueueHandle_t, const void*, TickType_t);
int xQueueReceive(QueueHandle_t, void*, TickType_t);
void vQueueDelete(QueueHandle_t);
