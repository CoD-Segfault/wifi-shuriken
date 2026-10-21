#pragma once
#include <cstring>
#include <deque>
#include <vector>
struct FakeQueue {
  size_t capacity;
  size_t itemSize;
  std::deque<std::vector<unsigned char>> items;
};
using QueueHandle_t = FakeQueue*;
inline QueueHandle_t xQueueCreate(size_t capacity, size_t size) {
  return new FakeQueue{capacity, size, {}};
}
inline void vQueueDelete(QueueHandle_t queue) { delete queue; }
inline int xQueueSend(QueueHandle_t queue, const void* item, unsigned timeout) {
  if (timeout != 0) throw "Unexpected blocking queue send";
  if (queue->items.size() == queue->capacity) return 0;
  const auto* data = static_cast<const unsigned char*>(item);
  queue->items.emplace_back(data, data + queue->itemSize);
  return 1;
}
inline int xQueueReceive(QueueHandle_t queue, void* item, unsigned timeout) {
  if (timeout != 0) throw "Unexpected blocking queue receive";
  if (queue->items.empty()) return 0;
  memcpy(item, queue->items.front().data(), queue->itemSize);
  queue->items.pop_front();
  return 1;
}
