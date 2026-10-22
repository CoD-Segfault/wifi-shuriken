#include "scanner_update_transport.h"
#include <atomic>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>
struct FakeQueue {
  size_t size;
  unsigned capacity;
  std::mutex mutex;
  std::condition_variable cv;
  std::deque<std::vector<uint8_t>> items;
  FakeQueue(unsigned count, size_t bytes) : size(bytes), capacity(count) {}
};
static unsigned queues = 0, creations = 0, fail_queue = 0;
static bool fail_task = false, fail_send = false;
static thread_local unsigned current_core = 0;
static std::vector<std::thread> tasks;
QueueHandle_t xQueueCreate(unsigned capacity, size_t size) {
  assert(capacity == 1);
  if (++creations == fail_queue) return nullptr;
  ++queues; return new FakeQueue(capacity, size);
}
int xQueueSend(QueueHandle_t queue, const void* data, TickType_t timeout) {
  assert(timeout == 0);
  if (current_core == 0 && fail_send) { fail_send = false; return pdFALSE; }
  std::lock_guard<std::mutex> lock(queue->mutex);
  if (queue->items.size() == queue->capacity) return pdFALSE;
  const auto* bytes = static_cast<const uint8_t*>(data);
  queue->items.emplace_back(bytes, bytes + queue->size); queue->cv.notify_one(); return pdTRUE;
}
int xQueueReceive(QueueHandle_t queue, void* output, TickType_t timeout) {
  std::unique_lock<std::mutex> lock(queue->mutex);
  if (timeout == portMAX_DELAY) queue->cv.wait(lock, [&] { return !queue->items.empty(); });
  else if (!queue->cv.wait_for(lock, std::chrono::milliseconds(timeout), [&] { return !queue->items.empty(); })) return pdFALSE;
  std::memcpy(output, queue->items.front().data(), queue->size); queue->items.pop_front(); return pdTRUE;
}
void vQueueDelete(QueueHandle_t queue) { --queues; delete queue; }
int xTaskCreatePinnedToCore(void (*entry)(void*), const char*, unsigned, void* context, unsigned, void*, unsigned core) {
  assert(core == 1);
  if (fail_task) return pdFALSE;
  tasks.emplace_back([=] { current_core = core; entry(context); }); return pdPASS;
}
TickType_t xTaskGetTickCount() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
void vTaskDelay(TickType_t ticks) { std::this_thread::sleep_for(std::chrono::milliseconds(ticks)); }
void vTaskDelete(void*) {}
static void join() { for (auto& task : tasks) task.join(); tasks.clear(); assert(queues == 0); }
struct Bus final : controller::hal::ScannerTransport {
  bool begin_ok = true, transfer_ok = true;
  unsigned begins = 0, frames = 0;
  std::atomic<bool> blocked{false};
  bool begin() override { assert(current_core == 1); ++begins; return begin_ok; }
  uint8_t slotCount() const override { return 6; }
  bool transfer(uint8_t slot, uint32_t hz, const uint8_t* tx, uint8_t* rx) override {
    assert(current_core == 1 && slot < 6 && hz == 10000000); ++frames;
    while (blocked) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    for (unsigned i = 0; i < 64; ++i) rx[i] = tx[i] ^ 0xa5;
    return transfer_ok;
  }
};
int main() {
  for (int failure = 0; failure < 3; ++failure) {
    Bus bus; p4::ScannerUpdateTransport proxy(bus);
    fail_queue = failure < 2 ? creations + failure + 1 : 0; fail_task = failure == 2;
    assert(!proxy.open() && proxy.closed()); assert(bus.begins == 0); join();
    fail_queue = 0; fail_task = false;
  }
  Bus bus; p4::ScannerUpdateTransport proxy(bus);
  assert(proxy.open() && !proxy.open()); assert(proxy.begin() && proxy.slotCount() == 6);
  uint8_t tx[64], rx[64]; for (unsigned i = 0; i < 64; ++i) tx[i] = i;
  for (uint8_t slot = 0; slot < 6; ++slot) {
    assert(proxy.transfer(slot, 10000000, tx, rx));
    for (unsigned i = 0; i < 64; ++i) assert(rx[i] == uint8_t(i ^ 0xa5));
  }
  assert(proxy.transfer(0, 10000000, nullptr, nullptr));
  assert(proxy.close() && proxy.close() && proxy.closed()); join();
  assert(!proxy.transfer(0, 10000000, tx, rx));
  assert(proxy.open()); bus.begin_ok = false; assert(!proxy.begin()); assert(proxy.close()); join();
  assert(proxy.open()); fail_send = true; assert(!proxy.begin()); assert(proxy.close()); join();
  bus.begin_ok = true; assert(proxy.open() && proxy.begin());
  bus.transfer_ok = false; std::memset(rx, 0x42, sizeof(rx));
  assert(!proxy.transfer(0, 10000000, tx, rx)); for (auto byte : rx) assert(byte == 0x42);
  assert(proxy.close()); join();
  bus.transfer_ok = true; assert(proxy.open() && proxy.begin()); bus.blocked = true;
  assert(!proxy.transfer(0, 10000000, tx, rx));
  assert(!proxy.close() && !proxy.closed() && queues == 2);
  bus.blocked = false;
  assert(proxy.close() && proxy.closed()); join();
  std::puts("P4 boot SPI bridge: core affinity/frames/failures/timeouts/safe queue lifetime passed");
}
