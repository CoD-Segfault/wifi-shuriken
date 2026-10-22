#include "scanner_update_transport.h"
#include <cstring>
namespace p4 {
namespace { constexpr uint32_t timeoutMs = 1000; }
bool ScannerUpdateTransport::open() {
  if (!closed()) return false;
  finished_ = false; pending_ = ready_ = false;
  requests_ = xQueueCreate(1, sizeof(Request));
  responses_ = xQueueCreate(1, sizeof(Response));
  if (!requests_ || !responses_ ||
      xTaskCreatePinnedToCore(task, "scanner-boot-spi", 4096, this, 1, nullptr, 1) != pdPASS) {
    if (requests_) vQueueDelete(requests_);
    if (responses_) vQueueDelete(responses_);
    requests_ = responses_ = nullptr;
    return false;
  }
  return true;
}
bool ScannerUpdateTransport::exchange(const Request& request, Response& response) {
  if (closed() || pending_ || xQueueSend(requests_, &request, 0) != pdTRUE) return false;
  pending_ = true;
  if (xQueueReceive(responses_, &response, pdMS_TO_TICKS(timeoutMs)) != pdTRUE) return false;
  pending_ = false;
  return true;
}
bool ScannerUpdateTransport::begin() {
  const Request request = {Operation::Begin, 0, 0, {0}};
  Response response = {};
  ready_ = exchange(request, response) && response.success;
  return ready_;
}
bool ScannerUpdateTransport::transfer(uint8_t slot, uint32_t hz, const uint8_t* tx, uint8_t* rx) {
  if (!ready_) return false;
  Request request = {Operation::Transfer, slot, hz, {0}};
  if (tx) std::memcpy(request.tx, tx, sizeof(request.tx));
  Response response = {};
  if (!exchange(request, response) || !response.success) { ready_ = false; return false; }
  if (rx) std::memcpy(rx, response.rx, sizeof(response.rx));
  return true;
}
bool ScannerUpdateTransport::close() {
  if (closed()) return true;
  ready_ = false;
  // A timeout never permits concurrent normal scanning or freeing a queue that
  // the worker may still be using. Drain the outstanding operation first.
  Response response = {};
  if (pending_) {
    if (xQueueReceive(responses_, &response, pdMS_TO_TICKS(timeoutMs)) != pdTRUE) return false;
    pending_ = false;
  }
  const Request stop = {Operation::Stop, 0, 0, {0}};
  if (!exchange(stop, response) && !finished_) return false;
  const TickType_t start = xTaskGetTickCount();
  while (!finished_) {
    if (xTaskGetTickCount() - start >= pdMS_TO_TICKS(timeoutMs)) return false;
    vTaskDelay(1);
  }
  vQueueDelete(requests_); vQueueDelete(responses_);
  requests_ = responses_ = nullptr; pending_ = false;
  return true;
}
void ScannerUpdateTransport::task(void* context) {
  auto& self = *static_cast<ScannerUpdateTransport*>(context);
  Request request;
  while (xQueueReceive(self.requests_, &request, portMAX_DELAY) == pdTRUE) {
    Response response = {};
    if (request.operation == Operation::Begin) response.success = self.bus_.begin();
    else if (request.operation == Operation::Transfer)
      response.success = self.bus_.transfer(request.slot, request.hz, request.tx, response.rx);
    else response.success = true;
    xQueueSend(self.responses_, &response, 0);
    if (request.operation == Operation::Stop) break;
  }
  self.finished_ = true;  // No further access to this object or its queues.
  vTaskDelete(nullptr);
}
}
