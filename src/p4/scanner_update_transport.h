#pragma once
#include "hal/controller.h"
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
namespace p4 {
// Boot-only RPC: the application retains file ownership on core 0 while all
// physical SPI operations execute on core 1, just like the normal scanner loop.
class ScannerUpdateTransport final : public controller::hal::ScannerTransport {
 public:
  explicit ScannerUpdateTransport(controller::hal::ScannerTransport& bus) : bus_(bus) {}
  bool open();
  bool close();
  bool closed() const { return requests_ == nullptr; }
  bool begin() override;
  uint8_t slotCount() const override { return bus_.slotCount(); }
  bool transfer(uint8_t slot, uint32_t hz, const uint8_t* tx, uint8_t* rx) override;
 private:
  enum class Operation : uint8_t { Begin, Transfer, Stop };
  struct Request { Operation operation; uint8_t slot; uint32_t hz; uint8_t tx[64]; };
  struct Response { bool success; uint8_t rx[64]; };
  bool exchange(const Request& request, Response& response);
  static void task(void* context);
  controller::hal::ScannerTransport& bus_;
  QueueHandle_t requests_ = nullptr, responses_ = nullptr;
  std::atomic<bool> finished_{false};
  bool pending_ = false, ready_ = false;
};
}
