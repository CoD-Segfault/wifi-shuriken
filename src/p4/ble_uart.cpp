#include "ble_uart.h"

#include <Arduino.h>
#include <BLEDevice.h>
#include <atomic>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace p4 {
namespace {
constexpr char SERVICE_UUID[] = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char RX_UUID[] = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char TX_UUID[] = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr size_t TX_BYTES = 20;  // Works at the default ATT MTU of 23.
constexpr unsigned RX_DEPTH = 4;
constexpr unsigned TX_DEPTH = 32;

struct Incoming {
  uint32_t session;
  BleMessage message;
};
struct Outgoing {
  uint32_t session;
  uint8_t length;
  uint8_t data[TX_BYTES];
};

QueueHandle_t incoming = nullptr;
QueueHandle_t outgoing = nullptr;
BLECharacteristic* tx = nullptr;
bool attempted = false;
bool ready = false;
std::atomic<bool> connected{false};
std::atomic<bool> subscribed{false};
std::atomic<bool> greetingPending{false};
std::atomic<uint32_t> session{0};
std::atomic<uint32_t> rxDropped{0};
std::atomic<uint32_t> txDroppedBytes{0};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    session.fetch_add(1);
    connected = true;
  }
  void onDisconnect(BLEServer*) override {
    subscribed = false;
    connected = false;
    greetingPending = false;
    session.fetch_add(1);  // Queued data must not leak into the next connection.
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    const String value = characteristic->getValue();
    if (!connected || value.length() > sizeof(BleMessage::data)) {
      rxDropped.fetch_add(1);
      return;
    }
    Incoming item = {};
    item.session = session.load();
    item.message.length = static_cast<uint16_t>(value.length());
    memcpy(item.message.data, value.c_str(), value.length());
    if (xQueueSend(incoming, &item, 0) != pdTRUE) rxDropped.fetch_add(1);
  }
};

class TxCallbacks : public BLECharacteristicCallbacks {
  void onSubscribe(BLECharacteristic*, ble_gap_conn_desc*, uint16_t value) override {
    // Invalidate queued output when subscriptions change, too.
    session.fetch_add(1);
    subscribed = (value & 1U) != 0;
    greetingPending = subscribed.load();
  }
};

ServerCallbacks serverCallbacks;
RxCallbacks rxCallbacks;
TxCallbacks txCallbacks;

bool startupFailed() {
  connected = false;
  subscribed = false;
  greetingPending = false;
  BLEDevice::deinit();
  tx = nullptr;
  // Retain queues for the lifetime of this one-shot module: callbacks must
  // never race freed queue storage during partial-startup cleanup.
  return false;
}
}  // namespace

bool bleBegin() {
  if (attempted) return ready;
  attempted = true;
  incoming = xQueueCreate(RX_DEPTH, sizeof(Incoming));
  outgoing = xQueueCreate(TX_DEPTH, sizeof(Outgoing));
  if (!incoming || !outgoing) {
    if (incoming) vQueueDelete(incoming);
    if (outgoing) vQueueDelete(outgoing);
    incoming = outgoing = nullptr;
    return false;
  }
  // C6 radio on SDIO slot 1, separate from the slot-0 SD card.
  if (!BLEDevice::setPins(18, 19, 14, 15, 16, 17, 54) ||
      !BLEDevice::init("Shuriken-P4")) return startupFailed();
  BLEServer* server = BLEDevice::createServer();
  if (!server) return startupFailed();
  server->setCallbacks(&serverCallbacks);
  server->advertiseOnDisconnect(true);
  BLEService* service = server->createService(SERVICE_UUID);
  if (!service) return startupFailed();
  tx = service->createCharacteristic(TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  BLECharacteristic* rx = service->createCharacteristic(
      RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  if (!tx || !rx) return startupFailed();
  tx->setCallbacks(&txCallbacks);
  rx->setCallbacks(&rxCallbacks);
  if (!service->start()) return startupFailed();
  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  if (!advertising) return startupFailed();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  ready = advertising->start();
  if (!ready) return startupFailed();
  return true;
}

size_t bleWrite(const uint8_t* data, size_t length) {
  if (!data || length == 0) return 0;
  if (!ready || !connected || !subscribed) {
    txDroppedBytes.fetch_add(length);
    return 0;
  }
  const uint32_t current = session.load();
  size_t accepted = 0;
  while (accepted < length) {
    Outgoing item = {};
    item.session = current;
    const size_t remaining = length - accepted;
    item.length = static_cast<uint8_t>(remaining < TX_BYTES ? remaining : TX_BYTES);
    memcpy(item.data, data + accepted, item.length);
    if (xQueueSend(outgoing, &item, 0) != pdTRUE) break;
    accepted += item.length;
  }
  txDroppedBytes.fetch_add(length - accepted);
  return accepted;
}

void blePoll() {
  if (!ready) return;
  if (greetingPending.exchange(false) && connected && subscribed) {
    constexpr char greeting[] = "Hello from Shuriken P4!\n";
    bleWrite(reinterpret_cast<const uint8_t*>(greeting), sizeof(greeting) - 1);
  }
  Outgoing item;
  // Explicit budget even when another task or radio completion runs quickly.
  for (unsigned i = 0; i < 4 && xQueueReceive(outgoing, &item, 0) == pdTRUE; ++i) {
    if (!connected || !subscribed || item.session != session.load()) {
      txDroppedBytes.fetch_add(item.length);
      continue;
    }
    tx->setValue(item.data, item.length);
    tx->notify();  // Notifications are best-effort, not application acknowledgements.
  }
}

bool bleRead(BleMessage& message) {
  if (!ready) return false;
  Incoming item;
  for (unsigned i = 0; i < RX_DEPTH && xQueueReceive(incoming, &item, 0) == pdTRUE; ++i) {
    if (connected && item.session == session.load()) {
      message = item.message;
      message.session = item.session;
      return true;
    }
    rxDropped.fetch_add(1);
  }
  return false;
}

BleStatus bleStatus() {
  return {ready, connected.load(), subscribed.load(), rxDropped.load(), txDroppedBytes.load(), session.load()};
}
}  // namespace p4
