#include <BLEDevice.h>
#include "ble_uart.h"
#include <cassert>
#include <cstdio>

static size_t write(const String& text) {
  return p4::bleWrite(reinterpret_cast<const uint8_t*>(text.data()), text.size());
}
static void subscribe() {
  auto& server = BLEDevice::server;
  server.callbacks->onConnect(&server);
  server.service.tx.callbacks->onSubscribe(&server.service.tx, nullptr, 1);
  p4::blePoll();
  server.service.tx.notifications.clear();
}
static void receive(const String& text) {
  auto& rx = BLEDevice::server.service.rx;
  rx.value = text;
  rx.callbacks->onWrite(&rx);
}

int main(int argc, char**) {
  if (argc > 1) {
    BLEDevice::advertising.succeeds = false;
    assert(!p4::bleBegin() && !p4::bleBegin());
    assert(BLEDevice::initCalls == 1 && BLEDevice::deinitCalls == 1);
    assert(write("unsent") == 0 && !p4::bleStatus().ready);
    p4::blePoll();
    puts("PASS: startup failure cleanup, one-shot initialization, disabled TX");
    return 0;
  }
  assert(p4::bleBegin() && p4::bleBegin() && BLEDevice::initCalls == 1);
  assert(write("offline") == 0);
  assert(p4::bleStatus().txDroppedBytes == 7);
  subscribe();
  auto& tx = BLEDevice::server.service.tx;
  const String message(45, 'a');
  assert(write(message) == 45 && tx.notifications.empty());
  p4::blePoll();
  assert(tx.notifications.size() == 3);
  assert(tx.notifications[0].size() == 20 && tx.notifications[1].size() == 20 && tx.notifications[2].size() == 5);
  assert(tx.notifications[0] + tx.notifications[1] + tx.notifications[2] == message);

  tx.notifications.clear();
  assert(write(String(1000, 'b')) == 640);  // Bounded queue, no waiting.
  p4::blePoll();
  assert(tx.notifications.size() == 4);  // Per-loop budget.
  assert(p4::bleStatus().txDroppedBytes == 367);
  for (int i = 0; i < 7; ++i) p4::blePoll();
  assert(tx.notifications.size() == 32);
  for (int i = 0; i < 5; ++i) receive(String(512, 'c'));
  assert(p4::bleStatus().rxDropped == 1);
  p4::BleMessage item;
  for (int i = 0; i < 4; ++i) {
    assert(p4::bleRead(item) && item.length == 512 && item.data[511] == 'c');
    assert(item.session == p4::bleStatus().session);
  }
  assert(!p4::bleRead(item));
  receive(String(513, 'd'));
  assert(!p4::bleRead(item) && p4::bleStatus().rxDropped == 2);

  // Data pending on both sides of a disconnect must never replay to a new phone.
  assert(write("old TX") == 6);
  receive("old RX");
  BLEDevice::server.callbacks->onDisconnect(&BLEDevice::server);
  subscribe();
  p4::blePoll();
  assert(tx.notifications.empty() && !p4::bleRead(item));
  receive("new RX");
  assert(p4::bleRead(item) && item.length == 6);
  assert(String(reinterpret_cast<char*>(item.data), item.length) == "new RX");

  assert(write("before unsubscribe") > 0);
  tx.callbacks->onSubscribe(&tx, nullptr, 0);
  p4::blePoll();
  assert(tx.notifications.empty() && write("no subscriber") == 0);
  // Phone NMEA input must not require subscribing to outgoing notifications.
  receive("RX without TX subscription");
  assert(p4::bleRead(item) && item.session == p4::bleStatus().session);
  puts("PASS: chunking, bounded queues/polling, RX limits, drops, stale-session rejection, unsubscribe");
}
