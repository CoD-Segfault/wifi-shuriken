#pragma once

#include <stddef.h>
#include <stdint.h>

namespace p4 {
struct BleMessage {
  uint16_t length;
  uint8_t data[512];
  uint32_t session;
};

struct BleStatus {
  bool ready;
  bool connected;
  bool subscribed;
  uint32_t rxDropped;
  uint32_t txDroppedBytes;
  uint32_t session;
};

// One-shot initialization. All application-facing calls belong to the main
// loop; only blePoll updates the TX characteristic. Callbacks only enqueue RX.
bool bleBegin();
void blePoll();
bool bleRead(BleMessage& message);
// Best-effort, never waits for queue space. Returns bytes enqueued, not delivered.
size_t bleWrite(const uint8_t* data, size_t length);
BleStatus bleStatus();
}  // namespace p4
