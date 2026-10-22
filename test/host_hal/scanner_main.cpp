#include "controller_hal.h"
#include "controller/scanner_runtime.h"
#include "spi_protocol_shared.h"
#include <array>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <deque>
#include <vector>

using namespace controller;
struct TestClock : hal::Clock {
  uint32_t now = 100000;
  uint32_t milliseconds() const override { return now / 1000; }
  uint32_t microseconds() const override { return now; }
  uint64_t microseconds64() const override { return now; }
  void sleepMicroseconds(uint32_t us) override { now += us; }
};
struct EmptyUart : hal::ByteStream {
  int read(uint8_t*, size_t) override { return 0; }
  int write(const uint8_t*, size_t) override { return 0; }
};
struct FakeScanners : hal::ScannerTransport {
  struct Slot {
    bool present = true, supported = true, fail = false, bad_tag = false, busy = false;
    unsigned ids = 0, resets = 0, scans = 0, frames = 0;
    std::deque<WiFiResultPacket> results;
    std::array<uint8_t, SPI_FRAME_SIZE> reply = {};
  };
  TestClock* clock = nullptr;
  Slot slots[6];
  uint8_t count = 6;
  mutable unsigned slot_queries = 0;
  bool begin() override { return true; }
  uint8_t slotCount() const override { ++slot_queries; return count; }
  bool transfer(uint8_t index, uint32_t hz, const uint8_t* tx, uint8_t* rx) override {
    assert(index < count && hz == 10000000 && tx);
    if (clock) clock->now += 40;
    Slot& slot = slots[index]; ++slot.frames;
    if (slot.fail) return false;
    if (rx) std::memcpy(rx, slot.reply.data(), SPI_FRAME_SIZE);
    slot.reply.fill(0);
    if (!slot.present || tx[0] == CMD_NOP) return true;
    switch (tx[0]) {
      case CMD_ID: {
        ++slot.ids;
        DeviceIdReply id = {PROTO_VERSION, uint8_t(slot.supported ? SCANNER_TYPE_ESP32_C5 : 99),
                           CAP_BAND_24GHZ | CAP_BAND_5GHZ, PROTO_MAX_RESULTS, {0}};
        std::memcpy(slot.reply.data(), &id, sizeof(id));
        break;
      }
      case CMD_DEDUPE_RESET: ++slot.resets; slot.reply[0] = SCANNER_STATUS_OK; break;
      case CMD_SCAN:
        ++slot.scans;
        assert(tx[1] == 2 || tx[1] == 5);
        assert(tx[2] > 0);
        slot.reply[0] = SCANNER_STATUS_OK;
        break;
      case CMD_RESULT_COUNT:
        slot.reply[0] = slot.busy ? uint8_t(SCANNER_STATUS_BUSY) : uint8_t(slot.results.size());
        break;
      case CMD_RESULT_GET: {
        WiFiResultPacket packet = {};
        if (slot.results.empty()) packet.result_type = RESULT_END;
        else { packet = slot.results.front(); slot.results.pop_front(); }
        std::memcpy(slot.reply.data(), &packet, sizeof(packet));
        break;
      }
      default: assert(false);
    }
    slot.reply[SPI_FRAME_TAG_INDEX] = slot.bad_tag ? 0xee : tx[0];
    return true;
  }
};
static std::vector<controller::ScanResult> queued;
static bool queue_full = false;
static bool enqueue(const controller::ScanResult& result) {
  if (queue_full) return false;
  queued.push_back(result); return true;
}
static WiFiResultPacket observation(uint8_t id) {
  WiFiResultPacket packet = {};
  packet.result_type = RESULT_WIFI;
  std::strcpy(packet.result.ssid, "HAL regression");
  packet.result.bssid[0] = 2; packet.result.bssid[5] = id;
  packet.result.channel = 1; packet.result.band = 2; packet.result.rssi = -50;
  return packet;
}
static unsigned profile_reports = 0;
static void diagnostic(const char* text) {
  if (std::strncmp(text, "Sweep profile: wall_us=", 23) != 0) return;
  unsigned long wall, spi, wait, other, frames, passes;
  assert(std::strlen(text) < 192);
  assert(std::sscanf(text, "Sweep profile: wall_us=%lu spi_us=%lu wait_us=%lu other_us=%lu frames=%lu passes=%lu",
      &wall, &spi, &wait, &other, &frames, &passes) == 6);
  assert(spi == frames * 40 && frames > 0 && passes > 0);
  assert(wall == spi + wait + other);
  ++profile_reports;
}
int main() {
  TestClock clock; EmptyUart uart; FakeScanners scanner;
  scanner.clock = &clock;
  pc::bindPlatform(scanner, clock, uart);
  WiFiDedupeHash hash_slots[8192] = {}, fifo[4000] = {};
  WiFiDedupeTable dedupe;
  wifiDedupeTableInit(&dedupe, hash_slots, 8192, fifo, 4000);
  uint32_t queue_drops = 0, dedupe_drops = 0;
  volatile uint32_t sweeps = 0, sweep_ms = 0, heartbeat = 0, present = 0;
  ControllerScannerRuntimeContext context = {enqueue, &queue_drops, &dedupe_drops,
    &sweeps, &sweep_ms, &dedupe, diagnostic, &heartbeat, &present};
  assert(controllerScannerRuntimeInitBus());
  controllerScannerRuntimeStart(context);
  unsigned startup_queries = scanner.slot_queries;
  auto round = [&] {
    for (int i = 0; i < 6; ++i) controllerScannerRuntimePoll();
    assert(scanner.slot_queries == startup_queries);
  };
  round();
  assert(present == 0x3f && heartbeat > 0);
  for (auto& slot : scanner.slots) assert(slot.ids == 1 && slot.resets == 1 && slot.scans == 1);
  scanner.slots[0].results.push_back(observation(1));
  scanner.slots[1].results.push_back(observation(1));
  round(); assert(queued.size() == 1 && dedupe_drops == 1);
  assert(queued[0].rssi == -50 && std::strcmp(queued[0].ssid, "HAL regression") == 0);
  // Invalid observations must not enter the master dedupe table or queue.
  auto invalid = observation(2); std::memset(invalid.result.bssid, 0, 6);
  scanner.slots[0].results.push_back(invalid);
  round(); assert(queued.size() == 1 && dedupe_drops == 1);
  queue_full = true; scanner.slots[0].results.push_back(observation(3));
  round(); assert(queue_drops == 1 && queued.size() == 1); queue_full = false;
  // One busy/failing scanner cannot prevent the other slots making progress.
  scanner.slots[0].busy = true;
  const unsigned scans_before = scanner.slots[0].scans;
  round(); assert(scanner.slots[0].scans == scans_before);
  scanner.slots[1].results.push_back(observation(4));
  scanner.slots[0].fail = true;
  round(); assert(queued.size() == 2);
  scanner.slots[0].fail = false; scanner.slots[0].busy = false;
  clock.sleepMicroseconds(1000000); round();
  // Per-pass result drain is capped at sixteen, including END/BUSY replies.
  for (int i = 0; i < 20; ++i) scanner.slots[0].results.push_back(observation(uint8_t(20 + i)));
  round(); assert(scanner.slots[0].results.size() == 4);
  round(); assert(scanner.slots[0].results.empty());
  // Fix acquisition's deferred dedupe reset must allow prior rows to relog.
  controllerScannerRuntimeRequestDedupeReset();
  scanner.slots[0].busy = true;
  const unsigned resets = scanner.slots[0].resets;
  round(); assert(scanner.slots[0].resets == resets);
  scanner.slots[0].busy = false;
  clock.sleepMicroseconds(200000); round();
  assert(scanner.slots[0].resets == resets + 1);
  const size_t before_relog = queued.size();
  scanner.slots[0].results.push_back(observation(1));
  round(); assert(queued.size() == before_relog + 1);
  // Malformed tags are bounded; independent slots continue scanning.
  scanner.slots[0].bad_tag = true;
  const unsigned healthy_scans = scanner.slots[5].scans;
  round(); assert(scanner.slots[5].scans > healthy_scans);
  scanner.slots[0].bad_tag = false;
  // Restart provides deterministic state and identifies only supported slots.
  scanner.slots[0].present = false; scanner.slots[1].supported = false;
  present = 0;
  controllerScannerRuntimeStart(context); round();
  assert(present == 0x3c);
  const unsigned absent_probes = scanner.slots[0].frames;
  round(); assert(scanner.slots[0].frames == absent_probes); // reprobe cooldown
  clock.sleepMicroseconds(2100000);
  scanner.slots[0].present = true; scanner.slots[1].supported = true;
  round(); assert(present == 0x3f);
  for (unsigned i = 0; i < 200; ++i) round();
  assert(sweeps > 0);
#if SCANNER_SWEEP_PROFILE
  assert(profile_reports > 0);
#else
  assert(profile_reports == 0);
#endif
  scanner.count = 0; assert(!controllerScannerRuntimeInitBus());
  scanner.count = 9; assert(!controllerScannerRuntimeInitBus());
  scanner.count = 1; assert(controllerScannerRuntimeInitBus());
  startup_queries = scanner.slot_queries;
  present = 0;
  controllerScannerRuntimeStart(context);
  const unsigned other_frames = scanner.slots[1].frames;
  round();
  assert(present == 1 && scanner.slots[1].frames == other_frames);
  std::puts("Shared scanner: identity/reprobe/protocol/dedupe/relog/backpressure/fairness/budgets passed");
}
