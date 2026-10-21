// Include the actual application loop so the scheduler is tested, not a copy.
#include "../../src/controller_p4_main.cpp"
#include <cassert>

static bool cardReady = true;
static bool checkResult = true;
static unsigned checks = 0, blePolls = 0, echoed = 0;
static unsigned received = 0;
static bool nmeaMessage = false;
static bool phoneMessage = false;
static uint32_t lastBlePoll = 0;
static bool bleConnected = false, bleSubscribed = false, bleAccepts = true;
static std::string bleOutput;

namespace p4 {
bool controllerBegin(Stream&) { return true; }
void controllerPoll() {}
void controllerPrintStatus(Stream&) {}
static unsigned gpsDebugStarts = 0, gpsDebugStops = 0;
static bool gpsDebugWatch = false;
void controllerGpsDebugStart(bool watch) { ++gpsDebugStarts; gpsDebugWatch = watch; }
void controllerGpsDebugStop() { ++gpsDebugStops; }
bool controllerReceiveBle(const BleMessage&) { ++received; return nmeaMessage; }
bool storageBegin() { return cardReady; }
bool storageReady() { return cardReady; }
const char* storageFilesystem() { return "exFAT"; }
bool storageVerify() {
  assert(lastBlePoll == millis());  // BLE serviced before every SD check.
  ++checks;
  return checkResult;
}
bool bleBegin() { return true; }
void blePoll() { ++blePolls; lastBlePoll = millis(); }
BleStatus bleStatus() { return {true, bleConnected, bleSubscribed, 0, 0, 0}; }
size_t bleWrite(const uint8_t* bytes, size_t length) {
  ++echoed;
  if (!bleAccepts) return 0;
  bleOutput.append(reinterpret_cast<const char*>(bytes), length);
  return length;
}
bool bleRead(BleMessage& message) {
  if (!phoneMessage) return false;
  phoneMessage = false;
  message.length = 3;
  memcpy(message.data, "hi\n", 3);
  return true;
}
}

int main() {
  setup();
  Serial0.command(":sd-test\n");
  loop();
  assert(sdTest.active && checks == 1);
  const uint32_t start = sdTest.startedAt;
  Serial0.command(":sd-test\n:status\n");
  loop();
  assert(sdTest.startedAt == start && checks == 1);  // No restart or busy-loop.
  for (unsigned i = 0; i < 2500; ++i) {
    if (i % 10 == 0) phoneMessage = true;
    loop();
  }
  assert(!sdTest.active && checks == 100 && blePolls >= 2500 && echoed == 0);
  assert(Serial0.output.find("SD test running:") != std::string::npos);
  assert(Serial0.output.find("SD test PASS: 100 passed, 0 failed, 400 KiB verified") != std::string::npos);
  assert(received == 250);
  nmeaMessage = phoneMessage = true;
  loop();
  assert(received == 251 && echoed == 0);  // Neither ordinary input nor NMEA echoes.
  nmeaMessage = false;
  Serial0.command("to phone\n");
  loop();
  assert(echoed == 1);  // Explicit serial-to-phone output still works.

  checkResult = false;
  Serial0.command(":sd-test\n");
  loop();
  assert(!sdTest.active && sdTest.failed == 1 && checks == 101);
  for (unsigned i = 0; i < 50; ++i) loop();
  assert(checks == 101 && Serial0.output.find("SD test FAIL:") != std::string::npos);

  cardReady = false;
  Serial0.command(":sd-test\n");
  loop();
  assert(!sdTest.active && checks == 101);

  // Unsigned elapsed time and signed deadlines must survive millis rollover.
  cardReady = checkResult = true;
  fakeMillis = UINT32_MAX - 500;
  Serial0.command(":sd-test\n");
  for (unsigned i = 0; i < 2500; ++i) loop();
  assert(!sdTest.active && checks == 201 && sdTest.passed == 100);

  consoleText("old console history");
  bleConnected = bleSubscribed = true;
  bleOutput.clear();
  Serial0.writable = 0;
  consoleText("fresh console output\r\n");
  assert(bleOutput == "fresh console output\r\n");  // Independent of UART drain.
  phoneMessage = true;
  loop();
  assert(bleOutput == "fresh console output\r\n");  // BLE RX must not loop via console mirroring.
  nmeaMessage = phoneMessage = true;
  loop();
  assert(bleOutput == "fresh console output\r\n");
  nmeaMessage = false;

  bleAccepts = false;
  const unsigned writes = echoed;
  const std::string flood(2048, 'x');
  consoleWrite(reinterpret_cast<const uint8_t*>(flood.data()), flood.size());
  loop();
  assert(echoed == writes + 1 && consoleUsed <= sizeof(consoleBuffer));  // No wait/retry on full TX.
  bleAccepts = true;
  bleSubscribed = false;
  consoleText("not subscribed");
  bleConnected = false;
  consoleText("not connected");
  assert(echoed == writes + 1);
  Serial0.writable = 64;
  for (unsigned i = 0; i < 100; ++i) loop();
  assert(echoed == writes + 1);  // UART draining cannot replay old output over BLE.
  Serial0.command(":gps-debug\n");
  loop();
  assert(p4::gpsDebugStarts == 1 && !p4::gpsDebugWatch);
  Serial0.command(":gps-debug on\n");
  loop();
  assert(p4::gpsDebugStarts == 2 && p4::gpsDebugWatch);
  Serial0.command(":gps-debug off\n");
  loop();
  assert(p4::gpsDebugStops == 1);
  puts("PASS: SD/BLE scheduling, console mirror independent of UART, RX echo isolation, full/disconnected TX, commands, timer rollover");
}
