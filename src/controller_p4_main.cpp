#include <Arduino.h>
#include <cstring>
#include <stdio.h>

#include "p4/ble_uart.h"
#include "p4/sdmmc_storage.h"
#include "p4/controller_runtime.h"

// P4 controller frontend. The scanner task owns SPI; this loop owns all SD I/O.
// Console diagnostics are lossy; an unread UART must not stall controller work.
namespace {
uint8_t consoleBuffer[2048];
size_t consoleHead = 0;
size_t consoleTail = 0;
size_t consoleUsed = 0;

void consoleWrite(const uint8_t* data, size_t length, bool mirrorBle = true) {
  // Mirror at production time, not when UART drains: a stalled serial monitor
  // must not stall BLE or replay pre-connection console history to a new phone.
  if (mirrorBle) {
    const auto ble = p4::bleStatus();
    if (ble.ready && ble.connected && ble.subscribed) p4::bleWrite(data, length);
  }
  // Buffer short diagnostic bursts without waiting for the UART. Drop excess
  // messages rather than blocking if the application outpaces the monitor.
  if (length > sizeof(consoleBuffer) - consoleUsed) return;
  for (size_t i = 0; i < length; ++i) {
    consoleBuffer[consoleHead] = data[i];
    consoleHead = (consoleHead + 1) % sizeof(consoleBuffer);
  }
  consoleUsed += length;
}
void consolePump() {
  const int writable = Serial0.availableForWrite();
  if (writable <= 0 || consoleUsed == 0) return;
  size_t count = sizeof(consoleBuffer) - consoleTail;
  if (count > consoleUsed) count = consoleUsed;
  if (count > 64) count = 64;
  if (count > static_cast<size_t>(writable)) count = writable;
  const size_t written = Serial0.write(consoleBuffer + consoleTail, count);
  consoleTail = (consoleTail + written) % sizeof(consoleBuffer);
  consoleUsed -= written;
}
void consoleText(const char* text, bool mirrorBle = true) {
  consoleWrite(reinterpret_cast<const uint8_t*>(text), strlen(text), mirrorBle);
}
class QueuedConsoleStream : public Stream {
 public:
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  void flush() override {}  // Never wait for a serial consumer.
  int availableForWrite() override { return sizeof(consoleBuffer) - consoleUsed; }
  size_t write(uint8_t byte) override { consoleWrite(&byte, 1); return 1; }
  size_t write(const uint8_t* bytes, size_t length) override {
    consoleWrite(bytes, length);
    return length;
  }
} consoleStream;

void reportStatus() {
  const p4::BleStatus ble = p4::bleStatus();
  char text[256];
  snprintf(text, sizeof(text),
           "SD: %s (%s); BLE: %s, phone=%s, subscribed=%s; RX drops=%lu, TX discarded bytes=%lu\r\n",
           p4::storageReady() ? "mounted" : "unavailable", p4::storageFilesystem(),
           ble.ready ? "ready" : "unavailable", ble.connected ? "yes" : "no",
           ble.subscribed ? "yes" : "no", static_cast<unsigned long>(ble.rxDropped),
           static_cast<unsigned long>(ble.txDroppedBytes));
  consoleText(text);
  p4::controllerPrintStatus(consoleStream);
}

constexpr uint32_t SD_TEST_DURATION_MS = 10000;
constexpr uint32_t SD_TEST_INTERVAL_MS = 100;
struct {
  bool active = false;
  uint32_t startedAt = 0;
  uint32_t nextCheckAt = 0;
  uint32_t lastReportAt = 0;
  uint32_t passed = 0;
  uint32_t failed = 0;
} sdTest;

void reportSdTest(const char* state) {
  char text[192];
  snprintf(text, sizeof(text),
           "SD test %s: %lu passed, %lu failed, %lu KiB verified, %lu ms elapsed.\r\n",
           state, static_cast<unsigned long>(sdTest.passed),
           static_cast<unsigned long>(sdTest.failed),
           static_cast<unsigned long>(sdTest.passed * 4),
           static_cast<unsigned long>(millis() - sdTest.startedAt));
  consoleText(text);
}

void startSdTest() {
  if (sdTest.active) {
    consoleText("SD test already running; use :status or keep sending phone data.\r\n");
    return;
  }
  if (!p4::storageReady()) {
    consoleText("SD test unavailable: no mounted card.\r\n");
    return;
  }
  sdTest = {};
  sdTest.active = true;
  sdTest.startedAt = sdTest.nextCheckAt = sdTest.lastReportAt = millis();
  consoleText("SD test started: repeated 4096-byte checks for 10 seconds. Send phone data now.\r\n");
}

void serviceSdTest() {
  if (!sdTest.active) return;
  const uint32_t now = millis();
  if (now - sdTest.startedAt >= SD_TEST_DURATION_MS) {
    sdTest.active = false;
    reportSdTest(sdTest.failed == 0 && sdTest.passed > 0 ? "PASS" : "FAIL");
    reportStatus();
    return;
  }
  if (static_cast<int32_t>(now - sdTest.nextCheckAt) < 0) return;
  // One check per loop at most. Each write/read is synchronous, but BLE and
  // serial servicing run between checks; never sleep or spin for the duration.
  if (p4::storageVerify()) {
    ++sdTest.passed;
  } else {
    ++sdTest.failed;
    sdTest.active = false;
    reportSdTest("FAIL");
    reportStatus();
    return;
  }
  sdTest.nextCheckAt = millis() + SD_TEST_INTERVAL_MS;
  if (millis() - sdTest.lastReportAt >= 1000) {
    sdTest.lastReportAt = millis();
    reportSdTest("running");
    reportStatus();
  }
}

void handleSerialLine(const uint8_t* data, size_t length) {
  // Local serial-only commands. BLE writes are never interpreted as
  // storage commands on this unauthenticated bring-up service.
  if (length && data[0] == ':') {
    char command[129];
    memcpy(command, data, length);
    command[length] = '\0';
    while (length && (command[length - 1] == '\n' || command[length - 1] == '\r')) {
      command[--length] = '\0';
    }
    if (strcmp(command, ":status") == 0) {
      reportStatus();
    } else if (strcmp(command, ":sd-test") == 0) {
      startSdTest();
    } else if (strcmp(command, ":gps-debug") == 0) {
      p4::controllerGpsDebugStart(false);
    } else if (strcmp(command, ":gps-debug on") == 0) {
      p4::controllerGpsDebugStart(true);
    } else if (strcmp(command, ":gps-debug off") == 0) {
      p4::controllerGpsDebugStop();
    } else {
      consoleText("Local commands: :status, :sd-test, :gps-debug [on|off] (finish with newline).\r\n");
    }
  } else {
    p4::bleWrite(data, length);
  }
}

void serviceSerial() {
  static uint8_t line[128];
  static size_t length = 0;
  static uint32_t lastByte = 0;
  static bool discardLine = false;
  for (unsigned read = 0; read < 128 && Serial0.available(); ++read) {
    const int c = Serial0.read();
    if (c < 0) break;
    if (discardLine) {
      if (c == '\n') discardLine = false;
      continue;
    }
    line[length++] = static_cast<uint8_t>(c);
    lastByte = millis();
    if (c == '\n') {
      handleSerialLine(line, length);
      length = 0;
    } else if (length == sizeof(line)) {
      if (line[0] == ':') {
        discardLine = true;
        consoleText("Local command too long; discarded.\r\n");
      } else {
        handleSerialLine(line, length);
      }
      length = 0;
    }
  }
  // Ordinary serial text is a byte stream; local commands require a newline.
  if (length && line[0] != ':' && millis() - lastByte >= 50) {
    handleSerialLine(line, length);
    length = 0;
  }
}
}  // namespace

void setup() {
  Serial0.begin(115200);
  delay(1500);
  consoleText("Shuriken P4 controller: SDMMC, BLE UART, scanner worker and logger.\r\n");
  if (!p4::storageBegin()) consoleText("SD mount failed; no formatting attempted. Continuing with BLE.\r\n");
  if (!p4::bleBegin()) consoleText("BLE startup failed; retain ESP-Hosted logs before considering C6 changes.\r\n");
  if (!p4::controllerBegin(consoleStream)) consoleText("Controller startup failed; diagnostic commands remain available.\r\n");
  consoleText("BLE UART: Shuriken-P4. Local serial commands: :status, :sd-test, :gps-debug [on|off]\r\n");
  reportStatus();
}

void loop() {
  consolePump();
  serviceSerial();
  p4::blePoll();
  p4::BleMessage message;
  for (unsigned i = 0; i < 2 && p4::bleRead(message); ++i) {
    if (p4::controllerReceiveBle(message)) continue;
    consoleText("BLE RX: ", false);
    consoleWrite(message.data, message.length, false);
    consoleText("\r\n", false);
    // Do not echo phone input back over BLE; NMEA is handled above.
  }
  p4::controllerPoll();
  serviceSdTest();
  delay(5);
}
