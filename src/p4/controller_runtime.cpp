#include "controller_runtime.h"

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <atomic>
#include <new>

#include "Configuration.h"
#include "controller_gnss_runtime.h"
#include "controller_scanner_runtime.h"
#include "pico_logging.h"
#include "sdmmc_storage.h"
#include "ble_uart.h"
#include "ble_gnss.h"
#include "gps_debug.h"

namespace p4 {
namespace {
// NimBLE is precompiled into the SDK. Moving only its CONFIG_* macro would
// not move the host task; pin the Arduino application alongside it instead.
static_assert(ARDUINO_RUNNING_CORE == 0 && CONFIG_BT_NIMBLE_PINNED_TO_CORE == 0,
              "P4 GNSS/application loop and NimBLE must share core 0");
constexpr BaseType_t scannerCore = 1;
using pico_logging::QueuedScanResult;
struct Diagnostic { char text[192]; };
QueueHandle_t results = nullptr;
QueueHandle_t diagnostics = nullptr;
uint32_t queueDrops = 0, dedupeDrops = 0;
volatile uint32_t sweeps = 0, sweepMs = 0, heartbeat = 0, presentMask = 0;
std::atomic<unsigned> workerState{0};  // 0=starting, 1=running, 2=bus failure.
WiFiDedupeHash dedupeSlots[WIFI_DEDUPE_TABLE_SIZE];
WiFiDedupeHash dedupeFifo[WIFI_DEDUPE_TABLE_CAPACITY];
WiFiDedupeTable dedupe;
TinyGPSPlus gps;
TinyGPSPlus gpsUsb;
BleGnss gpsBle;
bool hardwareFix = false, usbFix = false, bleFix = false;
const char* gpsSource = "none";
GpsDebugReport gpsDebugReport;
bool gpsDebugPending = false, gpsDebugWatch = false;
size_t gpsDebugLine = 0;
uint32_t gpsDebugStarted = 0, gpsDebugCaptured = 0, gpsDebugLastLine = 0;
pico_logging::LogStorage sd;
pico_logging::LogFile logFile;
pico_logging::State logState;
constexpr uint32_t storageClocks[] = {20000000};  // Legacy config field; P4 mount policy owns the clock.
const pico_logging::Config logConfig = {
  0, storageClocks, 1, 500, 30000,
  CONTROLLER_GNSS_BOOT_TIMESTAMP_WAIT_MS, CONTROLLER_GNSS_BOOT_TIMESTAMP_READS,
  CONTROLLER_GPS_FIELD_MAX_AGE_MS, 1000, 5000, CONTROLLER_GNSS_MIN_VALID_YEAR
};
// Construct after the application's queued console is available; no per-row allocation.
alignas(pico_logging::Logger) uint8_t loggerStorage[sizeof(pico_logging::Logger)];
pico_logging::Logger* logger = nullptr;
Stream* consoleOut = nullptr;
bool active = false;
bool mainWatched = false;
bool attempted = false;
bool lastFix = false;
bool resetOnce = false;
uint32_t lastReset = 0;

bool queueResult(const QueuedScanResult& result) {
  return xQueueSend(results, &result, 0) == pdTRUE;
}
void queueDiagnostic(const char* text) {
  Diagnostic item = {};
  snprintf(item.text, sizeof(item.text), "%s", text);
  xQueueSend(diagnostics, &item, 0);  // Drop logs rather than stall SPI on a host.
}
void scannerTask(void*) {
  if (!controllerScannerRuntimeInitBus()) {
    workerState = 2;
    queueDiagnostic("Scanner SPI initialization failed; outputs left disabled\n");
    vTaskDelete(nullptr);
    return;
  }
  const ControllerScannerRuntimeContext context = {
    queueResult, &queueDrops, &dedupeDrops, &sweeps, &sweepMs,
    &dedupe, queueDiagnostic, &heartbeat, &presentMask
  };
  workerState = 1;
  controllerScannerRuntimeRun(context);
}

const char* resetName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON: return "PWRON";
    case ESP_RST_SW: return "SOFT";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WATCHDOG";
    case ESP_RST_TASK_WDT: return "TASK_WATCHDOG";
    case ESP_RST_WDT: return "WATCHDOG";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    default: return "OTHER";
  }
}

void captureGpsDebug() {
  const GpsDebugSource sources[3] = {
    {"UART", &gps, true, hardwareFix}, {"USB", &gpsUsb, false, usbFix},
    {"BLE", &gpsBle.parser(), true, bleFix}
  };
  gpsDebugReport.capture(sources, gpsSource, CONTROLLER_GPS_FIELD_MAX_AGE_MS);
  gpsDebugLine = 0;
  gpsDebugCaptured = millis();
  gpsDebugLastLine = millis() - 100;
}

void serviceGpsDebug() {
  const uint32_t now = millis();
  if (gpsDebugWatch && now - gpsDebugStarted >= 30000) {
    gpsDebugWatch = false;
    consoleOut->println("GPS DEBUG watch complete");
  }
  if (gpsDebugPending || (gpsDebugWatch && gpsDebugLine == gpsDebugReport.size() && now - gpsDebugCaptured >= 2000)) {
    gpsDebugPending = false;
    captureGpsDebug();
  }
  // At most one short line per 100 ms, not a blocking multi-line burst. Leave
  // the normal SD/GNSS/BLE/scanner work and bounded console queues in control.
  if (gpsDebugLine < gpsDebugReport.size() && now - gpsDebugLastLine >= 100) {
    consoleOut->print(gpsDebugReport.line(gpsDebugLine++));
    gpsDebugLastLine = now;
  }
}
}  // namespace

void controllerGpsDebugStart(bool watch) {
  if (!active) return;
  gpsDebugWatch = watch;
  gpsDebugPending = true;
  gpsDebugStarted = millis();
}

void controllerGpsDebugStop() {
  gpsDebugWatch = gpsDebugPending = false;
  gpsDebugLine = gpsDebugReport.size();
}

bool controllerBegin(Stream& console) {
  if (attempted) return active;
  attempted = true;
  consoleOut = &console;
  results = xQueueCreate(SCAN_RESULT_QUEUE_DEPTH, sizeof(QueuedScanResult));
  diagnostics = xQueueCreate(SERIAL_MSG_QUEUE_DEPTH, sizeof(Diagnostic));
  if (!results || !diagnostics) {
    if (results) vQueueDelete(results);
    if (diagnostics) vQueueDelete(diagnostics);
    results = diagnostics = nullptr;
    console.println("Controller queue allocation failed; SD/BLE diagnostics remain available");
    return false;
  }
  logger = new (loggerStorage) pico_logging::Logger(sd, logFile, gps, GNSS_UART, console, logConfig, logState);
  logger->initUtcTimezone();
  logger->registerSdDateTimeCallback();
  const esp_reset_reason_t reason = esp_reset_reason();
  logger->setBootResetReason(static_cast<uint8_t>(reason), resetName(reason));
  logState.sd_ready = storageReady();
  controllerGnssRuntimeBegin();  // Initial baud sweep/configuration only; no fix wait.
  wifiDedupeTableInit(&dedupe, dedupeSlots, WIFI_DEDUPE_TABLE_SIZE,
                      dedupeFifo, WIFI_DEDUPE_TABLE_CAPACITY);
  wifiDedupeTableReset(&dedupe);
  TaskHandle_t task = nullptr;
  // Core0 owns GNSS, application BLE servicing, console and all SD/file I/O,
  // alongside the higher-priority BLE host. Core1 owns scanner SPI exclusively
  // among our application tasks; communication stays on bounded queues.
  if (xTaskCreatePinnedToCore(scannerTask, "shuriken-scanner", 8192, nullptr, 1, &task, scannerCore) != pdPASS) {
    console.println("Scanner worker creation failed; reboot to retry");
    return false;
  }
  active = true;
  mainWatched = esp_task_wdt_add(nullptr) == ESP_OK;
  if (!mainWatched) console.println("Warning: controller task watchdog registration failed");
  console.println("P4 controller started: six scanner slots, UART GNSS, SDMMC CSV logger");
  console.println("Core 0: BLE/GNSS/SD/console; core 1: scanner SPI");
  return true;
}

bool controllerReceiveBle(const BleMessage& message) {
  if (!active) return false;
  const auto status = bleStatus();
  gpsBle.updateSession(status.connected, status.session, status.rxDropped);
  // A disconnect/subscription change may occur after bleRead(). Consume stale
  // chunks rather than forwarding them to the parser or ordinary text echo.
  if (!status.connected || message.session != status.session) return true;
  return gpsBle.feed(message.data, message.length, message.session);
}

void controllerPoll() {
  if (!active) return;
  if (mainWatched) esp_task_wdt_reset();
  const auto status = bleStatus();
  gpsBle.updateSession(status.connected, status.session, status.rxDropped);
  hardwareFix = controllerGnssRuntimeService(gps, CONTROLLER_GPS_FIELD_MAX_AGE_MS);
  usbFix = controllerPhoneGnssRuntimeService(gpsUsb, CONTROLLER_GPS_FIELD_MAX_AGE_MS);
  bleFix = gpsBle.usable(CONTROLLER_GPS_FIELD_MAX_AGE_MS);
  const bool fix = hardwareFix || usbFix || bleFix;
  logger->setGpsSource(selectGpsSource(gps, hardwareFix, gpsUsb, usbFix, gpsBle.parser(), bleFix));
  gpsSource = hardwareFix ? "UART" : usbFix ? "USB" : bleFix ? "BLE" : "none";
#if CONTROLLER_DEDUPE_RESET_ON_FIX_ACQUIRE
  // Relog observations when a usable location becomes available, with the same
  // cooldown and deferred scanner-side reset as the RP2350 controller.
  if (fix && !lastFix && (!resetOnce || millis() - lastReset >= CONTROLLER_DEDUPE_RESET_COOLDOWN_MS)) {
    controllerScannerRuntimeRequestDedupeReset();
    lastReset = millis();
    resetOnce = true;
  }
#endif
  lastFix = fix;
  logger->syncMasterClockFromGnss();
  logger->tryRecoverSdLogging();
  logger->tryAppendBootResetLog();

  Diagnostic message;
  for (unsigned i = 0; i < CONTROLLER_SERIAL_MSG_DRAIN_PER_LOOP &&
       xQueueReceive(diagnostics, &message, 0) == pdTRUE; ++i) {
    consoleOut->print(message.text);
  }
  // Leave results queued while waiting for storage (or time in strict mode) instead of silently
  // discarding them through appendCsvRow's not-ready guard.
  if (logState.csv_ready) {
    QueuedScanResult result;
    for (unsigned i = 0; i < CONTROLLER_SCAN_RESULT_DRAIN_PER_LOOP && logState.csv_ready &&
         xQueueReceive(results, &result, 0) == pdTRUE; ++i) logger->appendCsvRow(result);
  }
  logger->flushCsvIfDue();
  serviceGpsDebug();
}

void controllerPrintStatus(Stream& console) {
  if (!active) { console.println("Controller worker: unavailable"); return; }
  unsigned present = 0;
  const uint32_t mask = __atomic_load_n(&presentMask, __ATOMIC_RELAXED);
  for (unsigned slot = 0; slot < SCANNER_SHIFTREG_OUTPUTS; ++slot) if (mask & (1u << slot)) ++present;
  char text[256];
  const unsigned state = workerState.load();
  const uint32_t seenAt = __atomic_load_n(&heartbeat, __ATOMIC_RELAXED);
  snprintf(text, sizeof(text),
           "Scanner worker=%s; present=%u/6; queue=%u; drops=%lu; dedupe=%lu; heartbeat age=%lu ms; GNSS=%s; clock=%s; CSV=%s; rows=%lu\r\n",
           state == 1 ? "running" : state == 2 ? "SPI failed" : "starting", present,
           static_cast<unsigned>(uxQueueMessagesWaiting(results)),
           static_cast<unsigned long>(__atomic_load_n(&queueDrops, __ATOMIC_RELAXED)),
           static_cast<unsigned long>(__atomic_load_n(&dedupeDrops, __ATOMIC_RELAXED)),
           static_cast<unsigned long>(millis() - seenAt), lastFix ? "fix" : "no fix",
           logState.master_clock_valid ? "valid" : "unset",
           logState.csv_ready ? "logging" : !logState.sd_ready ? "waiting for SD" : "waiting for time/file",
           static_cast<unsigned long>(logState.csv_rows));
  console.print(text);
  snprintf(text, sizeof(text), "GPS source=%s; UART=%s; USB=%s; BLE=%s; BLE NMEA chars=%lu; checksum failures=%lu\r\n",
           gpsSource, hardwareFix ? "fix" : "no fix", usbFix ? "fix" : "no fix", bleFix ? "fix" : "no fix",
           static_cast<unsigned long>(gpsBle.parser().charsProcessed()),
           static_cast<unsigned long>(gpsBle.parser().failedChecksum()));
  console.print(text);
  if (logState.csv_ready) { console.print("Log: "); console.println(logState.csv_log_path); }
}
}  // namespace p4
