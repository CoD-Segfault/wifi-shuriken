#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <TinyGPSPlus.h>
#include <SdFat.h>
#if defined(USE_TINYUSB)
#include <Adafruit_TinyUSB.h>
#endif
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "hardware/watchdog.h"
#include "hardware/gpio.h"
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>
#include <stdarg.h>
#include "pico_logging.h"
#include "wifi_dedupe.h"
#include "Configuration.h"
#include "controller_gnss_runtime.h"
#include "controller_scanner_runtime.h"

// Top-level controller orchestration lives here after the runtime split.
// This file owns Arduino entrypoints, shared peripherals, cross-core queues,
// SD/logging lifecycle, and wiring between the GNSS and scanner runtimes.

// Hardware/peripheral instances owned by core0.
SdFat sd;
FsFile logFile;
Adafruit_NeoPixel pixels(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);
TinyGPSPlus gps;
TinyGPSPlus gps_phone;
using QueuedScanResult = pico_logging::QueuedScanResult;

// Cross-core queues:
// - core1 (SPI/scanner) produces scan rows + diagnostic lines.
// - core0 (main loop) consumes and performs serial output + SD writes.
static queue_t scan_result_queue;
struct QueuedSerialMsg {
  char text[192];
};
static queue_t serial_msg_queue;
static uint32_t scan_queue_drops = 0;
static uint32_t serial_msg_drops = 0;
static uint32_t dedupe_drops = 0;
static volatile uint32_t sweep_cycles_completed = 0;
static volatile uint32_t last_full_sweep_ms = 0;
static volatile uint32_t wd_core1_last_ms = 0;
static uint32_t last_reported_scan_queue_drops = 0;
static uint32_t last_reported_dedupe_drops = 0;

// Controller-wide dedupe storage used to suppress duplicate results arriving
// from different scanner slots during the same operating session.
static constexpr uint16_t MASTER_DEDUPE_CAPACITY = WIFI_DEDUPE_TABLE_CAPACITY;
static WiFiDedupeHash master_dedupe_slots[WIFI_DEDUPE_TABLE_SIZE];
static WiFiDedupeHash master_dedupe_fifo[MASTER_DEDUPE_CAPACITY];
static WiFiDedupeTable master_dedupe_table = {};

// SD/logging retry policy remains owned by controller_main.cpp because the
// logging subsystem still lives here.
static constexpr uint8_t SD_INIT_MAX_ATTEMPTS = 5;
static constexpr uint16_t SD_INIT_RETRY_DELAY_MS = 500;
static constexpr uint16_t SD_RETRY_BACKGROUND_MS = 30000;
static constexpr uint32_t SD_SPI_CLOCKS_HZ[] = {SD_SPI_CLOCK, 4000000, 1000000, 400000}; // 10MHz init, then back off to more reliable speeds if needed.
static constexpr uint8_t RESET_BUTTON_DEBOUNCE_MS = 50;
static constexpr uint16_t CSV_LOG_TIME_WAIT_RETRY_MS = 1000;
static constexpr uint16_t CSV_LOG_FLUSH_INTERVAL_MS = 5000;

static pico_logging::State logging_state = {};
static const pico_logging::Config logging_config = {
  SD_PIN_CS,
  SD_SPI_CLOCKS_HZ,
  sizeof(SD_SPI_CLOCKS_HZ) / sizeof(SD_SPI_CLOCKS_HZ[0]),
  SD_INIT_RETRY_DELAY_MS,
  SD_RETRY_BACKGROUND_MS,
  CONTROLLER_GNSS_BOOT_TIMESTAMP_WAIT_MS,
  CONTROLLER_GNSS_BOOT_TIMESTAMP_READS,
  CONTROLLER_GPS_FIELD_MAX_AGE_MS,
  CSV_LOG_TIME_WAIT_RETRY_MS,
  CSV_LOG_FLUSH_INTERVAL_MS,
  CONTROLLER_GNSS_MIN_VALID_YEAR
};
static pico_logging::Logger logging(sd, logFile, gps, GNSS_UART, Serial, logging_config, logging_state);

// Normalize outgoing console text to CRLF so host tools on Linux and Windows
// render the controller console consistently.
static void streamWriteNormalized(Stream& stream, const char* text) {
  if (text == nullptr) {
    return;
  }

  char prev = '\0';
  while (*text != '\0') {
    const char c = *text++;
    if (c == '\n' && prev != '\r') {
      stream.write('\r');
    }
    stream.write(static_cast<uint8_t>(c));
    prev = c;
  }
}

static void serialPrintfNormalized(const char* fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  streamWriteNormalized(Serial, buf);
}

static void serialQueueTry(const char* text) {
  // core1 cannot safely own the USB console directly, so scanner diagnostics are
  // queued here and drained from loop() on core0.
  QueuedSerialMsg msg = {};
  strncpy(msg.text, text, sizeof(msg.text) - 1);
  if (!queue_try_add(&serial_msg_queue, &msg)) {
    serial_msg_drops++;
  }
}

static void handleResetButton() {
  // Keep the user reset path local to core0 so it can flush logs and the USB
  // console before triggering the watchdog reboot.
  static bool pressed_latched = false;
  static uint32_t pressed_ms = 0;
  const bool pressed = digitalRead(RESET_BUTTON_PIN) == LOW;

  if (!pressed) {
    pressed_latched = false;
    return;
  }

  if (!pressed_latched) {
    pressed_latched = true;
    pressed_ms = millis();
    return;
  }

  if ((millis() - pressed_ms) < RESET_BUTTON_DEBOUNCE_MS) {
    return;
  }

  Serial.println("Reset button pressed, rebooting Pico...");
  if (logFile) {
    logFile.flush();
  }
  Serial.flush();
  delay(20);
  watchdog_reboot(0, 0, 0);
  while (1) {
    delay(1);
  }
}

static void serialPrintRuntimeStatus() {
  // Periodic status combines queue health, logging totals, and scanner sweep
  // timing so field tuning can be done from one summary line.
  serialPrintfNormalized("Queue drops=%lu serial_drop=%lu dedupe_drop=%lu logged=%lu blank_gps=%lu sweeps=%lu last_sweep_ms=%lu\n",
                         (unsigned long)scan_queue_drops,
                         (unsigned long)serial_msg_drops,
                         (unsigned long)dedupe_drops,
                         (unsigned long)logging_state.csv_rows,
                         (unsigned long)logging_state.csv_rows_blank_gps,
                         (unsigned long)sweep_cycles_completed,
                         (unsigned long)last_full_sweep_ms);
}

static void reportScannerDropCounters() {
  if (scan_queue_drops != last_reported_scan_queue_drops) {
    serialPrintfNormalized("Warning: scan queue drops increased to %lu\n",
                           (unsigned long)scan_queue_drops);
    last_reported_scan_queue_drops = scan_queue_drops;
  }
  if (dedupe_drops != last_reported_dedupe_drops) {
    serialPrintfNormalized("Notice: dedupe drops increased to %lu\n",
                           (unsigned long)dedupe_drops);
    last_reported_dedupe_drops = dedupe_drops;
  }
}

static void printPeriodicStatus(bool usable_fix, bool usable_phone_fix) {
  static uint32_t lastStatMs = 0;
#if PERIODIC_STATUS_INTERVAL_MS == 0
  (void)usable_fix;
  (void)usable_phone_fix;
  return;
#else
  const uint32_t now = millis();
  if ((now - lastStatMs) <= PERIODIC_STATUS_INTERVAL_MS) {
    return;
  }
  lastStatMs = now;
  serialPrintRuntimeStatus();
  controllerGnssRuntimeSerialPrintStatus(gps, usable_fix, serialPrintfNormalized);
  serialPrintfNormalized("GPS(phone): usable=%s loc_valid=%s lat=%.7f lon=%.7f age=%lu chars=%lu\n",
                         usable_phone_fix ? "YES" : "NO",
                         gps_phone.location.isValid() ? "YES" : "NO",
                         gps_phone.location.isValid() ? gps_phone.location.lat() : 0.0,
                         gps_phone.location.isValid() ? gps_phone.location.lng() : 0.0,
                         (unsigned long)gps_phone.location.age(),
                         (unsigned long)gps_phone.charsProcessed());
#endif
}

static void maybeRequestDedupeResetOnFixAcquire(bool usable_fix) {
#if !CONTROLLER_DEDUPE_RESET_ON_FIX_ACQUIRE
  (void)usable_fix;
  return;
#else
  static bool last_usable_fix = false;
  static bool reset_requested_once = false;
  static uint32_t last_reset_request_ms = 0;

  const bool fix_acquired = usable_fix && !last_usable_fix;
  last_usable_fix = usable_fix;
  if (!fix_acquired) {
    return;
  }

  const uint32_t now = millis();
  const bool cooldown_active =
      reset_requested_once &&
      CONTROLLER_DEDUPE_RESET_COOLDOWN_MS > 0 &&
      ((int32_t)(now - last_reset_request_ms) <
       (int32_t)CONTROLLER_DEDUPE_RESET_COOLDOWN_MS);
  if (cooldown_active) {
    return;
  }

  controllerScannerRuntimeRequestDedupeReset();
  last_reset_request_ms = now;
  reset_requested_once = true;
  serialPrintfNormalized("GNSS usable fix acquired; requested dedupe reset (cooldown=%lums)\n",
                         (unsigned long)CONTROLLER_DEDUPE_RESET_COOLDOWN_MS);
#endif
}

void loop2() {
  // core1 only needs the shared runtime context and then hands control to the
  // scanner module permanently.
  const ControllerScannerRuntimeContext scanner_runtime = {
    &scan_result_queue,
    &scan_queue_drops,
    &dedupe_drops,
    &sweep_cycles_completed,
    &last_full_sweep_ms,
    &master_dedupe_table,
    serialQueueTry,
    &wd_core1_last_ms
  };
  controllerScannerRuntimeRun(scanner_runtime);
}

void setup() {
  // Bring up status LED first so bare-board bring-up has immediate feedback.
#if RGB_POWER_ENABLED
  pinMode(RGB_POWER_PIN, OUTPUT);
  digitalWrite(RGB_POWER_PIN, HIGH);
  delay(10);  // Let WS2812 power rail settle before first data frame.
#endif
  pixels.begin();
  pixels.setPixelColor(0, pixels.Color(128, 0, 0));
  pixels.show();

#if defined(USE_TINYUSB)
  TinyUSBDevice.setManufacturerDescriptor(USB_DEVICE_MANUFACTURER_NAME);
  TinyUSBDevice.setProductDescriptor(USB_DEVICE_PRODUCT_NAME);
  if (USB_DEVICE_SERIAL_OVERRIDE[0] != '\0') {
    TinyUSBDevice.setSerialDescriptor(USB_DEVICE_SERIAL_OVERRIDE);
  }
#endif
  Serial.begin(115200);
  controllerGnssRuntimeInitUsbPassthrough();
  // Give USB a moment to enumerate before the startup burst of status prints.
  delay(2000);

  Serial.println("WiFi Shuriken Startup");
  serialPrintfNormalized("RGB config: data=%d power_en=%d power_pin=%d\n",
                         RGB_PIN, RGB_POWER_ENABLED, RGB_POWER_PIN);
  logging.initUtcTimezone();
#if defined(USE_TINYUSB)
  serialPrintfNormalized("USB CDC IDs: CDC0='%s' CDC1='%s'\n",
                         USB_CDC0_IFACE_NAME, USB_CDC1_IFACE_NAME);
#endif
  logging.registerSdDateTimeCallback();
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  Serial.println("Reset button enabled on GPIO17");
  // Setup SD card on SPI0 before any logger recovery or CSV path selection.
  SPI.setSCK(SD_PIN_SCK);
  SPI.setTX(SD_PIN_MOSI);
  SPI.setRX(SD_PIN_MISO);
#if SD_MISO_PULLUP
  gpio_pull_up(SD_PIN_MISO);
#endif
  pinMode(SD_PIN_CS, OUTPUT);
  digitalWrite(SD_PIN_CS, HIGH);
  SPI.begin();
#if SD_MISO_PULLUP
  gpio_pull_up(SD_PIN_MISO);
  Serial.println("SD MISO internal pull-up enabled");
#endif

  // Initialize SD card with retries and fallback clocks.
  logging_state.sd_ready = logging.mountSdWithRetries(SD_INIT_MAX_ATTEMPTS);
  if (!logging_state.sd_ready) {
    Serial.println("SD initialization failed after retries; continuing without SD logging.");
    logging_state.sd_next_retry_ms = millis() + SD_RETRY_BACKGROUND_MS;
  } else {
    Serial.println("SD initialization done");
  }
  // Queue capacities are sized to tolerate short bursts from core1 while core0
  // is busy with SD or GNSS work.
  queue_init(&scan_result_queue, sizeof(QueuedScanResult), SCAN_RESULT_QUEUE_DEPTH);
  queue_init(&serial_msg_queue, sizeof(QueuedSerialMsg), SERIAL_MSG_QUEUE_DEPTH);
  wifiDedupeTableInit(&master_dedupe_table,
                      master_dedupe_slots, WIFI_DEDUPE_TABLE_SIZE,
                      master_dedupe_fifo, MASTER_DEDUPE_CAPACITY);
  wifiDedupeTableReset(&master_dedupe_table);

  // Initialize SPI1 for scanner communication
  controllerScannerRuntimeInitBus();

#if SCANNER_USE_SHIFTREG_CS
  serialPrintfNormalized("SPI1 initialized (shift-register scanner CS enabled, slots=%u).\n",
                         (unsigned)SCANNER_SHIFTREG_OUTPUTS);
#else
  Serial.println("SPI1 initialized.");
#endif
#if SCANNER_MISO_PULLUP
  Serial.println("Scanner MISO internal pull-up enabled");
#endif
  // Initialize GNSS UART
  controllerGnssRuntimeInitUart();

  // Configure GNSS module for 5Hz update rate and disable unnecessary NMEA sentences
  controllerGnssRuntimeSendPMTKCommand("$PMTK220,200*2C"); // Set update rate to 5Hz
  controllerGnssRuntimeSendPMTKCommand("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"); // Enable GGA and RMC sentences only

  // Capture RTC-backed GNSS date/time once per boot for the log filename.
  logging.captureBootTimestampFromGnss();

  // If SD is present, wait until the clock is usable before creating the CSV so
  // boot-time filenames are timestamped correctly.
  if (logging_state.sd_ready) {
    if (!logging.ensureBootTimestampFromClock()) {
      Serial.println("Waiting for valid GNSS time before creating CSV log file");
      logging_state.sd_next_retry_ms = millis() + CSV_LOG_TIME_WAIT_RETRY_MS;
    } else if (!logging.selectCsvLogPath()) {
      Serial.println("CSV log path selection failed");
      logging_state.sd_next_retry_ms = millis() + SD_RETRY_BACKGROUND_MS;
    } else {
      logging_state.csv_ready = logging.initCsvLog(logging_state.csv_log_path);
      if (logging_state.csv_ready) {
        Serial.print("CSV log ready: ");
        Serial.println(logging_state.csv_log_path);
      } else {
        Serial.print("CSV log open failed: ");
        Serial.println(logging_state.csv_log_path);
        logging_state.sd_next_retry_ms = millis() + SD_RETRY_BACKGROUND_MS;
      }
    }
  }

  // Set up second core loop for SPI communication with scanner
  multicore_launch_core1(loop2);

#if CONTROLLER_WATCHDOG_TIMEOUT_MS > 0
  // Pre-seed core1's heartbeat so the first loop() pass always has a valid
  // baseline regardless of how long core1 takes to start.
  wd_core1_last_ms = millis();
  watchdog_enable(CONTROLLER_WATCHDOG_TIMEOUT_MS, true);
  Serial.println("Watchdog enabled");
#endif
}

void loop() {
#if CONTROLLER_WATCHDOG_TIMEOUT_MS > 0
  {
    const uint32_t now = millis();
    // Only kick if core1 has checked in recently. Core0 being alive is
    // self-evident since it is running this code and feeding the watchdog.
    if ((now - wd_core1_last_ms) < (CONTROLLER_WATCHDOG_TIMEOUT_MS / 2)) {
      watchdog_update();
    }
  }
#endif
  handleResetButton();

  // Read GNSS data, update the fix LED, and compute the current "usable fix"
  // state that drives logging and periodic status.
  const bool usable_hw_fix = controllerGnssRuntimeService(gps, pixels, CONTROLLER_GPS_FIELD_MAX_AGE_MS);
  const bool usable_phone_fix = controllerPhoneGnssRuntimeService(gps_phone, CONTROLLER_GPS_FIELD_MAX_AGE_MS);
  const bool usable_fix = usable_hw_fix || usable_phone_fix;

  // Point the logger at whichever source has a valid fix. Hardware GPS takes
  // priority; phone GPS is the fallback when the module has no lock.
  logging.setGpsSource(usable_hw_fix ? &gps : &gps_phone);

  maybeRequestDedupeResetOnFixAcquire(usable_fix);
  logging.syncMasterClockFromGnss();

  // Drain core1 serial messages via queue (caps work per iteration).
  QueuedSerialMsg sm = {};
  int serial_drained = 0;
  while (serial_drained < CONTROLLER_SERIAL_MSG_DRAIN_PER_LOOP &&
         queue_try_remove(&serial_msg_queue, &sm)) {
    streamWriteNormalized(Serial, sm.text);
    serial_drained++;
  }

  // Drain AP results produced by core1 and handle logging on core0.
  QueuedScanResult q = {};
  // Limit per-iteration SD writes so GNSS parsing is serviced frequently.
  int drained = 0;
  while (drained < CONTROLLER_SCAN_RESULT_DRAIN_PER_LOOP &&
         queue_try_remove(&scan_result_queue, &q)) {
    logging.appendCsvRow(q);
    drained++;
  }
  reportScannerDropCounters();
  // Time-based flush closes the durability gap between row-count flushes.
  logging.flushCsvIfDue();

  if (!logging_state.csv_ready) {
    logging.tryRecoverSdLogging();
  }

  printPeriodicStatus(usable_hw_fix, usable_phone_fix);
}
