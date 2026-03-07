#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <TinyGPSPlus.h>
#include <SdFat.h>
#if defined(USE_TINYUSB)
#include <Adafruit_TinyUSB.h>
#include "class/cdc/cdc_device.h"
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
#include "controller_scanner_runtime.h"

static constexpr uint32_t GNSS_TARGET_BAUD = 115200;

SdFat sd;
FsFile logFile;
Adafruit_NeoPixel pixels(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);
TinyGPSPlus gps;
#if defined(USE_TINYUSB)
static Adafruit_USBD_CDC nmea_passthrough_cdc;
static constexpr uint8_t NMEA_PASSTHROUGH_CDC_INSTANCE = 1;
#endif
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
static constexpr uint16_t GPS_FIELD_MAX_AGE_MS = 3000;

static constexpr uint16_t MASTER_DEDUPE_CAPACITY = WIFI_DEDUPE_TABLE_CAPACITY;
static WiFiDedupeHash master_dedupe_storage[MASTER_DEDUPE_CAPACITY];
static WiFiDedupeTable master_dedupe_table = {};
static constexpr uint8_t SD_INIT_MAX_ATTEMPTS = 5;
static constexpr uint16_t SD_INIT_RETRY_DELAY_MS = 500;
static constexpr uint16_t SD_RETRY_BACKGROUND_MS = 30000;
static constexpr uint32_t SD_SPI_CLOCKS_HZ[] = {SD_SPI_CLOCK, 4000000, 1000000, 400000}; // 10MHz init, then back off to more reliable speeds if needed.
static constexpr uint16_t GNSS_BOOT_TIMESTAMP_WAIT_MS = 6000;
static constexpr uint8_t GNSS_BOOT_TIMESTAMP_READS = 5;
static constexpr uint8_t RESET_BUTTON_DEBOUNCE_MS = 50;
static constexpr uint16_t CSV_LOG_TIME_WAIT_RETRY_MS = 1000;
static constexpr uint16_t CSV_LOG_FLUSH_INTERVAL_MS = 5000;
static constexpr uint16_t GNSS_MIN_VALID_YEAR = 2026;

static pico_logging::State logging_state = {};
static const pico_logging::Config logging_config = {
  SD_PIN_CS,
  SD_SPI_CLOCKS_HZ,
  sizeof(SD_SPI_CLOCKS_HZ) / sizeof(SD_SPI_CLOCKS_HZ[0]),
  SD_INIT_RETRY_DELAY_MS,
  SD_RETRY_BACKGROUND_MS,
  GNSS_BOOT_TIMESTAMP_WAIT_MS,
  GNSS_BOOT_TIMESTAMP_READS,
  GPS_FIELD_MAX_AGE_MS,
  CSV_LOG_TIME_WAIT_RETRY_MS,
  CSV_LOG_FLUSH_INTERVAL_MS,
  GNSS_MIN_VALID_YEAR
};
static pico_logging::Logger logging(sd, logFile, gps, GNSS_UART, Serial, logging_config, logging_state);

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
  QueuedSerialMsg msg = {};
  strncpy(msg.text, text, sizeof(msg.text) - 1);
  if (!queue_try_add(&serial_msg_queue, &msg)) {
    serial_msg_drops++;
  }
}

static inline void nmeaPassthroughWriteChar(char c) {
#if !NMEA_PASSTHROUGH
  (void)c;
#elif defined(USE_TINYUSB)
  // Bypass the Arduino CDC wrapper here because it drops writes unless the
  // host asserts DTR. gpsd often opens the tty without doing that.
  if (tud_cdc_n_ready(NMEA_PASSTHROUGH_CDC_INSTANCE)) {
    tud_cdc_n_write_char(NMEA_PASSTHROUGH_CDC_INSTANCE, c);
    if (c == '\n') {
      tud_cdc_n_write_flush(NMEA_PASSTHROUGH_CDC_INSTANCE);
    }
  }
#else
  Serial.write(static_cast<uint8_t>(c));
#endif
}

static void updateGpsFixLed(bool usable_fix) {
  static int8_t last_state = -1;
  const int8_t state = usable_fix ? 1 : 0;
  if (state == last_state) {
    return;
  }
  last_state = state;

  const uint32_t color = usable_fix ? pixels.Color(0, 96, 0) : pixels.Color(75, 20, 0);
  pixels.setPixelColor(0, color);
  pixels.show();
}

void sendPMTKCommand(const char* cmd) {
  GNSS_UART.println(cmd);
  Serial.print("Sent PMTK Command: ");
  Serial.println(cmd);
  delay(100); // Short delay to ensure command is processed
}

static void gnssUartBeginWithConfiguredBuffer(uint32_t baud) {
  static bool fifo_warned = false;
  if (!GNSS_UART.setFIFOSize(SERIAL_BUFFER_SIZE) && !fifo_warned) {
    Serial.print("Warning: GNSS UART FIFO resize failed, keeping default. requested=");
    Serial.println((unsigned)SERIAL_BUFFER_SIZE);
    fifo_warned = true;
  }
  GNSS_UART.begin(baud);
}

static void configureGnssBaudToTarget() {
  static const uint32_t probe_bauds[] = {115200, 9600, 38400};

  // Try the baud-rate switch command at common existing rates so legacy and
  // newer modules both converge to GNSS_TARGET_BAUD.
  for (size_t i = 0; i < (sizeof(probe_bauds) / sizeof(probe_bauds[0])); i++) {
    GNSS_UART.end();
    gnssUartBeginWithConfiguredBuffer(probe_bauds[i]);
    delay(30);
    GNSS_UART.println("$PMTK251,115200*1F");
    GNSS_UART.flush();
    delay(80);
  }

  GNSS_UART.end();
  gnssUartBeginWithConfiguredBuffer(GNSS_TARGET_BAUD);
  Serial.print("GNSS UART configured to ");
  Serial.print(GNSS_TARGET_BAUD);
  Serial.print(" baud (FIFO=");
  Serial.print((unsigned)SERIAL_BUFFER_SIZE);
  Serial.println(").");
}

static void handleResetButton() {
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
  serialPrintfNormalized("Queue drops=%lu serial_drop=%lu dedupe_drop=%lu logged=%lu blank_gps=%lu sweeps=%lu last_sweep_ms=%lu\n",
                         (unsigned long)scan_queue_drops,
                         (unsigned long)serial_msg_drops,
                         (unsigned long)dedupe_drops,
                         (unsigned long)logging_state.csv_rows,
                         (unsigned long)logging_state.csv_rows_blank_gps,
                         (unsigned long)sweep_cycles_completed,
                         (unsigned long)last_full_sweep_ms);
}

static void serialPrintGpsStatus(bool usable_fix) {
  serialPrintfNormalized("GPS: usable=%s loc_valid=%s updated=%s lat=%.7f lon=%.7f hdop=%.2f sats=%u age=%lu chars=%lu fix=%lu cksum_fail=%lu\n",
                         usable_fix ? "YES" : "NO",
                         gps.location.isValid() ? "YES" : "NO",
                         gps.location.isUpdated() ? "YES" : "NO",
                         gps.location.isValid() ? gps.location.lat() : 0.0,
                         gps.location.isValid() ? gps.location.lng() : 0.0,
                         gps.hdop.isValid() ? gps.hdop.hdop() : 0.0,
                         gps.satellites.isValid() ? gps.satellites.value() : 0,
                         (unsigned long)gps.location.age(),
                         (unsigned long)gps.charsProcessed(),
                         (unsigned long)gps.sentencesWithFix(),
                         (unsigned long)gps.failedChecksum());
}

static void printPeriodicStatus(bool usable_fix) {
  static uint32_t lastStatMs = 0;
#if PERIODIC_STATUS_INTERVAL_MS == 0
  (void)usable_fix;
  return;
#else
  const uint32_t now = millis();
  if ((now - lastStatMs) <= PERIODIC_STATUS_INTERVAL_MS) {
    return;
  }
  lastStatMs = now;
  serialPrintRuntimeStatus();
  serialPrintGpsStatus(usable_fix);
#endif
}

void loop2() {
  const ControllerScannerRuntimeContext scanner_runtime = {
    &scan_result_queue,
    &scan_queue_drops,
    &dedupe_drops,
    &sweep_cycles_completed,
    &last_full_sweep_ms,
    &master_dedupe_table,
    serialQueueTry
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
#if defined(USE_TINYUSB)
  nmea_passthrough_cdc.begin(115200);
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }
#endif
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
  // Setup SD Card on SPI0
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
  queue_init(&scan_result_queue, sizeof(QueuedScanResult), 128);
  queue_init(&serial_msg_queue, sizeof(QueuedSerialMsg), 64);
  wifiDedupeTableInit(&master_dedupe_table, master_dedupe_storage, MASTER_DEDUPE_CAPACITY);
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
  GNSS_UART.setTX(GNSS_TX);
  GNSS_UART.setRX(GNSS_RX);
  configureGnssBaudToTarget();

  // Configure GNSS module for 5Hz update rate and disable unnecessary NMEA sentences
  sendPMTKCommand("$PMTK220,200*2C"); // Set update rate to 5Hz
  sendPMTKCommand("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"); // Enable GGA and RMC sentences only

  // Capture RTC-backed GNSS date/time once per boot for the log filename.
  logging.captureBootTimestampFromGnss();

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

}

void loop() {
  handleResetButton();

  // Read GNSS data and update GPS info.
  while (GNSS_UART.available()) {
    char c = GNSS_UART.read();
    gps.encode(c);

#if NMEA_PASSTHROUGH
    nmeaPassthroughWriteChar(c);
#endif
  }
  logging.syncMasterClockFromGnss();

  const bool usable_fix =
      gps.location.isValid() &&
      gps.location.age() < GPS_FIELD_MAX_AGE_MS &&
      gps.satellites.isValid() &&
      gps.satellites.value() >= 3;
  updateGpsFixLed(usable_fix);

  // Drain core1 serial messages via queue (caps work per iteration).
  QueuedSerialMsg sm = {};
  int serial_drained = 0;
  while (serial_drained < 16 && queue_try_remove(&serial_msg_queue, &sm)) {
    streamWriteNormalized(Serial, sm.text);
    serial_drained++;
  }

  // Drain AP results produced by core1 and handle logging on core0.
  QueuedScanResult q = {};
  // Limit per-iteration SD writes so GNSS parsing is serviced frequently.
  int drained = 0;
  while (drained < 16 && queue_try_remove(&scan_result_queue, &q)) {
    logging.appendCsvRow(q);
    drained++;
  }
  // Time-based flush closes the durability gap between row-count flushes.
  logging.flushCsvIfDue();

  if (!logging_state.csv_ready) {
    logging.tryRecoverSdLogging();
  }

  printPeriodicStatus(usable_fix);
}
