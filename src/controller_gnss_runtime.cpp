#include "controller_gnss_runtime.h"

#include <Arduino.h>
#include <TinyGPSPlus.h>

#if defined(USE_TINYUSB)
#include <Adafruit_TinyUSB.h>
#include "class/cdc/cdc_device.h"
#endif

#include "Configuration.h"

// This module owns controller-side GNSS plumbing: UART bring-up, PMTK setup,
// raw NMEA passthrough, TinyGPS++ feeding, and the GPS fix LED.

#if defined(USE_TINYUSB)
// Secondary CDC interface used exclusively for raw NMEA passthrough.
static Adafruit_USBD_CDC nmea_passthrough_cdc;
static constexpr uint8_t NMEA_PASSTHROUGH_CDC_INSTANCE = 1;
#endif

void controllerGnssRuntimeInitUsbPassthrough() {
#if defined(USE_TINYUSB)
  // Reattach after begin() so descriptor changes applied in setup() are visible
  // on hosts that enumerate immediately.
  nmea_passthrough_cdc.begin(115200);
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }
#endif
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


// Silences the per-command console prints while a re-init runs, so a module
// that never answers cannot spam the console once per retry interval.
static bool gnss_config_quiet = false;

void controllerGnssRuntimeSendPMTKCommand(const char* cmd) {
  GNSS_UART.println(cmd);
  if (!gnss_config_quiet) {
    Serial.print("Sent PMTK Command: ");
    Serial.println(cmd);
  }
  delay(100); // Short delay to ensure command is processed
}

#if CONTROLLER_GNSS_UBLOX_CONFIG
// u-blox binary (UBX) configuration.
//
// u-blox modules ignore both the MTK ($PMTK) and Quectel ($PAIR) sentences
// above, so without this block a u-blox module is only usable if it happens to
// already be emitting NMEA at the target baud. TBS-branded M8 modules in
// particular are commonly shipped configured for UBX binary output at 9600 for
// Betaflight, which TinyGPS++ cannot parse at all.
static constexpr uint8_t UBX_SYNC_CHAR_1 = 0xB5;
static constexpr uint8_t UBX_SYNC_CHAR_2 = 0x62;
static constexpr uint8_t UBX_CLASS_CFG = 0x06;
static constexpr uint8_t UBX_CFG_PRT = 0x00;
static constexpr uint8_t UBX_CFG_MSG = 0x01;
static constexpr uint8_t UBX_CFG_RATE = 0x08;
static constexpr uint8_t UBX_CFG_CFG = 0x09;

// NMEA standard message IDs within UBX message class 0xF0.
static constexpr uint8_t UBX_NMEA_MSG_CLASS = 0xF0;
static constexpr uint8_t UBX_NMEA_ID_GGA = 0x00;
static constexpr uint8_t UBX_NMEA_ID_GLL = 0x01;
static constexpr uint8_t UBX_NMEA_ID_GSA = 0x02;
static constexpr uint8_t UBX_NMEA_ID_GSV = 0x03;
static constexpr uint8_t UBX_NMEA_ID_RMC = 0x04;
static constexpr uint8_t UBX_NMEA_ID_VTG = 0x05;

static void ubxSend(uint8_t msg_class,
                    uint8_t msg_id,
                    const uint8_t* payload,
                    uint16_t length) {
  const uint8_t header[6] = {
    UBX_SYNC_CHAR_1,
    UBX_SYNC_CHAR_2,
    msg_class,
    msg_id,
    (uint8_t)(length & 0xFFu),
    (uint8_t)((length >> 8) & 0xFFu)
  };

  // 8-bit Fletcher checksum over everything except the two sync characters.
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;
  for (size_t i = 2; i < sizeof(header); i++) {
    ck_a = (uint8_t)(ck_a + header[i]);
    ck_b = (uint8_t)(ck_b + ck_a);
  }
  for (uint16_t i = 0; i < length; i++) {
    ck_a = (uint8_t)(ck_a + payload[i]);
    ck_b = (uint8_t)(ck_b + ck_a);
  }
  const uint8_t checksum[2] = {ck_a, ck_b};

  GNSS_UART.write(header, sizeof(header));
  if (length > 0 && payload != nullptr) {
    GNSS_UART.write(payload, length);
  }
  GNSS_UART.write(checksum, sizeof(checksum));
  GNSS_UART.flush();
  delay(20);  // Let the receiver consume the frame before the next one.
}

// UBX-CFG-PRT for UART1: set the port baud and force NMEA-only output.
//
// Output is restricted to NMEA so the raw passthrough CDC carries a clean
// sentence stream for gpsd instead of NMEA interleaved with UBX binary. Input
// still accepts UBX so this configuration path keeps working on the next boot.
static void ubxConfigurePortBaud(uint32_t baud) {
  uint8_t payload[20] = {};
  payload[0] = 0x01;  // portID: UART1.
  payload[4] = 0xD0;  // mode: 8 data bits, no parity, 1 stop bit.
  payload[5] = 0x08;
  payload[8] = (uint8_t)(baud & 0xFFu);
  payload[9] = (uint8_t)((baud >> 8) & 0xFFu);
  payload[10] = (uint8_t)((baud >> 16) & 0xFFu);
  payload[11] = (uint8_t)((baud >> 24) & 0xFFu);
  payload[12] = 0x03;  // inProtoMask: accept UBX and NMEA.
  payload[14] = 0x02;  // outProtoMask: emit NMEA only.
  ubxSend(UBX_CLASS_CFG, UBX_CFG_PRT, payload, sizeof(payload));
}

// UBX-CFG-RATE: navigation solution period in milliseconds.
static void ubxConfigureRate(uint16_t measurement_period_ms) {
  uint8_t payload[6] = {};
  payload[0] = (uint8_t)(measurement_period_ms & 0xFFu);
  payload[1] = (uint8_t)((measurement_period_ms >> 8) & 0xFFu);
  payload[2] = 0x01;  // navRate: one solution per measurement cycle.
  payload[4] = 0x01;  // timeRef: align to GPS time.
  ubxSend(UBX_CLASS_CFG, UBX_CFG_RATE, payload, sizeof(payload));
}

// UBX-CFG-MSG (short form): set an NMEA sentence rate on the current port.
static void ubxSetNmeaMessageRate(uint8_t nmea_msg_id, uint8_t rate) {
  const uint8_t payload[3] = {UBX_NMEA_MSG_CLASS, nmea_msg_id, rate};
  ubxSend(UBX_CLASS_CFG, UBX_CFG_MSG, payload, sizeof(payload));
}

#if CONTROLLER_GNSS_UBLOX_SAVE_CONFIG
// UBX-CFG-CFG: persist the current configuration to BBR, flash, and EEPROM.
static void ubxSaveConfiguration() {
  uint8_t payload[13] = {};
  // clearMask stays zero; save every configuration section.
  payload[4] = 0xFF;
  payload[5] = 0xFF;
  // loadMask stays zero.
  payload[12] = 0x17;  // deviceMask: BBR, flash, EEPROM, SPI flash.
  ubxSend(UBX_CLASS_CFG, UBX_CFG_CFG, payload, sizeof(payload));
}
#endif

static void configureUbloxModuleOutput(bool save_config) {
  // Match the MTK/Quectel configuration above: 5Hz, GGA and RMC only. GGA is
  // required because the usable-fix test needs a satellite count.
  ubxConfigureRate(200);
  ubxSetNmeaMessageRate(UBX_NMEA_ID_GGA, 1);
  ubxSetNmeaMessageRate(UBX_NMEA_ID_GLL, 0);
  ubxSetNmeaMessageRate(UBX_NMEA_ID_GSA, CONTROLLER_GNSS_NMEA_DEBUG_SENTENCES);
  ubxSetNmeaMessageRate(UBX_NMEA_ID_GSV, CONTROLLER_GNSS_NMEA_DEBUG_SENTENCES);
  ubxSetNmeaMessageRate(UBX_NMEA_ID_RMC, 1);
  ubxSetNmeaMessageRate(UBX_NMEA_ID_VTG, 0);
  if (!gnss_config_quiet) {
    Serial.println("Sent UBX configuration: 5Hz, NMEA GGA+RMC only");
  }
#if CONTROLLER_GNSS_UBLOX_SAVE_CONFIG
  // Only the boot pass saves. Persisting on every silent-module retry would
  // rewrite the module's flash indefinitely whenever one is disconnected.
  if (save_config) {
    ubxSaveConfiguration();
    Serial.println("Sent UBX save-configuration command");
  }
#else
  (void)save_config;
#endif
}
#endif  // CONTROLLER_GNSS_UBLOX_CONFIG

static void gnssUartBeginWithConfiguredBuffer(uint32_t baud) {
  static bool fifo_warned = false;
  // The Arduino-Pico FIFO resize can fail on some targets; warn once and keep
  // going with the default buffer instead of spamming the console.
  if (!GNSS_UART.setFIFOSize(SERIAL_BUFFER_SIZE) && !fifo_warned) {
    Serial.print("Warning: GNSS UART FIFO resize failed, keeping default. requested=");
    Serial.println((unsigned)SERIAL_BUFFER_SIZE);
    fifo_warned = true;
  }
  GNSS_UART.begin(baud);
}

static void configureGnssBaudToTarget() {
  static const uint32_t probe_bauds[] = {115200, 9600, 38400};

  // Some modules may already be at the target rate while others still power up
  // at older defaults. Probe the common cases and always finish at the target.
  // Try the baud-rate switch command at common existing rates so legacy and
  // newer modules both converge to CONTROLLER_GNSS_TARGET_BAUD.
  for (size_t i = 0; i < (sizeof(probe_bauds) / sizeof(probe_bauds[0])); i++) {
    GNSS_UART.end();
    gnssUartBeginWithConfiguredBuffer(probe_bauds[i]);
    delay(30);
    GNSS_UART.println("$PMTK251,115200*1F");
    GNSS_UART.flush();
    delay(80);
#if CONTROLLER_GNSS_UBLOX_CONFIG
    // u-blox modules ignore $PMTK, so drive them to the same target with
    // UBX-CFG-PRT. A module already at the target baud simply reapplies it.
    ubxConfigurePortBaud(CONTROLLER_GNSS_TARGET_BAUD);
    delay(80);
#endif
  }

  GNSS_UART.end();
  gnssUartBeginWithConfiguredBuffer(CONTROLLER_GNSS_TARGET_BAUD);
  if (!gnss_config_quiet) {
    Serial.print("GNSS UART configured to ");
    Serial.print(CONTROLLER_GNSS_TARGET_BAUD);
    Serial.print(" baud (FIFO=");
    Serial.print((unsigned)SERIAL_BUFFER_SIZE);
    Serial.println(").");
  }
}

void controllerGnssRuntimeInitUart() {
  GNSS_UART.setTX(GNSS_TX);
  GNSS_UART.setRX(GNSS_RX);
  configureGnssBaudToTarget();
}

static void configureGnssModuleOutput(bool save_config) {
  // Configure MTK-compatible modules for 5Hz updates with only GGA/RMC output.
  controllerGnssRuntimeSendPMTKCommand("$PMTK220,200*2C");
  controllerGnssRuntimeSendPMTKCommand("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28");

  // Configure LC86G modules for the same 5Hz GGA/RMC-only output.
  controllerGnssRuntimeSendPMTKCommand("$PAIR050,200*21");
  controllerGnssRuntimeSendPMTKCommand("$PAIR062,0,1*3F"); // GGA at 1x rate.
  controllerGnssRuntimeSendPMTKCommand("$PAIR062,1,0*3F"); // GLL off.
  controllerGnssRuntimeSendPMTKCommand("$PAIR062,2,0*3C"); // GSA off.
  controllerGnssRuntimeSendPMTKCommand("$PAIR062,3,0*3D"); // GSV off.
  controllerGnssRuntimeSendPMTKCommand("$PAIR062,4,1*3B"); // RMC at 1x rate.
  controllerGnssRuntimeSendPMTKCommand("$PAIR062,5,0*3B"); // VTG off.

#if CONTROLLER_GNSS_UBLOX_CONFIG
  // Configure u-blox modules for the same 5Hz GGA/RMC-only output.
  configureUbloxModuleOutput(save_config);
#else
  (void)save_config;
#endif
}

#if CONTROLLER_GNSS_RECONFIG_IDLE_MS > 0
// Timestamp of the last byte seen on the GNSS UART, and the silence the module
// must exceed before the next re-init. Both are plain millis() values compared
// with unsigned subtraction so the rollover at ~49 days needs no special case.
static uint32_t gnss_last_rx_ms = 0;
static uint32_t gnss_reconfig_idle_ms = CONTROLLER_GNSS_RECONFIG_IDLE_MS;

// Re-run the full bring-up, including the baud sweep. The sweep is what
// recovers a module that reverted to its factory rate, and it is only reachable
// here because the module is already silent -- there is no live NMEA to lose.
static void gnssReconfigureAfterSilence() {
  // Runs on core0 from loop(), so touching Serial directly is safe here.
  Serial.print("GNSS silent for ");
  Serial.print((unsigned long)(millis() - gnss_last_rx_ms));
  Serial.print("ms; re-running bring-up (retry interval ");
  Serial.print((unsigned long)gnss_reconfig_idle_ms);
  Serial.println("ms)");

  gnss_config_quiet = true;
  controllerGnssRuntimeInitUart();
  configureGnssModuleOutput(false);
  gnss_config_quiet = false;

  // Back off after an attempt that has not yet proven the module is alive. A
  // byte arriving in service() resets this to the configured floor.
  if (gnss_reconfig_idle_ms < CONTROLLER_GNSS_RECONFIG_MAX_IDLE_MS) {
    const uint32_t next = gnss_reconfig_idle_ms * 2;
    gnss_reconfig_idle_ms = (next > CONTROLLER_GNSS_RECONFIG_MAX_IDLE_MS)
                                ? (uint32_t)CONTROLLER_GNSS_RECONFIG_MAX_IDLE_MS
                                : next;
  }

  // Re-read the clock: the sequence above blocks for well over a second, and
  // the next window should be measured from when it finished.
  gnss_last_rx_ms = millis();
}
#endif

void controllerGnssRuntimeBegin() {
  controllerGnssRuntimeInitUart();
  configureGnssModuleOutput(true);
#if CONTROLLER_GNSS_RECONFIG_IDLE_MS > 0
  gnss_last_rx_ms = millis();
#endif
}

bool controllerGnssRuntimeService(TinyGPSPlus& gps,
                                  uint16_t gps_field_max_age_ms) {
  // Keep the GNSS UART drained aggressively so TinyGPS++ parsing and optional
  // NMEA mirroring do not fall behind while core0 is also handling SD writes.
  bool rx_any = false;
  while (GNSS_UART.available()) {
    char c = GNSS_UART.read();
    gps.encode(c);
    rx_any = true;

#if NMEA_PASSTHROUGH
    nmeaPassthroughWriteChar(c);
#endif
  }

#if CONTROLLER_GNSS_RECONFIG_IDLE_MS > 0
  // Any byte at all counts as alive, including the binary a factory-reverted
  // module emits: the point is to distinguish a module that is talking from one
  // that is not there. Parsing problems are a separate failure with their own
  // symptom (chars climbing while cksum_fail rises).
  const uint32_t now_ms = millis();
  if (rx_any) {
    gnss_last_rx_ms = now_ms;
    gnss_reconfig_idle_ms = CONTROLLER_GNSS_RECONFIG_IDLE_MS;
  } else if ((uint32_t)(now_ms - gnss_last_rx_ms) >= gnss_reconfig_idle_ms) {
    gnssReconfigureAfterSilence();
  }
#else
  (void)rx_any;
#endif

  // The controller only considers a fix usable when location is fresh and at
  // least a minimal satellite lock is present.
  const bool usable_fix =
      gps.location.isValid() &&
      gps.location.age() < gps_field_max_age_ms &&
      gps.satellites.isValid() &&
      gps.satellites.value() >= 3;
  return usable_fix;
}

bool controllerPhoneGnssRuntimeService(TinyGPSPlus& gps_phone,
                                       uint16_t gps_field_max_age_ms) {
#if defined(USE_TINYUSB)
  // Drain NMEA sentences arriving from the Android app on the shared NMEA CDC
  // port. The app sends standard NMEA (GPRMC/GPGGA) which TinyGPS++ parses
  // the same way it does hardware output. Keep this bounded so a continuously
  // replenished USB RX buffer cannot prevent core0 from returning to loop()
  // and feeding the watchdog. TinyGPS++ retains partial sentences between calls.
  static constexpr uint16_t PHONE_NMEA_DRAIN_BUDGET = 256;
  uint16_t bytes_drained = 0;
  while (bytes_drained < PHONE_NMEA_DRAIN_BUDGET &&
         nmea_passthrough_cdc.available()) {
    gps_phone.encode(static_cast<char>(nmea_passthrough_cdc.read()));
    bytes_drained++;
  }
#endif

  // Accept the phone fix if location is present and fresh. Satellite count is
  // not required since some apps omit GGA and only send RMC.
  return gps_phone.location.isValid() &&
         gps_phone.location.age() < gps_field_max_age_ms;
}

void controllerGnssRuntimeSerialPrintStatus(TinyGPSPlus& gps,
                                            bool usable_fix,
                                            ControllerSerialPrintfFn serial_printf_normalized) {
  // Keep formatting centralized here so controller_main.cpp only decides when
  // status should be emitted, not how GNSS fields are rendered.
  serial_printf_normalized("GPS: usable=%s loc_valid=%s updated=%s lat=%.7f lon=%.7f hdop=%.2f sats=%u age=%lu chars=%lu fix=%lu cksum_fail=%lu\n",
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
