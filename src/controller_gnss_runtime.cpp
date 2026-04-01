#include "controller_gnss_runtime.h"

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
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

static void updateGpsFixLed(Adafruit_NeoPixel& pixels, bool usable_fix) {
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

void controllerGnssRuntimeSendPMTKCommand(const char* cmd) {
  GNSS_UART.println(cmd);
  Serial.print("Sent PMTK Command: ");
  Serial.println(cmd);
  delay(100); // Short delay to ensure command is processed
}

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
  }

  GNSS_UART.end();
  gnssUartBeginWithConfiguredBuffer(CONTROLLER_GNSS_TARGET_BAUD);
  Serial.print("GNSS UART configured to ");
  Serial.print(CONTROLLER_GNSS_TARGET_BAUD);
  Serial.print(" baud (FIFO=");
  Serial.print((unsigned)SERIAL_BUFFER_SIZE);
  Serial.println(").");
}

void controllerGnssRuntimeInitUart() {
  GNSS_UART.setTX(GNSS_TX);
  GNSS_UART.setRX(GNSS_RX);
  configureGnssBaudToTarget();
}

bool controllerGnssRuntimeService(TinyGPSPlus& gps,
                                  Adafruit_NeoPixel& pixels,
                                  uint16_t gps_field_max_age_ms) {
  // Keep the GNSS UART drained aggressively so TinyGPS++ parsing and optional
  // NMEA mirroring do not fall behind while core0 is also handling SD writes.
  while (GNSS_UART.available()) {
    char c = GNSS_UART.read();
    gps.encode(c);

#if NMEA_PASSTHROUGH
    nmeaPassthroughWriteChar(c);
#endif
  }

  // The controller only considers a fix usable when location is fresh and at
  // least a minimal satellite lock is present.
  const bool usable_fix =
      gps.location.isValid() &&
      gps.location.age() < gps_field_max_age_ms &&
      gps.satellites.isValid() &&
      gps.satellites.value() >= 3;
  updateGpsFixLed(pixels, usable_fix);
  return usable_fix;
}

bool controllerPhoneGnssRuntimeService(TinyGPSPlus& gps_phone,
                                       uint16_t gps_field_max_age_ms) {
#if defined(USE_TINYUSB)
  // Drain NMEA sentences arriving from the Android app on the shared NMEA CDC
  // port. The app sends standard NMEA (GPRMC/GPGGA) which TinyGPS++ parses
  // the same way it does hardware output.
  while (nmea_passthrough_cdc.available()) {
    gps_phone.encode(static_cast<char>(nmea_passthrough_cdc.read()));
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
