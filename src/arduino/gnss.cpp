#include "arduino/gnss.h"
#include "hal/platform.h"

#include <Arduino.h>

#if defined(USE_TINYUSB)
#include <Adafruit_TinyUSB.h>
#include "class/cdc/cdc_device.h"
#endif

#include "Configuration.h"
#if defined(SHURIKEN_P4_CONTROLLER) && P4_USB_NMEA
#include "../p4/usb_nmea.h"
#endif

// This module owns controller-side GNSS plumbing: UART bring-up, PMTK setup,
// raw NMEA passthrough, TinyGPS++ feeding, and the GPS fix LED.

#if defined(USE_TINYUSB)
// Secondary CDC interface used exclusively for raw NMEA passthrough.
static Adafruit_USBD_CDC nmea_passthrough_cdc;
static constexpr uint8_t NMEA_PASSTHROUGH_CDC_INSTANCE = 1;
#endif

void controllerGnssRuntimeInitUsbPassthrough() {
#if defined(SHURIKEN_P4_CONTROLLER) && P4_USB_NMEA
  p4::beginUsbNmea();
#elif defined(USE_TINYUSB)
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
#elif defined(SHURIKEN_P4_CONTROLLER) && P4_USB_NMEA
  p4::mirrorUsbNmea(static_cast<uint8_t>(c));
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
#if defined(SHURIKEN_P4_CONTROLLER)
  const bool resized = GNSS_UART.setRxBufferSize(SERIAL_BUFFER_SIZE) != 0;
#else
  const bool resized = GNSS_UART.setFIFOSize(SERIAL_BUFFER_SIZE);
#endif
  if (!resized && !fifo_warned) {
    Serial.print("Warning: GNSS UART FIFO resize failed, keeping default. requested=");
    Serial.println((unsigned)SERIAL_BUFFER_SIZE);
    fifo_warned = true;
  }
#if defined(SHURIKEN_P4_CONTROLLER)
  GNSS_UART.begin(baud, SERIAL_8N1, GNSS_RX, GNSS_TX);
#else
  GNSS_UART.begin(baud);
#endif
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
#if CONTROLLER_GNSS_TARGET_BAUD == 115200
    // The trailing *1F is a checksum over "PMTK251,115200", so this sentence is
    // only valid at the default target. At any other target baud it would be a
    // malformed command, and MTK modules are simply left at their current rate.
    GNSS_UART.println("$PMTK251,115200*1F");
    GNSS_UART.flush();
    delay(80);
#endif
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
#if !defined(SHURIKEN_P4_CONTROLLER)
  GNSS_UART.setTX(GNSS_TX);
  GNSS_UART.setRX(GNSS_RX);
#endif
  configureGnssBaudToTarget();
}

static void configureGnssModuleOutput() {
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
}

void controllerGnssRuntimeBegin() {
  controllerGnssRuntimeInitUart();
  configureGnssModuleOutput();
}

controller::hal::ByteStream* controllerUsbNmeaStream() {
#if defined(SHURIKEN_P4_CONTROLLER) && P4_USB_NMEA
  return &p4::usbNmeaStream();
#elif defined(USE_TINYUSB)
  class UsbNmeaStream final : public controller::hal::ByteStream {
   public:
    int read(uint8_t* data, size_t capacity) override {
      size_t count = 0;
      while (count < capacity && nmea_passthrough_cdc.available()) {
        const int value = nmea_passthrough_cdc.read();
        if (value < 0) break;
        data[count++] = static_cast<uint8_t>(value);
      }
      return static_cast<int>(count);
    }
    int write(const uint8_t*, size_t) override { return -1; }
  };
  static UsbNmeaStream stream;
  return &stream;
#else
  return nullptr;
#endif
}
void controllerMirrorNmea(uint8_t byte) { nmeaPassthroughWriteChar(static_cast<char>(byte)); }
