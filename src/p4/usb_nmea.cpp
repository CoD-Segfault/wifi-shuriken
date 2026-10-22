#include "Configuration.h"
#if defined(SHURIKEN_P4_CONTROLLER) && P4_USB_NMEA
#include "usb_nmea.h"
#include <USBCDC.h>
#include "class/cdc/cdc_device.h"
#if ARDUINO_USB_MODE || !ARDUINO_USB_CDC_ON_BOOT
#error "P4 USB NMEA requires native TinyUSB mode and CDC0 on boot"
#endif
static_assert(CFG_TUD_CDC >= 2, "The P4 SDK must support at least two CDC interfaces");
namespace p4 {
namespace {
constexpr uint8_t nmeaInterface = 1;
// Construction registers CDC1's descriptor before Arduino starts USB in app_main.
// Do not call begin(): the SDK's RX callback can wait on a full FreeRTOS byte
// queue. The single application owner drains TinyUSB's bounded FIFO directly.
USBCDC descriptor(nmeaInterface);
bool started = false;
class NmeaStream final : public controller::hal::ByteStream {
 public:
  int read(uint8_t* data, size_t capacity) override {
    if (!data && capacity) return -1;
    if (!started || !capacity || !tud_cdc_n_ready(nmeaInterface)) return 0;
    // Application reads are bounded; clamp the API's wider size_t input too.
    const uint32_t length = capacity > INT32_MAX ? INT32_MAX : static_cast<uint32_t>(capacity);
    return static_cast<int>(tud_cdc_n_read(nmeaInterface, data, length));
  }
  int write(const uint8_t*, size_t) override { return -1; }
};
}
void beginUsbNmea() {
  descriptor.enableReboot(false); // NMEA clients must never trigger the 1200-baud reset gesture.
  started = true;
}
controller::hal::ByteStream& usbNmeaStream() { static NmeaStream stream; return stream; }
void mirrorUsbNmea(uint8_t byte) {
  if (!started || !tud_cdc_n_ready(nmeaInterface)) return;
  // ready() does not require DTR, allowing clients such as gpsd to read NMEA.
  // Full/disconnected output drops bytes instead of delaying scans or logging.
  if (tud_cdc_n_write_available(nmeaInterface)) tud_cdc_n_write_char(nmeaInterface, static_cast<char>(byte));
  if (byte == '\n') tud_cdc_n_write_flush(nmeaInterface);
}
}
#endif
