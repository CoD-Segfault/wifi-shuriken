#pragma once

#include <TinyGPSPlus.h>
#include <stddef.h>
#include <stdint.h>

namespace controller {
// Main-loop owned. Buffer whole sentences so dropped chunks, ordinary UART
// text and different BLE sessions cannot accidentally complete a stale fix.
class BleGnss {
 public:
  void updateSession(bool connected, uint32_t session, uint32_t drops) {
    if (!connected || session != session_ || drops != drops_) {
      gps_ = TinyGPSPlus();
      length_ = 0;
    }
    connected_ = connected;
    session_ = session;
    drops_ = drops;
  }

  // Returns true for chunks containing NMEA traffic; those should not echo.
  bool feed(const uint8_t* bytes, size_t length, uint32_t session) {
    if (!connected_ || session != session_) return false;
    bool nmea = length_ != 0;
    for (size_t i = 0; i < length; ++i) {
      const char c = static_cast<char>(bytes[i]);
      if (c == '$') { length_ = 0; line_[length_++] = c; nmea = true; }
      else if (length_) {
        if (c == '\n') {
          for (size_t j = 0; j < length_; ++j) gps_.encode(line_[j]);
          gps_.encode('\n');  // TinyGPS validates checksum before committing fields.
          length_ = 0;
        } else if ((c == '\r' || (c >= 32 && c <= 126)) && length_ < sizeof(line_)) {
          line_[length_++] = c;
        } else {
          length_ = 0;  // Oversize/binary input: wait for another '$'.
        }
      }
    }
    return nmea;
  }
  bool usable(uint32_t maxAge) { return connected_ && gps_.location.isValid() && gps_.location.age() < maxAge; }
  TinyGPSPlus& parser() { return gps_; }
 private:
  TinyGPSPlus gps_;
  char line_[128];
  size_t length_ = 0;
  uint32_t session_ = 0, drops_ = 0;
  bool connected_ = false;
};

inline TinyGPSPlus* selectGpsSource(TinyGPSPlus& hardware, bool hardwareFix,
                                    TinyGPSPlus& usb, bool usbFix,
                                    TinyGPSPlus& ble, bool bleFix) {
  if (hardwareFix) return &hardware;
  if (usbFix) return &usb;
  if (bleFix) return &ble;
  return &hardware;  // Preserve hardware clock/per-field fallback behavior.
}
}  // namespace controller
