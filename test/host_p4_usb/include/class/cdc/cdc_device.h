#pragma once
#include <USBCDC.h>
#define CFG_TUD_CDC 2
inline bool tud_cdc_n_ready(uint8_t interface) { assert(interface < 2); return fake_usb::mounted; }
inline uint32_t tud_cdc_n_read(uint8_t interface, void* output, uint32_t capacity) {
  assert(interface < 2); ++fake_usb::reads[interface];
  uint32_t count = 0; auto* data = static_cast<uint8_t*>(output);
  while (count < capacity && !fake_usb::rx[interface].empty()) {
    data[count++] = fake_usb::rx[interface].front(); fake_usb::rx[interface].pop_front();
  }
  return count;
}
inline uint32_t tud_cdc_n_write_available(uint8_t interface) { return fake_usb::room[interface]; }
inline uint32_t tud_cdc_n_write_char(uint8_t interface, char byte) {
  assert(interface < 2); ++fake_usb::writes[interface];
  if (!fake_usb::room[interface]) return 0;
  --fake_usb::room[interface]; fake_usb::tx[interface] += byte; return 1;
}
inline uint32_t tud_cdc_n_write_flush(uint8_t interface) { ++fake_usb::flushes[interface]; return 0; }
