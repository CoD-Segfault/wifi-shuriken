#include "hal/logging_platform.h"
#include <cstdio>
#include <cstring>
#if defined(SHURIKEN_P4_CONTROLLER)
#include <Arduino.h>
#else
#include <Arduino.h>
#include <pico/unique_id.h>
#endif
namespace controller { namespace hal {
static constexpr size_t DEVICE_UID_HEX_BYTES = 8;
void boardUidHex(char* out, size_t out_len) {
  if (!out || out_len == 0) {
    return;
  }
  out[0] = '\0';

#if !defined(SHURIKEN_P4_CONTROLLER)
  pico_unique_board_id_t uid = {};
  pico_get_unique_board_id(&uid);

  uint8_t uid_bytes[DEVICE_UID_HEX_BYTES] = {};
  const size_t uid_size = sizeof(uid);
  const size_t copy_bytes = (uid_size < DEVICE_UID_HEX_BYTES) ? uid_size : DEVICE_UID_HEX_BYTES;
  memcpy(uid_bytes, &uid, copy_bytes);

  if (out_len <= (DEVICE_UID_HEX_BYTES * 2)) {
    return;
  }
  size_t pos = 0;
  for (size_t i = 0; i < copy_bytes; i++) {
    const int written = snprintf(out + pos, out_len - pos, "%02X", uid_bytes[i]);
    if (written < 0) {
      out[0] = '\0';
      return;
    }
    pos += static_cast<size_t>(written);
  }
  out[pos] = '\0';
#else
  snprintf(out, out_len, "%012llX", static_cast<unsigned long long>(ESP.getEfuseMac()));
#endif
}

}}
