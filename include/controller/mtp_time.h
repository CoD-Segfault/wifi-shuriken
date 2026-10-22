#pragma once
#include "hal/time.h"
#include <cstring>

namespace controller::mtp {
// MTP DateTime uses an ISO 8601 basic timestamp. Explicit UTC avoids relying on
// the controller's libc timezone. Unknown dates stay empty rather than invented.
inline bool formatTimestamp(hal::EpochSeconds epoch, char (&output)[17]) {
  output[0] = '\0';
  char compact[15];
  if (epoch <= 0 || !hal::formatUtc(epoch, compact, sizeof(compact), true)) return false;
  std::memcpy(output, compact, 8);
  output[8] = 'T';
  std::memcpy(output + 9, compact + 8, 6);
  output[15] = 'Z';
  output[16] = '\0';
  return true;
}
inline hal::EpochSeconds fatTimestamp(uint16_t date, uint16_t time) {
  hal::UtcDateTime utc{static_cast<uint16_t>(1980 + (date >> 9)),
      static_cast<uint8_t>((date >> 5) & 15), static_cast<uint8_t>(date & 31),
      static_cast<uint8_t>(time >> 11), static_cast<uint8_t>((time >> 5) & 63),
      static_cast<uint8_t>((time & 31) * 2)};
  hal::EpochSeconds epoch = 0;
  return hal::epochFromUtc(utc, epoch) ? epoch : 0;
}
}
