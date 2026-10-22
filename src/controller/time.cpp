#include "hal/time.h"
#include <cstdio>
namespace controller::hal {
bool epochFromUtc(const UtcDateTime& utc, EpochSeconds& epoch) {
  const int year = utc.year, month = utc.month;
  if (year < 1970 || year > 9999 || month < 1 || month > 12 || utc.hour > 23 || utc.minute > 59 || utc.second > 59) return false;
  constexpr uint8_t days[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  const bool leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
  if (utc.day < 1 || utc.day > days[month - 1] + (month == 2 && leap)) return false;
  const int y = year - (month <= 2);
  const int era = y / 400, yoe = y - era * 400;
  const int doy = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + utc.day - 1;
  const int64_t total_days = int64_t(era) * 146097 + yoe * 365 + yoe / 4 - yoe / 100 + doy - 719468;
  epoch = total_days * 86400 + utc.hour * 3600 + utc.minute * 60 + utc.second;
  return true;
}
bool utcFromEpoch(EpochSeconds epoch, UtcDateTime& utc) {
  if (epoch < 0 || epoch > 253402300799LL) return false;
  const int64_t z = epoch / 86400 + 719468;
  const int era = static_cast<int>(z / 146097), doe = static_cast<int>(z - int64_t(era) * 146097);
  const int yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  int year = yoe + era * 400;
  const int doy = doe - (365 * yoe + yoe / 4 - yoe / 100), mp = (5 * doy + 2) / 153;
  const int day = doy - (153 * mp + 2) / 5 + 1, month = mp + (mp < 10 ? 3 : -9);
  year += month <= 2;
  const int seconds = static_cast<int>(epoch % 86400);
  utc = {static_cast<uint16_t>(year), static_cast<uint8_t>(month), static_cast<uint8_t>(day),
         static_cast<uint8_t>(seconds / 3600), static_cast<uint8_t>((seconds / 60) % 60), static_cast<uint8_t>(seconds % 60)};
  return true;
}
bool formatUtc(EpochSeconds epoch, char* output, size_t capacity, bool compact) {
  if (!output || !capacity) return false;
  output[0] = '\0'; UtcDateTime utc;
  if (!utcFromEpoch(epoch, utc)) return false;
  const int length = std::snprintf(output, capacity, compact ? "%04u%02u%02u%02u%02u%02u" : "%04u-%02u-%02u %02u:%02u:%02u",
      unsigned(utc.year), unsigned(utc.month), unsigned(utc.day), unsigned(utc.hour), unsigned(utc.minute), unsigned(utc.second));
  if (length < 0 || static_cast<size_t>(length) >= capacity) { output[0] = '\0'; return false; }
  return true;
}
}
