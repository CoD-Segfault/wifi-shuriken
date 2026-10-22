#pragma once

#include <TinyGPSPlus.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>

namespace controller {
struct GpsDebugSource {
  const char* name;
  const TinyGPSPlus* parser;
  bool available;
  bool usable;
};

// Fixed-memory snapshots. Read a COPY of each parser: TinyGPS field getters
// clear isUpdated flags, which debugging must not change on the live sources.
class GpsDebugReport {
 public:
  void capture(const GpsDebugSource (&sources)[3], const char* selected, uint32_t maxAge, uint32_t now) {
    count_ = 0;
    add("GPS DEBUG t=%lu ms selected=%s; latest observations, NOT time-aligned\r\n",
        static_cast<unsigned long>(now), selected);
    bool fresh[3] = {};
    double lat[3] = {}, lon[3] = {};
    for (unsigned i = 0; i < 3; ++i) {
      const auto& source = sources[i];
      if (!source.available) {
        add("%s unavailable (no input transport on this target)\r\n", source.name);
        continue;
      }
      TinyGPSPlus gps = *source.parser;
      fresh[i] = gps.location.isValid() && gps.location.age() < maxAge;
      char position[72] = "unset", altitude[32] = "unset", hdop[24] = "unset", sats[16] = "unset";
      char locAge[16], altAge[16], hdopAge[16], satsAge[16], dateAge[16], timeAge[16];
      if (gps.location.isValid()) {
        lat[i] = gps.location.lat(); lon[i] = gps.location.lng();
        snprintf(position, sizeof(position), "%.7f,%.7f", lat[i], lon[i]);
      }
      if (gps.altitude.isValid()) snprintf(altitude, sizeof(altitude), "%.2f", gps.altitude.meters());
      if (gps.hdop.isValid()) snprintf(hdop, sizeof(hdop), "%.2f", gps.hdop.hdop());
      if (gps.satellites.isValid()) snprintf(sats, sizeof(sats), "%lu", static_cast<unsigned long>(gps.satellites.value()));
      add("%s usable=%s pos=%s loc_age=%s alt_m=%s(age=%s) hdop=%s(age=%s) sats=%s(age=%s)\r\n",
          source.name, source.usable ? "yes" : "no", position, age(gps.location.age(), locAge),
          altitude, age(gps.altitude.age(), altAge), hdop, age(gps.hdop.age(), hdopAge),
          sats, age(gps.satellites.age(), satsAge));
      char date[20] = "unset", time[20] = "unset";
      if (gps.date.isValid()) snprintf(date, sizeof(date), "%04u-%02u-%02u", gps.date.year(), gps.date.month(), gps.date.day());
      if (gps.time.isValid()) snprintf(time, sizeof(time), "%02u:%02u:%02u.%02u", gps.time.hour(), gps.time.minute(), gps.time.second(), gps.time.centisecond());
      add("%s UTC=%sT%s date_age=%s time_age=%s chars=%lu fix_sentences=%lu checksum_ok=%lu checksum_fail=%lu\r\n",
          source.name, date, time, age(gps.date.age(), dateAge), age(gps.time.age(), timeAge),
          static_cast<unsigned long>(gps.charsProcessed()), static_cast<unsigned long>(gps.sentencesWithFix()),
          static_cast<unsigned long>(gps.passedChecksum()), static_cast<unsigned long>(gps.failedChecksum()));
    }
    for (unsigned a = 0; a < 3; ++a) for (unsigned b = a + 1; b < 3; ++b) {
      if (!sources[a].available || !sources[b].available) continue;
      if (fresh[a] && fresh[b]) {
        add("%s/%s separation=%.2f m (latest fresh positions; not an accuracy measurement)\r\n",
            sources[a].name, sources[b].name, TinyGPSPlus::distanceBetween(lat[a], lon[a], lat[b], lon[b]));
      } else {
        add("%s/%s separation=unavailable (both positions must be fresh)\r\n", sources[a].name, sources[b].name);
      }
    }
  }
  size_t size() const { return count_; }
  const char* line(size_t index) const { return index < count_ ? lines_[index] : ""; }
 private:
  static const char* age(uint32_t value, char (&out)[16]) {
    if (value == UINT32_MAX) snprintf(out, sizeof(out), "unset");
    else snprintf(out, sizeof(out), "%lums", static_cast<unsigned long>(value));
    return out;
  }
  void add(const char* format, ...) {
    if (count_ >= 10) return;
    va_list args;
    va_start(args, format);
    vsnprintf(lines_[count_++], sizeof(lines_[0]), format, args);
    va_end(args);
  }
  char lines_[10][256] = {};
  size_t count_ = 0;
};
}  // namespace controller
