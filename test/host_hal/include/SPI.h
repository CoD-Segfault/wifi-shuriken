#pragma once
#include "Arduino.h"
#define MSBFIRST 1
#define SPI_MODE0 0
#define FSPI 0
struct SPISettings {
  uint32_t hz;
  SPISettings(uint32_t freq, int order, int mode) : hz(freq) { assert(order == 1 && mode == 0); }
};
class SPIClass {
 public:
  explicit SPIClass(int = 0) {}
  bool begin(int = 0, int = 0, int = 0, int = 0) { fake::events.push_back(2000); return fake::begin_ok; }
  void end() { fake::events.push_back(2001); }
  void setSCK(int) {}
  void setTX(int) {}
  void setRX(int) {}
  void beginTransaction(SPISettings settings) {
    hz_ = settings.hz;
    if (hz_ != 1000000) fake::frames.push_back({fake::now, hz_, fake::cs, {}});
  }
  void endTransaction() {}
  uint8_t transfer(uint8_t byte) {
    if (hz_ == 1000000) { assert(!fake::enabled); fake::pending_cs = byte; }
    else {
#if SCANNER_USE_SHIFTREG_CS
      assert(fake::enabled && fake::cs != 0xff);
#else
      assert(fake::direct);
#endif
      fake::frames.back().data.push_back(byte);
    }
    return byte ^ 0xa5;
  }
 private:
  uint32_t hz_ = 0;
};
inline SPIClass SPI1;
