#pragma once
#include "posix/log_storage.h"
#include "sdmmc_storage.h"
#include <Arduino.h>
namespace p4 {
class LogStorage final : public controller::storage::PosixLogStorage {
 public:
  explicit LogStorage(Stream& console) : PosixLogStorage("/sdcard"), console_(console) {}
  bool mount() override {
    const bool ready = storageBegin() && PosixLogStorage::mount();
    console_.println(ready ? "SDMMC storage ready" : "SDMMC mount unavailable (no formatting)");
    return ready;
  }
 private:
  Stream& console_;
};
}
