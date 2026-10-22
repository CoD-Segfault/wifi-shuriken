#pragma once
#include "hal/mtp_storage.h"
#include <cassert>
#include <cstring>
namespace fake_mtp {
inline bool inUsb=false;
inline unsigned reads=0, closes=0, lists=0;
}
namespace controller::mtp {
class PosixStorage:public Storage {
 public:
  explicit PosixStorage(const char*) {}
  bool ready() const override {return true;}
  uint64_t capacity() const override {return 12345678;}
  void flush() override {assert(!fake_mtp::inUsb);}
  bool list(std::vector<Object>& items) override {assert(!fake_mtp::inUsb);++fake_mtp::lists;items.push_back({"scan.csv",500,false});return true;}
  bool stat(Object& item) override {assert(!fake_mtp::inUsb);item.size=500;return true;}
  bool open(const Object&) override {assert(!fake_mtp::inUsb);return true;}
  bool read(uint32_t offset,uint8_t* data,size_t length) override {assert(!fake_mtp::inUsb);assert(offset+length<=500);++fake_mtp::reads;memset(data,0x5a,length);return true;}
  void close() override {assert(!fake_mtp::inUsb);++fake_mtp::closes;}
};
}
