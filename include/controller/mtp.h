#pragma once
#include "hal/mtp_storage.h"
#include <array>
namespace controller::mtp {
// A transaction advances only after the previous USB IN transfer completes.
// All methods, including filesystem work, belong to the application thread.
class Responder {
 public:
  static constexpr uint32_t storageId = 0x00010001;
  explicit Responder(Storage& storage) : storage_(storage) {}
  bool command(const uint8_t* data, size_t length);
  // Returns the next container fragment. Zero means the transaction is done.
  size_t next(uint8_t* data, size_t capacity);
  void reset();
  void cancel();
  bool active() const { return phase_ != Phase::Idle; }
 private:
  enum class Phase { Idle, Data, Response };
  Storage& storage_;
  Phase phase_ = Phase::Idle;
  bool session_ = false, file_ = false, partial_ = false;
  uint16_t operation_ = 0, response_ = 0x2001;
  uint32_t transaction_ = 0, offset_ = 0, payloadSize_ = 0, sent_ = 0;
  std::array<uint32_t, 5> params_{};
  std::vector<Object> objects_;
  std::vector<uint8_t> payload_;
  void prepare();
  void fail(uint16_t response);
  void u16(uint16_t value);
  void u32(uint32_t value);
  void u64(uint64_t value);
  void string(const std::string& value);
  void codes(const uint16_t* values, size_t count);
  bool catalog();
  Object* object(uint32_t handle);
};
}
