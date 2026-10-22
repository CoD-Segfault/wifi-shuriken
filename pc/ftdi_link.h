#pragma once
#include "controller_hal.h"
struct ftdi_context;

namespace controller { namespace pc {
// Claims only interface B. The kernel's interface-A UART remains independent.
class FtdiLink final : public MpsseLink {
 public:
  FtdiLink();
  ~FtdiLink() override;
  FtdiLink(const FtdiLink&) = delete;
  FtdiLink& operator=(const FtdiLink&) = delete;
  bool open(const char* serial);
  void close();
  const char* error() const;
  int write(const uint8_t* data, size_t length) override;
  int read(uint8_t* data, size_t capacity) override;
 private:
  ftdi_context* context_;
  bool opened_ = false;
};
}}
