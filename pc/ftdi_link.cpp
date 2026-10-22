#include "ftdi_link.h"
#include <ftdi.h>
#include <limits.h>

namespace controller { namespace pc {
FtdiLink::FtdiLink() : context_(ftdi_new()) {}
FtdiLink::~FtdiLink() { close(); if (context_) ftdi_free(context_); }
void FtdiLink::close() {
  if (opened_) {
    ftdi_set_bitmode(context_, 0, BITMODE_RESET);
    ftdi_usb_close(context_);
  }
  opened_ = false;
}
bool FtdiLink::open(const char* serial) {
  close();
  if (!context_ || !serial || !*serial || ftdi_set_interface(context_, INTERFACE_B) < 0)
    return false;
  // Require the serial number; never silently choose an arbitrary USB adapter.
  if (ftdi_usb_open_desc(context_, 0x0403, 0x6010, nullptr, serial) < 0) return false;
  opened_ = true;
  context_->usb_read_timeout = 10;
  context_->usb_write_timeout = 100;
  if (ftdi_set_latency_timer(context_, 1) < 0 ||
      ftdi_set_bitmode(context_, 0, BITMODE_RESET) < 0 ||
      ftdi_set_bitmode(context_, 0, BITMODE_MPSSE) < 0 ||
      ftdi_tcioflush(context_) < 0) { close(); return false; }
  return true;
}
const char* FtdiLink::error() const {
  return context_ ? ftdi_get_error_string(context_) : "could not allocate FTDI context";
}
int FtdiLink::write(const uint8_t* data, size_t length) {
  return opened_ && length <= INT_MAX ? ftdi_write_data(context_, data, int(length)) : -1;
}
int FtdiLink::read(uint8_t* data, size_t capacity) {
  return opened_ && capacity <= INT_MAX ? ftdi_read_data(context_, data, int(capacity)) : -1;
}
}}
