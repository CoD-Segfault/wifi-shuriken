#include "ftdi.h"
#include "ftdi_link.h"
#include <cassert>
#include <cstring>
#include <cstdio>

static ftdi_context context;
static bool allocated = true;
static int calls, fail_at, closes, frees;
static int step() { return ++calls == fail_at ? -1 : 0; }
ftdi_context* ftdi_new() { return allocated ? &context : nullptr; }
void ftdi_free(ftdi_context* ptr) { assert(ptr == &context); ++frees; }
int ftdi_set_interface(ftdi_context*, ftdi_interface interface) { assert(interface == INTERFACE_B); return step(); }
int ftdi_usb_open_desc(ftdi_context*, int vendor, int product, const char* description, const char* serial) {
  assert(vendor == 0x0403 && product == 0x6010 && !description && std::strcmp(serial, "TGTEST") == 0);
  return step();
}
int ftdi_set_latency_timer(ftdi_context*, unsigned char timer) { assert(timer == 1); return step(); }
int ftdi_set_bitmode(ftdi_context*, unsigned char mask, unsigned char mode) {
  assert(mask == 0 && (mode == BITMODE_RESET || mode == BITMODE_MPSSE)); return step();
}
int ftdi_tcioflush(ftdi_context*) { return step(); }
int ftdi_usb_close(ftdi_context*) { ++closes; return 0; }
char* ftdi_get_error_string(ftdi_context*) { static char text[] = "fake USB failure"; return text; }
int ftdi_write_data(ftdi_context*, const unsigned char*, int length) { return length - 1; }
int ftdi_read_data(ftdi_context*, unsigned char*, int) { return 0; }
int main() {
  uint8_t data[64] = {};
  for (int failure = 0; failure <= 6; ++failure) {
    calls = closes = frees = 0; fail_at = failure;
    {
      controller::pc::FtdiLink link;
      assert(!link.open(nullptr) && !link.open(""));
      assert(link.read(data, sizeof(data)) == -1 && link.write(data, sizeof(data)) == -1);
      assert(link.open("TGTEST") == (failure == 0));
      if (!failure) {
        assert(context.usb_read_timeout == 10 && context.usb_write_timeout == 100);
        assert(link.write(data, sizeof(data)) == 63); // caller sees short writes
        assert(link.read(data, sizeof(data)) == 0);
      }
      assert(std::strcmp(link.error(), "fake USB failure") == 0);
      link.close(); link.close();
      assert(closes == (failure == 1 || failure == 2 ? 0 : 1));
    }
    assert(frees == 1);
  }
  allocated = false;
  controller::pc::FtdiLink link;
  assert(!link.open("TGTEST"));
  assert(link.read(data, sizeof(data)) == -1);
  assert(std::strstr(link.error(), "allocate"));
  std::puts("FTDI lifecycle: interface B/serial selection/short transfers/all initialization failures passed");
}
