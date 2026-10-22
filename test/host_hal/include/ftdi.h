#pragma once
#include <cstdint>
struct ftdi_context { int usb_read_timeout; int usb_write_timeout; };
enum ftdi_interface { INTERFACE_B = 2 };
enum { BITMODE_RESET = 0, BITMODE_MPSSE = 2 };
ftdi_context* ftdi_new();
void ftdi_free(ftdi_context*);
int ftdi_set_interface(ftdi_context*, ftdi_interface);
int ftdi_usb_open_desc(ftdi_context*, int, int, const char*, const char*);
int ftdi_set_latency_timer(ftdi_context*, unsigned char);
int ftdi_set_bitmode(ftdi_context*, unsigned char, unsigned char);
int ftdi_tcioflush(ftdi_context*);
int ftdi_usb_close(ftdi_context*);
char* ftdi_get_error_string(ftdi_context*);
int ftdi_write_data(ftdi_context*, const unsigned char*, int);
int ftdi_read_data(ftdi_context*, unsigned char*, int);
