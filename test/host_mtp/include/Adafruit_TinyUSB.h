#pragma once
#include "device/usbd.h"
#include "class/mtp/mtp_device.h"
class Adafruit_USBD_Interface {
 protected: uint8_t _strid = 0;
 public:
 virtual uint16_t getInterfaceDescriptor(uint8_t, uint8_t*, uint16_t) = 0;
 void setStringDescriptor(const char*) {}
};
struct FakeUsbDevice {
 bool addInterface(Adafruit_USBD_Interface&) { return true; }
 uint8_t allocInterface(unsigned) { return 4; }
 uint8_t allocEndpoint(unsigned) { return 5; }
};
inline FakeUsbDevice TinyUSBDevice;
inline void TinyUSB_Device_Task() {}
