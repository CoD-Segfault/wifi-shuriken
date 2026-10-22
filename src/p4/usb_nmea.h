#pragma once
#include "hal/controller.h"
namespace p4 {
// CDC0 is the console; CDC1 is raw NMEA. Call begin on the application owner.
void beginUsbNmea();
controller::hal::ByteStream& usbNmeaStream();
void mirrorUsbNmea(uint8_t byte);
}
