#include "controller_status.h"
#include <Adafruit_NeoPixel.h>

ControllerStatus controllerStatusCompute(bool sd_ready, bool usable_fix, bool csv_ready) {
  if (!sd_ready)   return ControllerStatus::SD_FAULT;
  if (!usable_fix) return ControllerStatus::NO_FIX;
  if (!csv_ready)  return ControllerStatus::FIX_NO_LOG;
  return ControllerStatus::OPERATIONAL;
}

void controllerStatusUpdateLed(Adafruit_NeoPixel& pixels, ControllerStatus status) {
  static ControllerStatus last_status = static_cast<ControllerStatus>(0xFF);
  if (status == last_status) {
    return;
  }
  last_status = status;

  uint32_t color;
  switch (status) {
    case ControllerStatus::SD_FAULT:    color = pixels.Color(128, 0,  0);  break;  // red
    case ControllerStatus::NO_FIX:      color = pixels.Color(75,  20, 0);  break;  // orange
    case ControllerStatus::FIX_NO_LOG:  color = pixels.Color(96,  80, 0);  break;  // amber
    case ControllerStatus::OPERATIONAL: color = pixels.Color(0,   96, 0);  break;  // green
    default:                            color = pixels.Color(128, 0,  0);  break;
  }
  pixels.setPixelColor(0, color);
  pixels.show();
}
