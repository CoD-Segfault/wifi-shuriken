#pragma once
#include <stdint.h>

class Adafruit_NeoPixel;

// Semantic operating state for the controller. Renderers (LED, serial, etc.)
// interpret this value independently, keeping output concerns out of the logic
// that computes the state.
enum class ControllerStatus : uint8_t {
  SD_FAULT    = 0,  // SD card not ready
  NO_FIX      = 1,  // SD ready, no usable GPS fix
  FIX_NO_LOG  = 2,  // GPS fix acquired, CSV logging not yet active
  OPERATIONAL = 3,  // GPS fix + CSV logging active
};

// Derive the current controller status from the three state flags.
ControllerStatus controllerStatusCompute(bool sd_ready, bool usable_fix, bool csv_ready);

// Update the NeoPixel LED to reflect status. Caches last value to skip
// redundant SPI writes.
void controllerStatusUpdateLed(Adafruit_NeoPixel& pixels, ControllerStatus status);
