#pragma once

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <SdFat.h>

// Returns true only when the controller has scheduled an immediate reboot to
// hand off a freshly staged firmware image to the OTA bootloader.
bool controllerUpdateRuntimeHandleBoot(SdFat& sd,
                                       bool sd_ready,
                                       Stream& serial,
                                       Adafruit_NeoPixel& pixels);
