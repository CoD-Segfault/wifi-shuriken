#pragma once

#include <stdint.h>

class Adafruit_NeoPixel;
class TinyGPSPlus;

static constexpr uint32_t CONTROLLER_GNSS_TARGET_BAUD = 115200;
static constexpr uint16_t CONTROLLER_GPS_FIELD_MAX_AGE_MS = 3000;
static constexpr uint16_t CONTROLLER_GNSS_BOOT_TIMESTAMP_WAIT_MS = 6000;
static constexpr uint8_t CONTROLLER_GNSS_BOOT_TIMESTAMP_READS = 5;
static constexpr uint16_t CONTROLLER_GNSS_MIN_VALID_YEAR = 2026;

using ControllerSerialPrintfFn = void (*)(const char* fmt, ...);

void controllerGnssRuntimeInitUsbPassthrough();
void controllerGnssRuntimeInitUart();
void controllerGnssRuntimeSendPMTKCommand(const char* cmd);
bool controllerGnssRuntimeService(TinyGPSPlus& gps,
                                  Adafruit_NeoPixel& pixels,
                                  uint16_t gps_field_max_age_ms);
void controllerGnssRuntimeSerialPrintStatus(TinyGPSPlus& gps,
                                            bool usable_fix,
                                            ControllerSerialPrintfFn serial_printf_normalized);
