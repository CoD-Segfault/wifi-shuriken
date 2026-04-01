#pragma once

#include <stdint.h>

class Adafruit_NeoPixel;
class TinyGPSPlus;

// Shared GNSS timing thresholds used both by the GNSS runtime and the
// controller logging config.
static constexpr uint32_t CONTROLLER_GNSS_TARGET_BAUD = 115200;
static constexpr uint16_t CONTROLLER_GPS_FIELD_MAX_AGE_MS = 3000;
static constexpr uint16_t CONTROLLER_GNSS_BOOT_TIMESTAMP_WAIT_MS = 6000;
static constexpr uint8_t CONTROLLER_GNSS_BOOT_TIMESTAMP_READS = 5;
static constexpr uint16_t CONTROLLER_GNSS_MIN_VALID_YEAR = 2026;

using ControllerSerialPrintfFn = void (*)(const char* fmt, ...);

// Bring up the optional USB CDC passthrough interface used for raw NMEA.
void controllerGnssRuntimeInitUsbPassthrough();

// Configure the GNSS UART pins, FIFO, and target baud rate.
void controllerGnssRuntimeInitUart();

// Send a PMTK command and emit the same console logging used before extraction.
void controllerGnssRuntimeSendPMTKCommand(const char* cmd);

// Drain GNSS UART bytes, feed TinyGPS++, mirror raw NMEA if enabled, and update
// the GPS status LED. Returns whether the current fix is considered usable.
bool controllerGnssRuntimeService(TinyGPSPlus& gps,
                                  Adafruit_NeoPixel& pixels,
                                  uint16_t gps_field_max_age_ms);

// Drain incoming NMEA bytes from the Android app on the USB CDC NMEA port,
// feed a separate TinyGPS++ instance, and return whether the phone-sourced fix
// is usable. Used as a fallback when the hardware GNSS has no valid fix.
bool controllerPhoneGnssRuntimeService(TinyGPSPlus& gps_phone,
                                       uint16_t gps_field_max_age_ms);

// Print the formatted GPS status line through the controller's normalized serial
// helper so line endings remain consistent with the rest of the console output.
void controllerGnssRuntimeSerialPrintStatus(TinyGPSPlus& gps,
                                            bool usable_fix,
                                            ControllerSerialPrintfFn serial_printf_normalized);
