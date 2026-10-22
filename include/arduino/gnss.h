#pragma once

#include <stdint.h>

// Shared GNSS timing thresholds used both by the GNSS runtime and the
// controller logging config.
// A macro rather than a constexpr so the preprocessor can test it: the MTK
// baud command below carries a precomputed checksum that is only valid at one
// baud, and that has to be guarded at compile time.
#ifndef CONTROLLER_GNSS_TARGET_BAUD
#define CONTROLLER_GNSS_TARGET_BAUD 115200
#endif
static constexpr uint16_t CONTROLLER_GPS_FIELD_MAX_AGE_MS = 3000;
static constexpr uint16_t CONTROLLER_GNSS_BOOT_TIMESTAMP_WAIT_MS = 6000;
static constexpr uint8_t CONTROLLER_GNSS_BOOT_TIMESTAMP_READS = 5;
static constexpr uint16_t CONTROLLER_GNSS_MIN_VALID_YEAR = 2026;

// Bring up the optional USB CDC passthrough interface used for raw NMEA.
void controllerGnssRuntimeInitUsbPassthrough();

// Configure the GNSS UART pins, FIFO, target baud rate, and module output.
void controllerGnssRuntimeBegin();

// Configure the GNSS UART pins, FIFO, and target baud rate.
void controllerGnssRuntimeInitUart();

// Send a PMTK command and emit the same console logging used before extraction.
void controllerGnssRuntimeSendPMTKCommand(const char* cmd);

namespace controller { namespace hal { class ByteStream; }}
controller::hal::ByteStream* controllerUsbNmeaStream();
void controllerMirrorNmea(uint8_t byte);
