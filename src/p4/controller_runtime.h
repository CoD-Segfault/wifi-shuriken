#pragma once
class Stream;

namespace p4 {
struct BleMessage;
// Main-loop owner: SD logging/GNSS here; scanner task owns its SPI bus and dedupe.
bool controllerBegin(Stream& console);
void controllerPoll();
void controllerPrintStatus(Stream& console);
// Opt-in diagnostics only; never change source selection, clock or log files.
void controllerGpsDebugStart(bool watch);
void controllerGpsDebugStop();
// Feed queued BLE bytes on the application loop. Returns true for NMEA chunks.
bool controllerReceiveBle(const BleMessage& message);
}  // namespace p4
