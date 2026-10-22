#pragma once
namespace p4 {
// The application loop owns every filesystem operation; USB callbacks only move packets.
void pollUsbMtp(void (*flushLog)());
}
