#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// All calls run under TinyUSB's USB mutex. Generation invalidates deferred
// application callbacks before endpoint buffers/command pointers can be reused.
uint32_t controllerMtpTransportGeneration(void);
uint16_t controllerMtpTransportStatus(void);
void controllerMtpTransportAbort(bool reset_session);
void controllerMtpTransportPoll(void);
void controllerMtpTransportSetSession(uint32_t session);

// Transport callback: flags only, never filesystem work in USB/IRQ context.
void controllerMtpTransportResetCallback(bool reset_session);

#ifdef __cplusplus
}
#endif
