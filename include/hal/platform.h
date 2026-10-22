#pragma once
#include "hal/controller.h"
#include "hal/time.h"

namespace controller { namespace hal {
// These instances retain platform-specific ownership/lifecycle. Scanner access
// belongs to the scanner worker (or the exclusive boot-time update window).
ScannerTransport& scannerTransport();
Clock& clock();
UtcClock& utcClock();
ByteStream& gnssStream();
Capabilities capabilities();
// Invoked on the scanner worker: watchdog registration/feed and scheduler yield.
bool scannerWorkerBegin();
void scannerWorkerPoll();
}}
