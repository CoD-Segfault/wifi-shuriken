#include "controller_hal.h"
#include "hal/platform.h"
#include <cassert>

namespace controller { namespace pc {
namespace {
hal::ScannerTransport* scanner_instance;
hal::Clock* clock_instance;
hal::ByteStream* gnss_instance;
}
void bindPlatform(hal::ScannerTransport& scanner, hal::Clock& clock, hal::ByteStream& gnss) {
  scanner_instance = &scanner; clock_instance = &clock; gnss_instance = &gnss;
}
}}
namespace controller { namespace hal {
ScannerTransport& scannerTransport() { assert(pc::scanner_instance); return *pc::scanner_instance; }
Clock& clock() { assert(pc::clock_instance); return *pc::clock_instance; }
ByteStream& gnssStream() { assert(pc::gnss_instance); return *pc::gnss_instance; }
Capabilities capabilities() { return pc::capabilities(); }
bool scannerWorkerBegin() { return true; }
void scannerWorkerPoll() { clock().sleepMicroseconds(1000); }
}}
