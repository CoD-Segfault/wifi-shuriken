#include "hal/platform.h"
#include "posix/utc_clock.h"
namespace controller::hal {
UtcClock& utcClock() { static SystemUtcClock instance; return instance; }
}
