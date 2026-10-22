#include "hal/logging_platform.h"
namespace controller::hal {
void boardUidHex(char* output, size_t capacity) { if (capacity) output[0] = '\0'; }
}
