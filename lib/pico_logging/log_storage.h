#pragma once

#if defined(SHURIKEN_P4_CONTROLLER)
#include "p4_log_storage.h"
#else
#include <SdFat.h>
namespace pico_logging {
using LogStorage = SdFat;
using LogFile = FsFile;
inline LogFile openLogAppend(LogStorage& sd, const char* path) {
  return sd.open(path, O_RDWR | O_CREAT | O_APPEND);
}
}  // namespace pico_logging
#endif
