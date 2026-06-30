#pragma once

#include <SdFat.h>

// Register the MTP USB interface before USB enumeration starts.
bool controllerMtpStorageBegin(SdFat& sd, FsFile& active_log);

// Make the SD storage visible after boot-time update processing is complete.
void controllerMtpStorageSetReady(bool ready);

// Run deferred MTP filesystem work from core0 rather than the USB IRQ.
void controllerMtpStorageTask();

// Serialize normal firmware filesystem work against USB MTP callbacks.
void controllerMtpStorageLock();
bool controllerMtpStorageTryLock();
void controllerMtpStorageUnlock();
