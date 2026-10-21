#pragma once

namespace p4 {
// Single-owner API: call from the controller loop, never a BLE callback.
bool storageBegin();
bool storageReady();
const char* storageFilesystem();
// Verifies a new, uniquely named scratch file; never replaces an existing file.
// Keeps the filesystem mounted. Intended for manual bring-up, not every boot.
bool storageVerify();
}  // namespace p4
