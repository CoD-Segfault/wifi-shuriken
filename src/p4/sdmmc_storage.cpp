#include "sdmmc_storage.h"

#include <Arduino.h>
#include <SD_MMC.h>
#include <driver/gpio.h>
#include <ff.h>
#include <stdio.h>

static_assert(FF_FS_EXFAT == 1, "The P4 target requires the project-local exFAT component");

namespace p4 {
namespace {
bool mounted = false;
const char* filesystem = "unavailable";
constexpr size_t TEST_BYTES = 4096;

const char* detectFilesystem() {
  // This target mounts only one FatFs volume: SD is logical drive 0.
  FF_DIR root;
  if (f_opendir(&root, "0:/") != FR_OK) return "unknown";
  const BYTE type = root.obj.fs->fs_type;
  f_closedir(&root);
  switch (type) {
    case FS_FAT12: return "FAT12";
    case FS_FAT16: return "FAT16";
    case FS_FAT32: return "FAT32";
    case FS_EXFAT: return "exFAT";
    default: return "unknown";
  }
}

uint8_t testByte(size_t offset) {
  return static_cast<uint8_t>((offset * 37U) ^ (offset >> 8U));
}
}  // namespace

bool storageBegin() {
  if (mounted) return true;
  // The esp32p4 variant manages slot 0, LDO4 and active-low GPIO45 power.
  if (!SD_MMC.setPins(43, 44, 39, 40, 41, 42) ||
      !SD_MMC.begin("/sdcard", false, false, SDMMC_FREQ_DEFAULT)) {
    return false;
  }
  // Must be reapplied after every mount, before application file I/O.
  for (int pin = 39; pin <= 44; ++pin) {
    if (gpio_set_drive_capability(static_cast<gpio_num_t>(pin), GPIO_DRIVE_CAP_2) != ESP_OK) {
      SD_MMC.end();
      return false;
    }
  }
  filesystem = detectFilesystem();
  mounted = true;
  return true;
}

bool storageReady() { return mounted; }
const char* storageFilesystem() { return filesystem; }

bool storageVerify() {
  if (!mounted) return false;
  // Only this loop owns SD access. Do not overwrite a diagnostic or user file.
  char path[64];
  bool selected = false;
  for (unsigned suffix = 0; suffix < 100; ++suffix) {
    snprintf(path, sizeof(path), "/shuriken-p4-test-%08lx-%02u.bin",
             static_cast<unsigned long>(micros()), suffix);
    if (!SD_MMC.exists(path)) { selected = true; break; }
  }
  if (!selected) return false;

  static uint8_t buffer[TEST_BYTES];
  for (size_t i = 0; i < sizeof(buffer); ++i) buffer[i] = testByte(i);
  File file = SD_MMC.open(path, FILE_WRITE);
  if (!file) return false;
  bool ok = file.write(buffer, sizeof(buffer)) == sizeof(buffer);
  file.close();
  if (ok) {
    file = SD_MMC.open(path, FILE_READ);
    ok = file && file.size() == sizeof(buffer) &&
         file.read(buffer, sizeof(buffer)) == sizeof(buffer);
    file.close();
  }
  if (ok) {
    for (size_t i = 0; i < sizeof(buffer); ++i) {
      if (buffer[i] != testByte(i)) { ok = false; break; }
    }
  }
  const bool removed = SD_MMC.remove(path);
  return ok && removed;
}
}  // namespace p4
