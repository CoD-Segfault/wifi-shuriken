#include <esp_vfs_fat.h>
#include <driver/sdmmc_host.h>
#include <cassert>
#include <cstring>
#include <cstdio>

extern "C" esp_err_t __wrap_esp_vfs_fat_sdmmc_mount(
    const char*, const sdmmc_host_t*, const void*, const esp_vfs_fat_mount_config_t*, sdmmc_card_t**);

static uint32_t seenFlags;
static int deinitializedSlot = -1;
static int globalDeinitCalls = 0;
static int mountResult = 0;
static const char path[] = "/sdcard";
static int slotConfig;
static esp_vfs_fat_mount_config_t mountConfig = {false};
static sdmmc_card_t* card = nullptr;

extern "C" esp_err_t sdmmc_host_deinit_slot(int slot) {
  deinitializedSlot = slot;
  return 0;
}
static esp_err_t globalDeinit() { ++globalDeinitCalls; return 0; }

extern "C" esp_err_t __real_esp_vfs_fat_sdmmc_mount(
    const char* receivedPath, const sdmmc_host_t* host, const void* config,
    const esp_vfs_fat_mount_config_t* mount, sdmmc_card_t** out) {
  assert(receivedPath == path && config == &slotConfig && mount == &mountConfig && out == &card);
  assert(!mount->format_if_mount_failed);
  seenFlags = host->flags;
  // Exercise the same flag-selected callback contract as vendored FatFs.
  if (host->flags & SDMMC_HOST_FLAG_DEINIT_ARG) host->deinit_p(host->slot);
  else host->deinit();
  return mountResult;
}

int main() {
  sdmmc_host_t host = {};
  host.flags = SDMMC_HOST_FLAG_4BIT;
  host.slot = 0;
  host.deinit_p = sdmmc_host_deinit_slot;
  assert(__wrap_esp_vfs_fat_sdmmc_mount(path, &host, &slotConfig, &mountConfig, &card) == 0);
  assert(seenFlags == (SDMMC_HOST_FLAG_4BIT | SDMMC_HOST_FLAG_DEINIT_ARG));
  assert(host.flags == SDMMC_HOST_FLAG_4BIT);  // Caller storage is unchanged.
  assert(deinitializedSlot == 0 && globalDeinitCalls == 0);

  host.flags |= SDMMC_HOST_FLAG_DEINIT_ARG;
  host.slot = 1;
  mountResult = -42;
  assert(__wrap_esp_vfs_fat_sdmmc_mount(path, &host, &slotConfig, &mountConfig, &card) == -42);
  assert(seenFlags == host.flags && deinitializedSlot == 1);

  host.flags = SDMMC_HOST_FLAG_4BIT;
  host.deinit = globalDeinit;
  assert(__wrap_esp_vfs_fat_sdmmc_mount(path, &host, &slotConfig, &mountConfig, &card) == -42);
  assert(seenFlags == SDMMC_HOST_FLAG_4BIT && globalDeinitCalls == 1);
  puts("PASS: slot deinit flag repair, caller isolation, error/argument forwarding, other callbacks unchanged");
}
