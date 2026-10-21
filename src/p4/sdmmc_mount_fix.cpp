#include <esp_vfs_fat.h>
#include <driver/sdmmc_host.h>

// Arduino 3.3.12's SD_MMC.begin() replaces host.flags with the bus-width
// flags, losing DEINIT_ARG even though SDMMC_HOST_DEFAULT uses deinit_p.
// Correct a local copy before FatFs uses/saves it, including mount failures.
// Linker wrapping leaves both the SDK source and other targets untouched.
extern "C" esp_err_t __real_esp_vfs_fat_sdmmc_mount(
    const char*, const sdmmc_host_t*, const void*, const esp_vfs_fat_mount_config_t*, sdmmc_card_t**);

extern "C" esp_err_t __wrap_esp_vfs_fat_sdmmc_mount(
    const char* path, const sdmmc_host_t* host_config, const void* slot_config,
    const esp_vfs_fat_mount_config_t* mount_config, sdmmc_card_t** card) {
  sdmmc_host_t host = *host_config;
  if (host.deinit_p == sdmmc_host_deinit_slot) {
    host.flags |= SDMMC_HOST_FLAG_DEINIT_ARG;
  }
  return __real_esp_vfs_fat_sdmmc_mount(path, &host, slot_config, mount_config, card);
}
