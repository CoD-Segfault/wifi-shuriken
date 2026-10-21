#pragma once
#include <stdint.h>
typedef int esp_err_t;
constexpr uint32_t SDMMC_HOST_FLAG_4BIT = 1u << 1;
constexpr uint32_t SDMMC_HOST_FLAG_DEINIT_ARG = 1u << 5;
struct sdmmc_host_t {
  uint32_t flags;
  int slot;
  union {
    esp_err_t (*deinit)();
    esp_err_t (*deinit_p)(int);
  };
};
struct esp_vfs_fat_mount_config_t { bool format_if_mount_failed; };
struct sdmmc_card_t {};
