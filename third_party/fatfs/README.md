# FatFs With exFAT

Vendored from ESP-IDF v5.5.5, commit
`b774170ff46c393eeb5e495ea37936038d3f4f4f`:
https://github.com/espressif/esp-idf/tree/v5.5.5/components/fatfs

Only the component's embedded build sources and headers are included. The sole
upstream code change is `FF_FS_EXFAT = 1` in `src/ffconf.h`. Original copyright
and license notices are preserved; see `LICENSE` and `src/00readme.txt`.

The PlatformIO scripts put these headers ahead of the SDK headers and rebuild
the entire component instead of linking the precompiled SDK `libfatfs.a`.
This is necessary because exFAT changes the FatFs public structure layouts.
The headers script deliberately rejects incompatible SDK updates; update this
component alongside the SDK rather than removing that check.

`SD_MMC.begin()` now accepts FAT12/16/32 and exFAT without API changes.
Automatic formatting remains disabled. The sketch creates, verifies, and removes
only `/p4-sdmmc-test.bin` (replacing that file if it already exists).

exFAT filesystem support does not make Arduino's `File` interface 64-bit:
`seek()`, `position()`, and `size()` still have 32-bit limits on ESP32-P4.
Applications needing files over 4 GiB must use the native FatFs API with
`FSIZE_t` and audit their own size/offset types. The disk layer still uses 32-bit
sector addresses (512-byte SD sectors, up to 2 TiB); GPT/64-bit LBA is not enabled.

## Host Regression Test

Run `python3 test/host_fatfs/run.py` with a C compiler, dosfstools, and exfatprogs
installed. It uses the installed P4 SDK configuration, single-threaded host
mutex stubs, and temporary disk images, never a physical card. FAT12, FAT16,
FAT32, and exFAT are formatted by independent system tools, exercised through
the vendored library, and checked with the corresponding filesystem checker.
An MBR-partitioned exFAT image is also tested. Address and undefined-behavior
sanitizers are enabled. Under a ptrace-based sandbox, run with
`ASAN_OPTIONS=detect_leaks=0` because LeakSanitizer cannot operate there.
