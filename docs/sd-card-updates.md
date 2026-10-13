# XIAO SD card updates

These files are for the XIAO RP2350 controller and attached XIAO ESP32-C5 scanners.
The installed firmware must already support SD card updates.

- `controller.bin`: updates the XIAO RP2350 controller.
- `scanner.bin`: updates attached, OTA-capable XIAO ESP32-C5 scanners over SPI.

Power off the Shuriken, remove its SD card, and copy the desired `.bin` file(s)
from this folder to the card's root directory. Keep their filenames unchanged.
Do not copy the containing folder: the updater looks for `/controller.bin` and
`/scanner.bin`, not files in subdirectories.

Reinsert the card and power on. Leave power connected while updates run. The
controller stages its update and reboots to apply it; scanner updates run through
the controller. After successful updates, the files are renamed to
`controller.applied.bin` and `scanner.applied.bin` (or numbered variants if those
names already exist).

For USB flashing, use the `xiao_rp2350_controller.uf2` or
`xiao_esp32c5_scanner-merged.bin` provided outside this folder. Do not use the UF2
or merged scanner image for SD updates, and do not rename them to these filenames.
