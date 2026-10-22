# Waveshare ESP32-P4-WIFI6 bring-up

The `waveshare_p4_controller` environment builds on the hardware-tested
`p4_testing` SDMMC/BLE bring-up. It now runs the shared six-scanner scheduler,
UART GNSS parser, deduplication, and WiGLE CSV logger alongside BLE UART.
The native USB variant includes read-only SD MTP; its composite USB behavior
still needs hardware validation. Controller firmware updates are not ported.
Existing XIAO and Pico
environments and default build targets are unchanged.

## Build and upload

```sh
pio run -e waveshare_p4_controller
pio run -e waveshare_p4_controller -t upload
pio device monitor -e waveshare_p4_controller
```

The PlatformIO executable may be `/home/segfault/.platformio/penv/bin/pio`.
The target pins [pioarduino release `55.03.312`](https://github.com/pioarduino/platform-espressif32/releases/tag/55.03.312), Arduino `3.3.12`, and SDK libraries
`5.5.5+sha.b774170ff46`. The pre-build guard rejects other versions or incompatible
FatFs headers. Board ID `esp32-p4_r3-evboard` intentionally selects production
revision-3 P4 support and the `esp32p4` variant; do not bypass revision checks.
Monitor uses `Serial0`, 115200 baud, with local echo; upload uses 921600 baud.

For the native USB OTG connector, use the separate console variant:

```sh
pio run -e waveshare_p4_usb_controller
pio run -e waveshare_p4_usb_controller -t upload --upload-port /dev/serial/by-id/<UART-adapter>
pio device monitor -b 115200 --port /dev/serial/by-id/<native-USB-device>
```

The base board definition selects `ARDUINO_USB_MODE=1` (hardware USB Serial/JTAG),
which does not start the high-speed OTG device. The USB variant removes that
setting, selects `ARDUINO_USB_MODE=0`, retains CDC-on-boot and routes application
console I/O to `Serial`/TinyUSB. The normal `waveshare_p4_controller` still uses
`Serial0` on the UART programming connector.

On [Waveshare ESP32-P4-WIFI6](https://www.waveshare.com/wiki/ESP32-P4-WIFI6),
the Type-C programming connector and four-pin high-speed USB OTG connector are
separate. The programming connector is one upload option. Native HS USB also
supports ROM download: close both CDC clients, hold BOOT, press and release RESET,
then release BOOT. Select the serial device that appears in download mode and run:

```sh
pio run -e waveshare_p4_usb_controller -t upload --upload-port /dev/ttyACM0
```

Replace `/dev/ttyACM0` with the ROM download device on your machine. Reset after
upload if the application does not restart automatically, then reopen its CDC
ports. The ROM device replaces the application console/NMEA pair during flashing.
The pinned Arduino SDK implements its CDC reboot-to-bootloader helper only for
ESP32-S2/S3, so the P4 native console currently requires manual BOOT/RESET entry;
a 1200-baud touch does not enter download mode. See Espressif
[USB download guidance](https://docs.espressif.com/projects/esp-techpedia/en/latest/esp-friends/get-started/try-firmware/try-firmware-hardware/esp32p4.html).

Use a data cable and an OTG device connection
to the host. There is no indefinite wait for a terminal; if startup messages are
missed, send `:status` or wait for the periodic status output. This variant exposes
two CDC interfaces: CDC0 is the application console and CDC1 is raw NMEA.
Low-level SDK messages may still use UART. After flashing, reopen the native USB
device so the host discovers both serial ports; do not assume fixed tty numbers.

The native USB variant also exposes an **MTP** interface for browsing and copying
root-level SD files, alongside both CDC ports. Reflash and reconnect the USB cable
so the host discovers the new composite device. Open Shuriken / SD in an
MTP-capable file manager. Writes, deletion and formatting are unsupported.
Directory entries are visible, but their children are not exported; files larger
than the ordinary MTP 32-bit object-size limit are omitted. Refresh/reopen the
storage listing to discover newly created logs.

MTP object information reports the SD file's stored modification time in UTC,
refreshed on each metadata request. FatFs records write times from the system
clock synchronized by GNSS, at FAT's two-second resolution. Existing stored dates
are preserved; unknown dates are not replaced with the current time.

The logger flushes before listing, inspecting or opening files. Downloads use a
size snapshot, so a currently growing CSV is consistent through its advertised
length; reopen it to fetch later rows. Reads run in bounded 4 KiB chunks from the
application loop, sharing its SD owner with logging. A slow/disconnected host
never holds a logging lock. Active transfers release their read handle on cancel,
USB reset or disconnect. The normal UART build does not expose MTP.

The second port accepts checksum-bearing RMC/GGA sentences as the shared USB
GNSS source. It also mirrors the hardware UART's raw NMEA output by default
(`NMEA_PASSTHROUGH=1` in the USB variant). Incoming USB data feeds the parser,
not the receiver UART or console. A conventional 115200 baud setting is suitable;
CDC data transfer does not depend on that virtual baud rate. NMEA output does
not require DTR and is best-effort/nonblocking when the host is slow or absent.
The NMEA interface does not implement Arduino's 1200-baud reboot gesture.


## SD card

SDMMC slot 0 uses four-bit mode at 20 MHz. CLK/CMD/D0/D1/D2/D3 are GPIO
43/44/39/40/41/42. The board variant supplies LDO4 and active-low GPIO45 card power.
After each mount, GPIO39-44 are set to `GPIO_DRIVE_CAP_2` before application I/O.
This preserves the tested fix for data-dependent multi-block write errors.

The complete matching FatFs component is vendored with exFAT enabled. Both
`p4_fatfs_headers.py` (pre) and `p4_fatfs_build.py` (post) are required so the
application, Arduino libraries, and rebuilt FatFs agree on public structure
layouts. The SDK's precompiled FatFs archive must not be linked too. Preserve
the vendored licenses; see `third_party/fatfs/README.md`.

A project-local linker wrapper repairs Arduino's missing `DEINIT_ARG` flag before
mounting. This keeps FatFs cleanup calling `sdmmc_host_deinit_slot(slot)` correctly,
including mount-error cleanup, without editing installed SDK sources. It does
not establish that all SD/C6 lifecycle interactions are hardware-safe; that still
requires testing. Normal operation deliberately leaves SD mounted.

Automatic formatting is disabled. Missing-card startup still attempts BLE. There
is no automatic remount or hotplug recovery yet; insert the card and reboot.
Native filesystem detection assumes SD is the only mounted FatFs volume (`0:`).
Large-file/GPT support is not added.

## External scanner and GNSS wiring

Defaults live in `include/p4_controller_config.h` and can be overridden with
build flags. Compile-time checks reject duplicate pins, reserved board pins,
or anything other than the six-scanner shift-register configuration.

| Signal | P4 GPIO |
| --- | --- |
| Shared scanner SPI SCK | 20 |
| Shared scanner SPI MOSI / shift-register data | 22 |
| Shared scanner SPI MISO | 21 |
| Shift-register latch | 26 |
| Shift-register OE (active low) | 27 |
| GNSS TX (P4 output → receiver RX) | 32 |
| GNSS RX (P4 input ← receiver TX) | 33 |

GPIO23 and GPIO46-48 remain spare.
Scanner transfers default to 10 MHz following wired hardware validation; shift-register
updates remain at 1 MHz. `SCANNER_SPI_CLOCK` can override the scanner clock.
P4 scanner frames have a minimum 1 ms gap after the preceding frame completes,
plus 5 µs CS setup and hold delays. Existing command-specific response waits
remain in place; the gap only adds any missing time. These conservative defaults
are overridable with `P4_SCANNER_FRAME_GAP_US`, `P4_SCANNER_CS_SETUP_US`, and
`P4_SCANNER_CS_HOLD_US`. RP2350 transport timing is unchanged.

Use the existing six-scanner shift-register
chip-select wiring, common ground, and compatible 3.3 V logic. The scanner SPI
clock also clocks the shift register; outputs change only when latched. SPI
initialization failure leaves outputs disabled.

The scanner worker owns SPI on core 1 and yields on every scheduler pass. It
passes observations and diagnostics to the core-0 application loop through
bounded, nonblocking queues. All SD files and GNSS parsing are owned by that
loop. Arduino application and event tasks are explicitly pinned to core 0 to
match the prebuilt NimBLE host; a compile-time guard checks the application/host
affinity. This keeps BLE and GNSS together for future phone NMEA input without
moving SPI or rebuilding the SDK. BLE callbacks still enqueue only: sharing a
core does not make it safe to access the parser or files from callbacks. Slow
SD I/O can still delay application GNSS/BLE servicing; it is not a separate task.
`:status` reports scanner presence, queue occupancy/drops, worker heartbeat,
GNSS fix, clock validity, CSV state, row count, and the active file. With no
scanners wired, `present=0/6` is expected; BLE and local diagnostics still work.

Logging uses the mounted SDMMC volume through a checked VFS file backend, not
a second SPI SD driver. Your default `P4_REQUIRE_GNSS_TIME_FOR_LOGGING=0` permits
`shuriken_NO_GNSS_TIME.csv` (with a unique suffix if needed) before a valid clock
exists. Missing time/location fields remain blank; no time or fix is invented.
Once selected, the filename stays unchanged as GNSS supplies time and location.
A newly acquired fix requests the same cooldown-limited dedupe reset as the
RP2350 controller so observations are logged again with location. Set
`P4_REQUIRE_GNSS_TIME_FOR_LOGGING=1` to require a valid clock before opening a
scan log. Blank-time/location records may not be useful for WiGLE uploads.
Reset diagnostics still wait for valid time. NMEA console passthrough is off
by default. GPS selection prefers a usable UART fix, then USB NMEA where
supported, then BLE NMEA. The native `waveshare_p4_usb_controller` build supports
all three sources; the UART-console build supports UART and BLE. If none qualifies, the logger
retains the hardware parser's clock/per-field fallback behavior.

To supply phone GNSS, write standard checksum-bearing RMC/GGA sentences with
newline endings to the BLE RX characteristic below. Writes can contain several
sentences or arbitrary fragments of one. RMC-only phone fixes are accepted
without a satellite count and expire after 3 seconds without a location update.
BLE parsing runs on the application loop, never in callbacks. Disconnect,
subscription/session changes, and detected RX drops clear the BLE parser/fix;
reconnecting requires fresh NMEA. A 128-byte sentence buffer rejects oversized
or binary input; TinyGPS validates checksums. `:status` shows the selected source,
per-source fix state, BLE parsed characters and checksum failures. Acquiring a
BLE fix after losing all fixes triggers the existing dedupe-reset/relog policy.

## BLE UART

The P4 runs Arduino's bundled NimBLE host; the onboard C6 supplies the radio via
ESP-Hosted SDIO slot 1. CLK/CMD/D0/D1/D2/D3 are GPIO18/19/14/15/16/17; reset is
GPIO54. No Wi-Fi credentials or C6 firmware changes are needed for this baseline.

Connect a BLE UART client to **Shuriken-P4**. The Nordic UART Service UUIDs are:

- Service: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
- Board RX (phone writes): `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`
- Board TX (phone subscribes): `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`

Subscribe to TX to receive the greeting. Phone writes are never echoed back over
BLE. Ordinary text is printed on Serial0; NMEA chunks go to the GPS parser without
console mirroring. Application console output (scanner diagnostics, logger messages,
local command responses) is mirrored to BLE while connected and subscribed.
Mirroring is nonblocking and lossy when the bounded BLE TX queue fills, independent
of UART draining. Output produced before connection is not replayed. SDK/boot logs
written directly to Serial0/Serial are not captured by this application mirror.
Received BLE text is explicitly excluded from mirroring to prevent echo loops.
Ordinary serial input is sent to the phone, flushing on
newline, a full buffer, or 50 ms idle. The `:` commands below are local serial
commands, never BLE commands.

`p4::bleWrite()` is a bounded, best-effort enqueue API, not a delivery guarantee.
TX uses 32 queued chunks of at most 20 bytes; `blePoll()` processes at most four
per pass. RX permits four queued writes of at most 512 bytes. Callback code never
performs SD/serial I/O. All application-facing BLE calls belong to the main loop.
Counters report discarded RX messages and TX bytes. Session tags discard stale
queued traffic after disconnect or subscription changes; advertising restarts on
disconnect. Queues do not wait for space, and serial diagnostics are lossy.

This is a one-phone, one-shot-initialization bring-up service. It is intentionally
unauthenticated: any connected client can supply location and UTC time and subscribe
to console diagnostics, which can include location data. Only use
trusted clients during bring-up. NMEA checksum validation is not authentication.
Private-log transfer and privileged commands are not implemented. The Shuriken
Android app now supports BLE location sharing and displays notifications on its
Serial Console tab. Mixed text/NMEA chunks are treated as NMEA traffic and not
mirrored to the console.

## Manual concurrent smoke test

### GPS-source comparison (debug only)

Enter `:gps-debug` on local Serial0 for one snapshot, or `:gps-debug on` to
sample every 2 seconds for 30 seconds. `:gps-debug off` stops early. Lines are
emitted one per 100 ms, without blocking normal scanning/logging/BLE work, and
appear in Android's BLE Serial Console too. These commands are local serial
commands, not BLE commands. No debug command changes source priority, clock
synchronization, dedupe policy, or the active CSV file. Debugging is off at boot.

Each source shows its usable-fix state, position and age, altitude/HDOP/satellites
with their ages, NMEA UTC/date/time ages, parsed-character and checksum counters,
and fix-bearing sentence count. USB is available in the native USB build and
explicitly unavailable in the UART-console build. Counters
are NMEA statistics, not claimed GNSS update frequencies; BLE counters reset
with its parser/session. Pairwise separation is printed only for fresh valid
positions. All samples are the latest observations, not synchronized epochs:
motion, different ages and different filtering can produce separation even
when both receivers work correctly. HDOP values are not a common accuracy
measurement across devices, and altitude references may differ.

For a useful first test, keep the phone and UART receiver stationary side by
side with a clear sky view, enable BLE NMEA, and capture a 30-second watch after
both show usable fixes. Compare position separation, UTC, freshness and checksum
counts. Then, while still sending BLE NMEA, disconnect UART data and repeat to
verify source fallback after the UART position ages out. Reconnect UART to
verify priority returns; disconnect BLE to check its parser/fix clears. Do not
change wiring while powered if it risks shorting adjacent pins. This is a
comparison of receivers and transport, not an absolute accuracy calibration.

### SD/BLE/scanner test

1. Boot with a FAT or exFAT card and check `:status` in the serial monitor.
2. Connect the phone, subscribe to TX, and exchange text in both directions.
3. Enter `:sd-test` followed by newline on Serial0, then send phone data while it
   runs for **10 seconds**. It repeats a check roughly every 100 ms, creating a
   uniquely named `/shuriken-p4-test-*.bin`, writing/reading/verifying 4096 bytes
   in one buffer, and removing the scratch file. Progress and BLE/drop status
   appear about once per second, followed by PASS/FAIL and totals. An I/O failure
   stops the test early. Existing files are never intentionally replaced. SD
   stays mounted and BLE is not stopped. Repeating the command while a test is
   active does not restart or extend it.
4. Check PASS and the queue-drop counters with `:status`. Repeat with both card
   formats, then disconnect/reconnect the phone and verify no stale text replays.
5. Boot without a card and verify BLE still works.
6. With the new external wiring, check scanner presence and CSV rows using
   `:status`; acquire a GNSS fix and verify location-bearing observations are
   logged again. Repeat while exchanging BLE text.
7. Without a usable UART fix, send fresh RMC/GGA NMEA over BLE RX and check
   `GPS source=BLE`, an increasing row count, and coordinates in the CSV. Stop
   sending for over 3 seconds or disconnect: BLE should no longer qualify.
   Restore a UART fix to check that it takes priority. BLE GPS fallback still
   needs hardware validation; scanner activity has been verified on hardware.

Each individual SD check is synchronous on the application loop, but the loop
services BLE and serial between checks throughout the 10-second test. BLE host
callbacks continue to enqueue; a long individual SD stall can still overflow the
bounded queues. The duration is not a hard deadline if SD I/O itself stalls. This is a smoke
test, not a throughput/lifecycle qualification, and a failed test or power loss
may leave a scratch file. Do not run a stress test against meaningful user files.

Host-only regression checks (never access a physical card):

```sh
ASAN_OPTIONS=detect_leaks=0 python3 test/host_p4_mount/run.py
ASAN_OPTIONS=detect_leaks=0 python3 test/host_p4_ble/run.py
ASAN_OPTIONS=detect_leaks=0 python3 test/host_p4_loop/run.py
ASAN_OPTIONS=detect_leaks=0 python3 test/host_p4_logger/run.py
ASAN_OPTIONS=detect_leaks=0 python3 test/host_p4_gnss/run.py
ASAN_OPTIONS=detect_leaks=0 python3 test/host_fatfs/run.py
```

The filesystem test needs `dosfstools`, `exfatprogs`, and the matching installed SDK.
The logger test uses the real CSV logger, TinyGPS parser, and VFS backend with
temporary host files, including both clock policies, checked flush failure and
recovery, append preservation, and reset records; it never changes the host clock.
The BLE test uses single-threaded API fakes to check the real module's queue limits,
20-byte chunks, polling budget, session isolation, and startup failure path; it
does not simulate radio timing or prove callback concurrency safety.
Leak detection is disabled here for ptrace-based sandboxes; address/UB checks stay
enabled. Hardware testing is still required for concurrent SDMMC/ESP-Hosted use.
