# MTP regression tests

Run `bash test/host_mtp/run.sh` from the repository root. Requires a native C/C++
compiler with AddressSanitizer/UndefinedBehaviorSanitizer and the pinned
Arduino-Pico framework installed in `~/.platformio/packages/framework-arduinopico`.
Set `ARDUINO_PICO_DIR` to use another installation. No hardware is accessed.

The suite compiles the production C transport and C++ storage implementation
against the framework's actual MTP/TinyUSB headers. USB endpoint transfers and
SdFat are simulated. It covers streaming boundaries/ZLP, partial reads, bounded
error responses, read/submission failures, malformed commands, session handling,
root/format filtering, property metadata/timestamps, oversized files, cancellation
before dispatch/during data, reset during data, bus reset, clear-halt recovery,
storage loss and inactivity timeout including timer rollover. It also checks that
the logger can acquire the SD lock while a download awaits its next USB transfer.

Hardware acceptance: copy zero-length, small, and multi-buffer files and compare
their bytes with the SD originals; verify timestamps and filenames in OpenMTP
Kalam, Android and Windows/Linux clients. Cancel/retry downloads, disconnect and
reconnect USB, pause the reader, and confirm logging and both CDC interfaces keep
working. Nested folders intentionally remain outside the root-only MTP view.
