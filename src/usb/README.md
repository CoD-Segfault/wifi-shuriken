# Project-local MTP transport

`mtp_device.c` is derived from the TinyUSB MTP driver bundled with the pinned
Arduino-Pico framework (`832f2c0650585775270b952e49a0623079c6cd42`). Its upstream
MIT license remains in the source. Public headers still come from that framework.

The controller builds this source as a project object. The PlatformIO middleware
in `scripts/use_local_mtp.py` excludes the framework's original `mtp_device.c`
from compilation. Other TinyUSB classes are unchanged; this replacement does not
modify installed SDK files. Verify the link map when changing build settings.

Local changes validate command/transfer lengths, bound response buffers, finish
class-reset control transfers, invalidate deferred callbacks on reset/cancel,
and recover abandoned transactions after an inactivity timeout. USB bulk stalls
are confined to MTP endpoints: CDC console/NMEA stay available. Hosts recover via
endpoint clear-halt or MTP DeviceReset; filesystem cleanup stays on core0.
