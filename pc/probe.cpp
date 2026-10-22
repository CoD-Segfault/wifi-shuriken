#include "controller_hal.h"
#include "ftdi_link.h"
#include "spi_protocol_shared.h"
#include <cstdio>
#include <cstring>

// Deliberately an explicit hardware probe, not a second copy of scanner policy.
// No scanning, SD access, updates or GNSS reconfiguration is performed.
int main(int argc, char** argv) {
  if (argc < 3 || argc > 4 || (argc == 4 && std::strcmp(argv[3], "--direct") != 0)) {
    std::fprintf(stderr, "Usage: %s TIGARD_SERIAL UART_DEVICE [--direct]\n", argv[0]);
    return 2;
  }
  controller::pc::SteadyClock clock;
  controller::pc::SerialStream gnss;
  if (!gnss.open(argv[2])) { std::perror("GNSS UART"); return 1; }
  controller::pc::FtdiLink usb;
  if (!usb.open(argv[1])) { std::fprintf(stderr, "Tigard: %s\n", usb.error()); return 1; }
  controller::pc::TigardConfig config;
  config.shift_register = argc != 4;
  controller::pc::TigardScannerTransport scanner(usb, clock, config);
  if (!scanner.begin()) { std::fprintf(stderr, "MPSSE initialization failed\n"); return 1; }
  for (uint8_t slot = 0; slot < scanner.slotCount(); ++slot) {
    uint8_t tx[SPI_FRAME_SIZE] = {CMD_ID}, rx[SPI_FRAME_SIZE] = {};
    if (!scanner.transfer(slot, 10000000, tx, rx)) return 1;
    clock.sleepMicroseconds(500);
    tx[0] = CMD_NOP;
    bool found = false;
    for (unsigned pull = 0; pull < 10; ++pull) {
      if (!scanner.transfer(slot, 10000000, tx, rx)) return 1;
      DeviceIdReply id = {};
      std::memcpy(&id, rx, sizeof(id));
      if (rx[SPI_FRAME_TAG_INDEX] == CMD_ID && id.proto_version == PROTO_VERSION) {
        std::printf("S%u protocol=%u type=%u\n", slot, id.proto_version, id.scanner_type);
        found = true; break;
      }
      clock.sleepMicroseconds(500);
    }
    if (!found) std::printf("S%u no compatible reply\n", slot);
  }
  // Bounded UART sample; use the same ByteStream contract as embedded GNSS.
  const uint32_t start = clock.milliseconds();
  while (uint32_t(clock.milliseconds() - start) < 3000) {
    uint8_t data[512];
    const int count = gnss.read(data, sizeof(data));
    if (count < 0) { std::perror("GNSS read"); return 1; }
    if (count) std::fwrite(data, 1, count, stdout);
    clock.sleepMicroseconds(1000);
  }
  return scanner.shutdown() ? 0 : 1;
}
