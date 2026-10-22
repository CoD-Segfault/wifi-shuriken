#include "controller/scanner_update.h"
#include "controller/application.h"
#include "spi_protocol_shared.h"
#include <cstdio>
#include <cstring>
#include <new>

namespace controller {
namespace {
constexpr size_t SCANNER_FRAME_SIZE = SPI_FRAME_SIZE;
constexpr size_t SCANNER_FRAME_TAG_INDEX = SPI_FRAME_TAG_INDEX;
constexpr uint32_t SCANNER_OTA_PROGRESS_BYTES = 32768;
constexpr uint32_t SCANNER_OTA_POST_SLOT_DELAY_MS = 250;
class UpdateText {
 public:
  explicit UpdateText(Output& output) : output_(output) {}
  void print(const char* text) { output_.text(text); }
  void print(char* text) { print(static_cast<const char*>(text)); }
  template<class T> void print(T value, int base = 10) {
    char text[32];
    std::snprintf(text, sizeof(text), base == 16 ? "%llX" : "%llu", static_cast<unsigned long long>(value));
    output_.text(text);
  }
  template<class T> void println(T value, int base = 10) { print(value, base); print("\r\n"); }
  void println(char* value) { println(static_cast<const char*>(value)); }
  void println(const char* value) { print(value); print("\r\n"); }
 private:
  Output& output_;
};
class Updater {
 public:
  Updater(hal::ScannerTransport& transport, hal::Clock& clock, const ScannerUpdateConfig& config)
    : transport_(transport), clock_(clock), config_(config), hz_(config.normal_hz) {}
  bool transfer(const uint8_t* tx, uint8_t* rx) { return transport_.transfer(active_slot_, hz_, tx, rx); }
  void scannerOtaInterframeWait() {
    clock_.sleepMicroseconds(config_.interframe_us ? config_.interframe_us : config_.normal_interframe_us);
  }
  void drain(unsigned count) {
    uint8_t nop[SPI_FRAME_SIZE] = {}, rx[SPI_FRAME_SIZE];
    for (unsigned i = 0; i < count; ++i) {
      if (!transfer(nop, rx)) return;
      clock_.sleepMicroseconds(config_.normal_interframe_us);
    }
  }
  bool identify(DeviceIdReply& id) {
    uint8_t tx[SPI_FRAME_SIZE] = {}, rx[SPI_FRAME_SIZE] = {}, nop[SPI_FRAME_SIZE] = {};
    tx[0] = CMD_ID;
    for (unsigned attempt = 0; attempt < 8; ++attempt) {
      if (!transfer(tx, rx)) return false;
      clock_.sleepMicroseconds(config_.identify_first_pull_us);
      for (unsigned pull = 0; pull < 10; ++pull) {
        if (!transfer(nop, rx)) return false;
        if (rx[SPI_FRAME_TAG_INDEX] == CMD_ID) {
          std::memcpy(&id, rx, sizeof(id));
          if (id.proto_version == PROTO_VERSION) return true;
          break;
        }
        clock_.sleepMicroseconds(config_.normal_interframe_us);
      }
      clock_.sleepMicroseconds(1000);
    }
    return false;
  }
const char* scannerOtaStatusToString(int8_t status) {
  switch (status) {
    case OTA_STATUS_OK: return "OK";
    case OTA_STATUS_BUSY: return "BUSY";
    case OTA_STATUS_INVALID: return "INVALID";
    case OTA_STATUS_NOT_ACTIVE: return "NOT_ACTIVE";
    case OTA_STATUS_SEQUENCE_MISMATCH: return "SEQUENCE_MISMATCH";
    case OTA_STATUS_CRC_MISMATCH: return "CRC_MISMATCH";
    case OTA_STATUS_WRITE_FAILED: return "WRITE_FAILED";
    case OTA_STATUS_SIZE_MISMATCH: return "SIZE_MISMATCH";
    case OTA_STATUS_HASH_MISMATCH: return "HASH_MISMATCH";
    case OTA_STATUS_BEGIN_FAILED: return "BEGIN_FAILED";
    case OTA_STATUS_END_FAILED: return "END_FAILED";
    default: return "UNKNOWN";
  }
}

bool scannerOtaCommandWithAck(const uint8_t* cmd_frame,
                                     uint8_t expected_cmd_tag,
                                     OtaAck& ack,
                                     uint16_t max_pulls = 0,
                                     uint32_t expected_offset = UINT32_MAX,
                                     uint16_t expected_next_sequence = UINT16_MAX) {
  if (!max_pulls) max_pulls = config_.response_pulls;
  uint8_t rx[SCANNER_FRAME_SIZE] = {0};
  uint8_t nop[SCANNER_FRAME_SIZE] = {0};
  nop[0] = CMD_NOP;

  auto frameLooksLikeAck = [&](const uint8_t* frame) -> bool {
    OtaAck candidate = {};
    memcpy(&candidate, frame, sizeof(candidate));
    const bool tag_match = frame[SCANNER_FRAME_TAG_INDEX] == expected_cmd_tag;
    const bool payload_match =
        candidate.active != 0 &&
        candidate.status <= OTA_STATUS_OK &&
        candidate.status >= OTA_STATUS_END_FAILED &&
        (expected_offset == UINT32_MAX || candidate.offset == expected_offset) &&
        (expected_next_sequence == UINT16_MAX ||
         candidate.next_sequence == expected_next_sequence);

    if (tag_match || payload_match) {
      ack = candidate;
      return true;
    }
    return false;
  };

  if (!transfer(cmd_frame, rx)) return false;
  if (frameLooksLikeAck(rx)) {
    return true;
  }
  if (config_.first_pull_us > 0) {
    clock_.sleepMicroseconds(config_.first_pull_us);
  }

  for (uint16_t pull = 0; pull < max_pulls; pull++) {
    if (!transfer(nop, rx)) return false;
    if (frameLooksLikeAck(rx)) {
      return true;
    }
    scannerOtaInterframeWait();
  }

  return false;
}

bool scannerOtaAbortActiveTransfer() {
  uint8_t tx[SCANNER_FRAME_SIZE] = {0};
  OtaAck ack = {};
  tx[0] = CMD_OTA_ABORT;
  return scannerOtaCommandWithAck(tx, CMD_OTA_ABORT, ack, 80) &&
         ack.status == OTA_STATUS_OK;
}

bool scannerOtaReadImageHash(ScannerUpdateFile& source,
                                    uint32_t image_size,
                                    uint64_t& image_hash,
                                    UpdateText& serial) {
  if (!source.seekSet(0)) {
    serial.println("scanner.bin seek failed before hashing");
    return false;
  }

  std::unique_ptr<uint8_t[]> buffer(new (std::nothrow) uint8_t[config_.hash_chunk_bytes]);
  if (!buffer) { serial.println("Scanner update hash buffer allocation failed"); return false; }
  uint64_t hash = OTA_HASH64_INIT;
  uint32_t total_read = 0;
  while (total_read < image_size) {
    const uint32_t remaining = image_size - total_read;
    const size_t want = (remaining < config_.hash_chunk_bytes) ? remaining : config_.hash_chunk_bytes;
    const int bytes_read = source.read(buffer.get(), want);
    if (bytes_read <= 0 || static_cast<size_t>(bytes_read) > want) {
      serial.println("scanner.bin read failed while hashing");
      return false;
    }
    hash = otaHash64Update(hash, buffer.get(), static_cast<size_t>(bytes_read));
    total_read += static_cast<uint32_t>(bytes_read);
  }

  image_hash = hash;
  return source.seekSet(0);
}

bool scannerOtaSendBegin(uint32_t image_size,
                                uint64_t image_hash,
                                UpdateText& serial,
                                uint8_t slot) {
  OtaBeginCommand begin = {};
  begin.cmd = CMD_OTA_BEGIN;
  begin.packet_data_bytes = OTA_DATA_BYTES_PER_FRAME;
  begin.image_size = image_size;
  begin.image_hash = image_hash;

  OtaAck ack = {};
  if (!scannerOtaCommandWithAck(reinterpret_cast<const uint8_t*>(&begin),
                                CMD_OTA_BEGIN,
                                ack,
                                config_.response_pulls,
                                0,
                                0)) {
    serial.print("S");
    serial.print(slot);
    serial.println(" scanner OTA begin timed out");
    return false;
  }
  if (ack.status != OTA_STATUS_OK) {
    serial.print("S");
    serial.print(slot);
    serial.print(" scanner OTA begin rejected: ");
    serial.println(scannerOtaStatusToString(ack.status));
    return false;
  }
  return true;
}

bool scannerOtaSendDataPacket(const OtaDataCommand& packet,
                                     uint32_t next_offset,
                                     UpdateText& serial,
                                     uint8_t slot) {
  uint8_t nop[SCANNER_FRAME_SIZE] = {0};
  nop[0] = CMD_NOP;

  for (uint8_t attempt = 0; attempt < config_.packet_retries; attempt++) {
    uint8_t rx[SCANNER_FRAME_SIZE] = {0};
    OtaAck ack = {};
    const auto frameLooksLikeDataAck = [&](const uint8_t* frame) -> bool {
      OtaAck candidate = {};
      memcpy(&candidate, frame, sizeof(candidate));
      const bool tag_match = frame[SCANNER_FRAME_TAG_INDEX] == CMD_OTA_DATA;
      const bool payload_match =
          candidate.active != 0 &&
          candidate.status <= OTA_STATUS_OK &&
          candidate.status >= OTA_STATUS_END_FAILED &&
          candidate.offset == next_offset &&
          (candidate.next_sequence == static_cast<uint16_t>(packet.sequence + 1) ||
           candidate.status == OTA_STATUS_SEQUENCE_MISMATCH);
      if (tag_match || payload_match) {
        ack = candidate;
        return true;
      }
      return false;
    };

    if (!transfer(reinterpret_cast<const uint8_t*>(&packet), rx)) return false;
    if (frameLooksLikeDataAck(rx)) {
      if (ack.status == OTA_STATUS_OK && ack.offset == next_offset) {
        return true;
      }
      if (ack.status == OTA_STATUS_SEQUENCE_MISMATCH && ack.offset == next_offset) {
        return true;
      }
      // Same command tag is used for every OTA data packet, so a delayed ACK
      // for the prior packet can arrive here. Keep polling unless the scanner
      // reports a non-recoverable error for this transfer.
      if (ack.status != OTA_STATUS_OK &&
          ack.status != OTA_STATUS_CRC_MISMATCH &&
          ack.status != OTA_STATUS_SEQUENCE_MISMATCH) {
        serial.print("S");
        serial.print(slot);
        serial.print(" scanner OTA data failed at offset ");
        serial.print(packet.offset);
        serial.print(": ");
        serial.println(scannerOtaStatusToString(ack.status));
        return false;
      }
    }

    if (config_.first_pull_us > 0) {
      clock_.sleepMicroseconds(config_.first_pull_us);
    }

    for (uint16_t pull = 0; pull < config_.response_pulls; pull++) {
      if (!transfer(nop, rx)) return false;
      if (!frameLooksLikeDataAck(rx)) {
        scannerOtaInterframeWait();
        continue;
      }

      if (ack.status == OTA_STATUS_OK && ack.offset == next_offset) {
        if (scannerOtaShouldLogPacket(packet.offset, packet.length)) {
          serial.print("S");
          serial.print(slot);
          serial.print(" scanner OTA ACK data seq=");
          serial.print(packet.sequence);
          serial.print(" offset=");
          serial.print(packet.offset);
          serial.print(" after_pulls=");
          serial.println(pull + 1);
        }
        return true;
      }

      // A lost ACK can make the controller retry a packet the scanner already
      // accepted. Treat the scanner's advanced offset as idempotent success.
      if (ack.status == OTA_STATUS_SEQUENCE_MISMATCH && ack.offset == next_offset) {
        return true;
      }

      if (ack.status == OTA_STATUS_CRC_MISMATCH ||
          ack.status == OTA_STATUS_SEQUENCE_MISMATCH ||
          (ack.status == OTA_STATUS_OK && ack.offset < next_offset)) {
        scannerOtaInterframeWait();
        continue;
      }

      serial.print("S");
      serial.print(slot);
      serial.print(" scanner OTA data failed at offset ");
      serial.print(packet.offset);
      serial.print(": ");
      serial.println(scannerOtaStatusToString(ack.status));
      return false;
    }
  }

  serial.print("S");
  serial.print(slot);
  serial.print(" scanner OTA data timed out at offset ");
  serial.println(packet.offset);
  return false;
}

bool scannerOtaSendEnd(uint32_t image_size,
                              uint64_t image_hash,
                              UpdateText& serial,
                              uint8_t slot) {
  OtaEndCommand end = {};
  end.cmd = CMD_OTA_END;
  end.image_size = image_size;
  end.image_hash = image_hash;

  OtaAck ack = {};
  if (!scannerOtaCommandWithAck(reinterpret_cast<const uint8_t*>(&end),
                                CMD_OTA_END,
                                ack,
                                config_.response_pulls,
                                image_size,
                                UINT16_MAX)) {
    serial.print("S");
    serial.print(slot);
    serial.println(" scanner OTA end timed out");
    return false;
  }
  if (ack.status != OTA_STATUS_OK) {
    serial.print("S");
    serial.print(slot);
    serial.print(" scanner OTA end rejected: ");
    serial.println(scannerOtaStatusToString(ack.status));
    return false;
  }
  return true;
}

bool scannerOtaShouldLogPacket(uint32_t offset, uint8_t length) {
  if (offset < 512) {
    return true;
  }
  if (!config_.flash_sector_bytes) return false;
  const uint32_t sector_offset = offset % config_.flash_sector_bytes;
  const uint32_t end_offset = (offset + length) % config_.flash_sector_bytes;
  return sector_offset < 128 ||
         sector_offset >= (config_.flash_sector_bytes - 128) ||
         end_offset < 128;
}

bool scannerOtaTransferImageToSlot(ScannerUpdateStorage& sd,
                                          uint8_t slot,
                                          uint32_t image_size,
                                          uint64_t image_hash,
                                          UpdateText& serial) {
  hz_ = config_.update_hz;

  auto source = sd.open(config_.image_path);
  if (!source) {
    serial.println("scanner.bin disappeared before scanner OTA");
    hz_ = config_.normal_hz;
    return false;
  }
  if (!source->seekSet(0)) {
    serial.println("scanner.bin seek failed before scanner OTA");
    source->close();
    hz_ = config_.normal_hz;
    return false;
  }

  active_slot_ = slot;
  drain(2);

  if (!scannerOtaSendBegin(image_size, image_hash, serial, slot)) {
    source->close();
    hz_ = config_.normal_hz;
    return false;
  }

  uint32_t offset = 0;
  uint16_t sequence = 0;
  uint32_t next_progress = SCANNER_OTA_PROGRESS_BYTES;
  while (offset < image_size) {
    OtaDataCommand packet = {};
    packet.cmd = CMD_OTA_DATA;
    packet.sequence = sequence;
    packet.offset = offset;

    const uint32_t remaining = image_size - offset;
    uint32_t packet_limit = OTA_DATA_BYTES_PER_FRAME;
    if (config_.flash_sector_bytes) {
    const uint32_t sector_offset = offset % config_.flash_sector_bytes;
    if (sector_offset != 0) {
      const uint32_t bytes_to_sector_end =
          config_.flash_sector_bytes - sector_offset;
      if (bytes_to_sector_end < packet_limit) {
        packet_limit = bytes_to_sector_end;
      }
    }
    }
    const uint8_t want = (remaining < packet_limit)
      ? static_cast<uint8_t>(remaining)
      : static_cast<uint8_t>(packet_limit);
    const int bytes_read = source->read(packet.data, want);
    if (bytes_read != want) {
      serial.print("S");
      serial.print(slot);
      serial.println(" scanner.bin read failed during scanner OTA");
      scannerOtaAbortActiveTransfer();
      source->close();
      hz_ = config_.normal_hz;
      return false;
    }

    packet.length = want;
    packet.crc32 = otaCrc32(packet.data, packet.length);
    const uint32_t next_offset = offset + packet.length;
    if (scannerOtaShouldLogPacket(packet.offset, packet.length)) {
      serial.print("S");
      serial.print(slot);
      serial.print(" scanner OTA TX data seq=");
      serial.print(packet.sequence);
      serial.print(" offset=");
      serial.print(packet.offset);
      serial.print(" len=");
      serial.print(packet.length);
      if (config_.flash_sector_bytes) {
        serial.print(" sector_off="); serial.print(packet.offset % config_.flash_sector_bytes);
      }
      serial.print(" next=");
      serial.print(next_offset);
      serial.print(" crc=0x");
      serial.println(packet.crc32, 16);
    }
    if (!scannerOtaSendDataPacket(packet, next_offset, serial, slot)) {
      scannerOtaAbortActiveTransfer();
      source->close();
      hz_ = config_.normal_hz;
      return false;
    }

    offset = next_offset;
    sequence++;

    if (offset >= next_progress || offset == image_size) {
      serial.print("S");
      serial.print(slot);
      serial.print(" scanner OTA progress ");
      serial.print(offset);
      serial.print("/");
      serial.println(image_size);
      while (next_progress <= offset) {
        next_progress += SCANNER_OTA_PROGRESS_BYTES;
      }
    }
  }

  source->close();

  if (!scannerOtaSendEnd(image_size, image_hash, serial, slot)) {
    hz_ = config_.normal_hz;
    return false;
  }

  clock_.sleepMicroseconds(SCANNER_OTA_POST_SLOT_DELAY_MS * 1000u);
  hz_ = config_.normal_hz;
  return true;
}

bool buildScannerAppliedPath(ScannerUpdateStorage& sd, char* out, size_t out_len) {
  const char* const applied_base = config_.applied_path;
  if (!sd.exists(applied_base)) {
    snprintf(out, out_len, "%s", applied_base);
    return true;
  }

  const char* const slash = strrchr(applied_base, '/');
  const char* const dot = strrchr(applied_base, '.');
  const bool has_extension = (dot != nullptr) && (slash == nullptr || dot > slash);

  for (uint16_t suffix = 1; suffix < 100; suffix++) {
    if (has_extension) {
      const int stem_len = static_cast<int>(dot - applied_base);
      snprintf(out, out_len, "%.*s_%02u%s",
               stem_len, applied_base, (unsigned)suffix, dot);
    } else {
      snprintf(out, out_len, "%s_%02u", applied_base, (unsigned)suffix);
    }

    if (!sd.exists(out)) {
      return true;
    }
  }

  out[0] = '\0';
  return false;
}

bool run(ScannerUpdateStorage& sd, UpdateText& serial, ScannerUpdateObserver* observer) {
  if (!sd.exists(config_.image_path)) {
    return false;
  }

  auto source = sd.open(config_.image_path);
  if (!source) {
    serial.println("scanner.bin exists but could not be opened");
    return false;
  }

  if (source->size() > UINT32_MAX) { serial.println("scanner.bin exceeds the protocol image-size limit"); return false; }
  const uint32_t image_size = static_cast<uint32_t>(source->size());
  if (image_size == 0) {
    serial.println("scanner.bin is empty; skipping scanner OTA");
    source->close();
    return false;
  }

  uint64_t image_hash = 0;
  if (!scannerOtaReadImageHash(*source, image_size, image_hash, serial)) {
    source->close();
    return false;
  }
  source->close();

  if (!transport_.begin()) { serial.println("Scanner update bus initialization failed"); return false; }
  if (observer) observer->started();

  serial.print("Scanner SPI OTA image: ");
  serial.print(image_size);
  serial.print(" bytes hash=0x");
  serial.print(static_cast<uint32_t>(image_hash >> 32), 16);
  serial.println(static_cast<uint32_t>(image_hash & 0xffffffffUL), 16);

  uint8_t ota_capable_slots = 0;
  uint8_t updated_slots = 0;
  uint8_t failed_slots = 0;
  for (uint8_t slot = 0; slot < transport_.slotCount(); slot++) {
    active_slot_ = slot;
    drain(2);

    DeviceIdReply id = {};
    if (!identify(id)) {
      continue;
    }
    if ((id.capabilities & CAP_OTA_UPDATE) == 0) {
      serial.print("S");
      serial.print(slot);
      serial.println(" scanner does not advertise SPI OTA; skipping");
      continue;
    }

    ota_capable_slots++;
    serial.print("S");
    serial.print(slot);
    serial.println(" scanner OTA starting");
    if (scannerOtaTransferImageToSlot(sd, slot, image_size, image_hash, serial)) {
      updated_slots++;
      serial.print("S");
      serial.print(slot);
      serial.println(" scanner OTA complete");
    } else {
      failed_slots++;
      serial.print("S");
      serial.print(slot);
      serial.println(" scanner OTA failed; leaving scanner.bin for retry");
    }
  }

  if (ota_capable_slots == 0) {
    serial.println("No SPI OTA-capable scanners found; leaving scanner.bin in place");
    return false;
  }

  if (failed_slots != 0 || updated_slots != ota_capable_slots) {
    serial.println("Scanner OTA did not complete for every capable scanner; leaving scanner.bin in place");
    return updated_slots != 0;
  }

  char applied_path[56] = {};
  if (!buildScannerAppliedPath(sd, applied_path, sizeof(applied_path))) {
    serial.println("Scanner OTA applied, but no free applied filename was available");
    return true;
  }
  if (!sd.rename(config_.image_path, applied_path)) {
    serial.print("Scanner OTA applied, but rename failed: ");
    serial.println(applied_path);
    return true;
  }

  serial.print("Scanner OTA applied successfully; renamed SD image to ");
  serial.println(applied_path);
  return true;
}
 private:
  hal::ScannerTransport& transport_;
  hal::Clock& clock_;
  const ScannerUpdateConfig& config_;
  uint8_t active_slot_ = 0;
  uint32_t hz_;
};
}
bool updateScannerFirmware(ScannerUpdateStorage& storage, hal::ScannerTransport& transport,
                           hal::Clock& clock, Output& output, const ScannerUpdateConfig& config,
                           ScannerUpdateObserver* observer) {
  if (!config.enabled) return false;
  if (!config.image_path || !config.applied_path || !config.normal_hz || !config.update_hz ||
      !config.response_pulls || !config.packet_retries || !config.hash_chunk_bytes ||
      config.hash_chunk_bytes > 65536 || (config.flash_sector_bytes && config.flash_sector_bytes < OTA_DATA_BYTES_PER_FRAME) ||
      !transport.slotCount() || transport.slotCount() > 8) {
    output.text("Invalid scanner update configuration\r\n"); return false;
  }
  UpdateText console(output);
  Updater updater(transport, clock, config);
  return updater.run(storage, console, observer);
}
}
