#include "controller/scanner_update.h"
#include "controller/application.h"
#include "hal/posix_update_storage.h"
#include "spi_protocol_shared.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace controller { Services& controllerServices() { throw std::logic_error("unused factory"); } }
using namespace controller;
struct Clock final : hal::Clock {
  uint32_t us = 0;
  uint32_t milliseconds() const override { return us / 1000; }
  uint32_t microseconds() const override { return us; }
  uint64_t microseconds64() const override { return us; }
  void sleepMicroseconds(uint32_t duration) override { us += duration; }
};
struct Console final : Output {
  std::string output;
  void write(const uint8_t* data, size_t length, bool = true) override { output.append(reinterpret_cast<const char*>(data), length); }
};
struct Store final : ScannerUpdateStorage {
  std::map<std::string, std::vector<uint8_t>> files;
  unsigned opens = 0, live = 0, closes = 0;
  bool fail_open = false, fail_seek = false, fail_rename = false, oversized = false;
  unsigned fail_read_on_open = 0, fail_reopen = 0;
  size_t read_chunk = 100000;
  struct File final : ScannerUpdateFile {
    Store& store;
    std::vector<uint8_t>& bytes;
    size_t position = 0;
    unsigned opening;
    bool closed = false;
    File(Store& s, std::vector<uint8_t>& data) : store(s), bytes(data), opening(s.opens) { ++store.live; }
    ~File() override { close(); }
    uint64_t size() const override { return store.oversized ? uint64_t(UINT32_MAX) + 1 : bytes.size(); }
    bool seekSet(uint32_t offset) override {
      if (store.fail_seek || offset > bytes.size()) return false;
      position = offset; return true;
    }
    int read(void* data, size_t capacity) override {
      if (opening == store.fail_read_on_open) return -1;
      const size_t count = std::min({capacity, bytes.size() - position, store.read_chunk});
      std::memcpy(data, bytes.data() + position, count); position += count;
      return static_cast<int>(count);
    }
    void close() override { if (!closed) { closed = true; --store.live; ++store.closes; } }
  };
  bool exists(const char* path) const override { return files.count(path) != 0; }
  std::unique_ptr<ScannerUpdateFile> open(const char* path) override {
    ++opens;
    if (fail_open || opens == fail_reopen || !exists(path)) return {};
    return std::make_unique<File>(*this, files[path]);
  }
  bool rename(const char* from, const char* to) override {
    if (fail_rename || exists(to) || !exists(from)) return false;
    files[to] = std::move(files[from]); files.erase(from); return true;
  }
  void image(size_t length = 173) {
    auto& bytes = files["/scanner.bin"]; bytes.resize(length);
    for (size_t i = 0; i < length; ++i) bytes[i] = static_cast<uint8_t>(i * 37);
  }
};
struct Observer final : ScannerUpdateObserver { unsigned calls = 0; void started() override { ++calls; } };
struct FakeTransport final : hal::ScannerTransport {
  struct Slot {
    bool present = true, capable = true;
    bool reject_begin = false, reject_end = false, fail_data = false, retry_crc = false, duplicate_ack = false;
    unsigned begins = 0, data_commands = 0, ends = 0, aborts = 0, delay = 0;
    unsigned delay_left = 0;
    std::array<uint8_t, SPI_FRAME_SIZE> pending = {};
    std::vector<uint8_t> received;
    std::vector<std::pair<uint32_t, uint8_t>> packets;
    uint16_t sequence = 0;
    uint32_t expected_size = 0;
    uint64_t expected_hash = 0;
    bool active = false;
  } slots[3];
  bool begin_ok = true, bus_error = false;
  unsigned begin_calls = 0, frames = 0;
  uint8_t count = 3;
  uint32_t normal_hz = 20000000, update_hz = 20000000;
  bool begin() override { ++begin_calls; return begin_ok; }
  uint8_t slotCount() const override { return count; }
  void reply(Slot& slot, uint8_t tag, int8_t status) {
    slot.pending.fill(0);
    OtaAck ack = {};
    ack.status = status; ack.active = slot.active;
    ack.offset = slot.received.size(); ack.next_sequence = slot.sequence;
    std::memcpy(slot.pending.data(), &ack, sizeof(ack));
    slot.pending[SPI_FRAME_TAG_INDEX] = tag; slot.delay_left = slot.delay;
  }
  bool transfer(uint8_t index, uint32_t hz, const uint8_t* tx, uint8_t* rx) override {
    assert(index < count && tx && rx); ++frames;
    if (bus_error) return false;
    auto& slot = slots[index];
    if (slot.delay_left) { --slot.delay_left; std::memset(rx, 0, SPI_FRAME_SIZE); }
    else { std::memcpy(rx, slot.pending.data(), SPI_FRAME_SIZE); slot.pending.fill(0); }
    if (!slot.present || tx[0] == CMD_NOP) return true;
    if (tx[0] == CMD_ID) {
      assert(hz == normal_hz);
      DeviceIdReply id = {PROTO_VERSION, SCANNER_TYPE_ESP32_C5,
        uint8_t(slot.capable ? CAP_OTA_UPDATE : 0), PROTO_MAX_RESULTS, {0}};
      std::memcpy(slot.pending.data(), &id, sizeof(id)); slot.pending[SPI_FRAME_TAG_INDEX] = CMD_ID;
      return true;
    }
    assert(hz == update_hz);
    if (tx[0] == CMD_OTA_BEGIN) {
      OtaBeginCommand command; std::memcpy(&command, tx, sizeof(command));
      assert(command.packet_data_bytes == OTA_DATA_BYTES_PER_FRAME);
      ++slot.begins; slot.active = !slot.reject_begin;
      slot.expected_size = command.image_size; slot.expected_hash = command.image_hash;
      slot.received.clear(); slot.sequence = 0;
      reply(slot, tx[0], slot.reject_begin ? OTA_STATUS_BEGIN_FAILED : OTA_STATUS_OK);
    } else if (tx[0] == CMD_OTA_DATA) {
      OtaDataCommand command; std::memcpy(&command, tx, sizeof(command)); ++slot.data_commands;
      assert(command.length && command.length <= OTA_DATA_BYTES_PER_FRAME);
      assert(command.crc32 == otaCrc32(command.data, command.length));
      if (slot.fail_data) { reply(slot, tx[0], OTA_STATUS_WRITE_FAILED); return true; }
      if (slot.retry_crc) { slot.retry_crc = false; reply(slot, tx[0], OTA_STATUS_CRC_MISMATCH); return true; }
      assert(command.offset == slot.received.size() && command.sequence == slot.sequence);
      slot.packets.push_back({uint32_t(command.offset), uint8_t(command.length)});
      slot.received.insert(slot.received.end(), command.data, command.data + command.length); ++slot.sequence;
      reply(slot, tx[0], slot.duplicate_ack ? OTA_STATUS_SEQUENCE_MISMATCH : OTA_STATUS_OK);
    } else if (tx[0] == CMD_OTA_END) {
      OtaEndCommand command; std::memcpy(&command, tx, sizeof(command)); ++slot.ends;
      assert(command.image_size == slot.received.size() && command.image_size == slot.expected_size);
      assert(command.image_hash == otaHash64Update(OTA_HASH64_INIT, slot.received.data(), slot.received.size()));
      assert(command.image_hash == slot.expected_hash);
      reply(slot, tx[0], slot.reject_end ? OTA_STATUS_END_FAILED : OTA_STATUS_OK); slot.active = false;
    } else if (tx[0] == CMD_OTA_ABORT) { ++slot.aborts; slot.active = false; reply(slot, tx[0], OTA_STATUS_OK); }
    else assert(false);
    return true;
  }
};
static ScannerUpdateConfig configuration() {
  ScannerUpdateConfig config;
  config.response_pulls = 4; config.packet_retries = 3;
  return config;
}
static bool run(Store& store, FakeTransport& scanners, ScannerUpdateConfig config = configuration()) {
  Clock clock; Console console; Observer observer;
  const bool result = updateScannerFirmware(store, scanners, clock, console, config, &observer);
  assert(store.live == 0);
  return result;
}
static void successAndRetries() {
  for (uint32_t rate : {20000000u, 10000000u}) {
    Store store; store.image(5003); const auto image = store.files["/scanner.bin"];
    store.files["/scanner.applied.bin"] = {9};
    FakeTransport scanner; scanner.normal_hz = scanner.update_hz = rate;
    scanner.slots[0].delay = 2; scanner.slots[1].retry_crc = true; scanner.slots[2].duplicate_ack = true;
    auto config = configuration(); config.normal_hz = config.update_hz = rate; config.flash_sector_bytes = 4096;
    assert(run(store, scanner, config));
    assert(!store.exists("/scanner.bin") && store.files["/scanner.applied.bin"] == std::vector<uint8_t>{9});
    assert(store.files["/scanner.applied_01.bin"] == image);
    assert(store.closes == 4 && scanner.begin_calls == 1);
    for (auto& slot : scanner.slots) {
      assert(slot.received == image && slot.begins == 1 && slot.ends == 1);
      for (const auto& packet : slot.packets) assert(packet.first / 4096 == (packet.first + packet.second - 1) / 4096);
    }
  }
  Store store; store.image(); FakeTransport scanner;
  scanner.slots[0].present = false; scanner.slots[1].capable = false;
  assert(run(store, scanner));
  assert(scanner.slots[0].begins == 0 && scanner.slots[1].begins == 0 && scanner.slots[2].ends == 1);
}
static void failures() {
  for (int failure = 0; failure < 13; ++failure) {
    Store store; store.image(); FakeTransport scanner;
    auto config = configuration();
    if (failure == 0) store.files.clear();
    if (failure == 1) store.image(0);
    if (failure == 2) store.fail_open = true;
    if (failure == 3) store.fail_seek = true;
    if (failure == 4) store.fail_read_on_open = 1;
    if (failure == 5) store.oversized = true;
    if (failure == 6) scanner.begin_ok = false;
    if (failure == 7) scanner.bus_error = true;
    if (failure == 8) for (auto& slot : scanner.slots) slot.capable = false;
    if (failure == 9) for (auto& slot : scanner.slots) slot.reject_begin = true;
    if (failure == 10) for (auto& slot : scanner.slots) slot.fail_data = true;
    if (failure == 11) for (auto& slot : scanner.slots) slot.reject_end = true;
    if (failure == 12) for (auto& slot : scanner.slots) slot.delay = 1000;
    assert(!run(store, scanner, config));
    assert(!store.exists("/scanner.applied.bin"));
    if (failure != 0) assert(store.exists("/scanner.bin"));
    if (failure == 10) for (auto& slot : scanner.slots) assert(slot.aborts == 1);
  }
  for (int failure = 0; failure < 4; ++failure) {
    Store store; store.image(); FakeTransport scanner;
    if (failure == 0) scanner.slots[1].fail_data = true;
    if (failure == 1) store.fail_reopen = 3;
    if (failure == 2) store.fail_read_on_open = 3;
    if (failure == 3) store.fail_rename = true;
    assert(run(store, scanner)); // At least one slot updated; staged image retained.
    assert(store.exists("/scanner.bin") && !store.exists("/scanner.applied.bin"));
  }
  Store store; store.image(); FakeTransport scanner;
  store.read_chunk = 7; // Hashing permits short reads; packet reads must be exact.
  assert(!run(store, scanner));
  for (auto& slot : scanner.slots) assert(slot.aborts == 1);
}
static void configurationAndPaths() {
  for (unsigned invalid = 0; invalid < 7; ++invalid) {
    Store store; store.image(); FakeTransport scanner; auto config = configuration();
    if (invalid == 0) config.enabled = false;
    if (invalid == 1) config.packet_retries = 0;
    if (invalid == 2) config.hash_chunk_bytes = 0;
    if (invalid == 3) config.response_pulls = 0;
    if (invalid == 4) config.flash_sector_bytes = 47;
    if (invalid == 5) scanner.count = 0;
    if (invalid == 6) config.image_path = nullptr;
    assert(!run(store, scanner, config)); assert(store.opens == 0 && scanner.frames == 0);
  }
  Store store; store.image(); FakeTransport scanner;
  store.files["/firmware"] = std::move(store.files["/scanner.bin"]); store.files.erase("/scanner.bin");
  store.files["/applied"] = {1};
  auto config = configuration(); config.image_path = "/firmware"; config.applied_path = "/applied";
  assert(run(store, scanner, config)); assert(store.exists("/applied_01"));
}
static void posixStorage() {
  char root[] = "/tmp/shuriken-update-store-XXXXXX"; assert(mkdtemp(root));
  hal::PosixUpdateStorage store(root);
  { std::ofstream file(std::string(root) + "/scanner.bin", std::ios::binary); file << "abcdef"; }
  auto file = store.open("/scanner.bin"); uint8_t bytes[3];
  assert(file && file->size() == 6 && file->read(bytes, 3) == 3 && bytes[0] == 'a');
  assert(file->seekSet(4) && file->read(bytes, 3) == 2 && bytes[0] == 'e'); file->close(); file.reset();
  assert(!store.open("/missing.bin") && !store.open("relative"));
  { std::ofstream existing(std::string(root) + "/scanner.applied.bin"); existing << "keep"; }
  assert(!store.rename("/scanner.bin", "/scanner.applied.bin") && store.exists("/scanner.bin"));
  assert(store.rename("/scanner.bin", "/scanner.applied_01.bin") && !store.exists("/scanner.bin"));
  std::filesystem::remove_all(root);
}
int main() {
  successAndRetries(); failures(); configurationAndPaths(); posixStorage();
  std::puts("Shared scanner updater: protocol/CRC/hash/retries/sectors/failures/markers/storage passed");
}
