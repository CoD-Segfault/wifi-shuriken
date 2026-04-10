#include "controller_update_runtime.h"

#include <LittleFS.h>
#include <Updater.h>
#include <pico_base/pico/ota_command.h>

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "Configuration.h"

namespace {

static constexpr char OTA_STAGED_FIRMWARE_PATH[] = "firmware.bin";
static constexpr char OTA_COMMAND_PATH[] = "otacommand.bin";
static constexpr char UPDATE_MARKER_PATH[] = "controller.pending";
static constexpr uint32_t UPDATE_MARKER_MAGIC = 0x55504454UL;  // "UPDT"
static constexpr uint16_t APPLIED_PATH_SUFFIX_LIMIT = 100;
static constexpr uint8_t UPDATE_LED_BLUE = 96;

struct UpdateMarker {
  uint32_t magic;
  uint32_t source_size;
};

void showUpdateLed(Adafruit_NeoPixel& pixels) {
  pixels.setPixelColor(0, pixels.Color(0, 0, UPDATE_LED_BLUE));
  pixels.show();
}

bool beginLittleFs(Stream& serial) {
  if (LittleFS.begin()) {
    return true;
  }
  serial.println("LittleFS mount failed; SD update support unavailable this boot");
  return false;
}

bool readUpdateMarker(UpdateMarker& marker) {
  memset(&marker, 0, sizeof(marker));

  File marker_file = LittleFS.open(UPDATE_MARKER_PATH, "r");
  if (!marker_file) {
    return false;
  }

  const size_t bytes_read = marker_file.read(reinterpret_cast<uint8_t*>(&marker),
                                             sizeof(marker));
  marker_file.close();
  return bytes_read == sizeof(marker) && marker.magic == UPDATE_MARKER_MAGIC;
}

bool writeUpdateMarker(uint32_t source_size, Stream& serial) {
  UpdateMarker marker = {};
  marker.magic = UPDATE_MARKER_MAGIC;
  marker.source_size = source_size;

  File marker_file = LittleFS.open(UPDATE_MARKER_PATH, "w");
  if (!marker_file) {
    serial.println("Update marker open failed");
    return false;
  }

  const size_t bytes_written =
      marker_file.write(reinterpret_cast<const uint8_t*>(&marker), sizeof(marker));
  marker_file.flush();
  marker_file.close();

  if (bytes_written != sizeof(marker)) {
    serial.println("Update marker write failed");
    return false;
  }
  return true;
}

void removeInternalUpdateArtifact(const char* path, Stream& serial) {
  if (LittleFS.exists(path) && !LittleFS.remove(path)) {
    serial.print("Unable to remove internal update artifact: ");
    serial.println(path);
  }
}

void clearInternalUpdateArtifacts(Stream& serial) {
  removeInternalUpdateArtifact(UPDATE_MARKER_PATH, serial);
  removeInternalUpdateArtifact(OTA_COMMAND_PATH, serial);
  removeInternalUpdateArtifact(OTA_STAGED_FIRMWARE_PATH, serial);
}

uint32_t computeOtaCommandCrc(const OTACmdPage& command_page) {
  uint32_t crc = 0xffffffff;
  const uint8_t* data = reinterpret_cast<const uint8_t*>(&command_page);
  for (size_t i = 0; i < offsetof(OTACmdPage, crc32); i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 1U) {
        crc = (crc >> 1) ^ 0xedb88320UL;
      } else {
        crc >>= 1;
      }
    }
  }
  return ~crc;
}

bool otaCommandLooksValid() {
  File command_file = LittleFS.open(OTA_COMMAND_PATH, "r");
  if (!command_file) {
    return false;
  }

  OTACmdPage command_page = {};
  const size_t bytes_read =
      command_file.read(reinterpret_cast<uint8_t*>(&command_page), sizeof(command_page));
  command_file.close();

  if (bytes_read != sizeof(command_page)) {
    return false;
  }
  if (memcmp(command_page.sign, "Pico OTA", sizeof(command_page.sign)) != 0) {
    return false;
  }
  if (command_page.count == 0 || command_page.count > 8) {
    return false;
  }
  return computeOtaCommandCrc(command_page) == command_page.crc32;
}

bool buildAppliedPath(SdFat& sd, char* out, size_t out_len) {
  const char* const applied_base = CONTROLLER_SD_UPDATE_APPLIED_PATH;
  if (!sd.exists(applied_base)) {
    snprintf(out, out_len, "%s", applied_base);
    return true;
  }

  const char* const slash = strrchr(applied_base, '/');
  const char* const dot = strrchr(applied_base, '.');
  const bool has_extension = (dot != nullptr) && (slash == nullptr || dot > slash);

  for (uint16_t suffix = 1; suffix < APPLIED_PATH_SUFFIX_LIMIT; suffix++) {
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

bool finalizeAppliedUpdateIfReady(SdFat& sd,
                                  bool sd_ready,
                                  const UpdateMarker& marker,
                                  Stream& serial) {
  const bool ota_command_pending = otaCommandLooksValid();
  if (ota_command_pending) {
    serial.println("Firmware update is staged and waiting for the OTA bootloader");
    return false;
  }

  if (!sd_ready) {
    serial.println("Firmware update applied; waiting for SD to come online before renaming the update file");
    return false;
  }

  if (!sd.exists(CONTROLLER_SD_UPDATE_PATH)) {
    serial.println("Firmware update finished and controller.bin is gone; clearing stale update marker");
    clearInternalUpdateArtifacts(serial);
    return true;
  }

  FsFile source = sd.open(CONTROLLER_SD_UPDATE_PATH, O_RDONLY);
  if (!source) {
    serial.println("Firmware update applied, but controller.bin could not be opened for rename");
    return false;
  }

  const uint32_t source_size = static_cast<uint32_t>(source.size());
  source.close();

  if (source_size != marker.source_size) {
    serial.print("Firmware update finished, but controller.bin size changed (expected ");
    serial.print(marker.source_size);
    serial.print(", found ");
    serial.print(source_size);
    serial.println("); clearing stale update marker");
    clearInternalUpdateArtifacts(serial);
    return true;
  }

  char applied_path[48] = {};
  if (!buildAppliedPath(sd, applied_path, sizeof(applied_path))) {
    serial.println("Firmware update applied, but no free applied filename was available");
    return false;
  }

  if (!sd.rename(CONTROLLER_SD_UPDATE_PATH, applied_path)) {
    serial.print("Firmware update applied, but rename failed: ");
    serial.println(applied_path);
    return false;
  }

  removeInternalUpdateArtifact(UPDATE_MARKER_PATH, serial);
  removeInternalUpdateArtifact(OTA_STAGED_FIRMWARE_PATH, serial);

  serial.print("Firmware update applied successfully; renamed SD image to ");
  serial.println(applied_path);
  return true;
}

bool stageUpdateFromSd(SdFat& sd, Stream& serial, Adafruit_NeoPixel& pixels) {
  FsFile source = sd.open(CONTROLLER_SD_UPDATE_PATH, O_RDONLY);
  if (!source) {
    serial.println("controller.bin exists but could not be opened");
    return false;
  }

  const uint32_t source_size = static_cast<uint32_t>(source.size());
  if (source_size == 0) {
    serial.println("controller.bin is empty; skipping firmware update");
    source.close();
    return false;
  }

  if (otaCommandLooksValid()) {
    serial.println("OTA command file is already present; refusing to stage a second controller update");
    source.close();
    return false;
  }

  serial.print("Staging controller firmware update from SD: ");
  serial.print(source_size);
  serial.println(" bytes");
  showUpdateLed(pixels);

  removeInternalUpdateArtifact(OTA_STAGED_FIRMWARE_PATH, serial);
  removeInternalUpdateArtifact(UPDATE_MARKER_PATH, serial);

  if (!Update.begin(source_size, U_FLASH)) {
    serial.print("Update.begin failed: ");
    Update.printError(serial);
    source.close();
    clearInternalUpdateArtifacts(serial);
    return false;
  }

  uint8_t buffer[CONTROLLER_SD_UPDATE_CHUNK_BYTES] = {};
  uint32_t total_written = 0;
  bool write_failed = false;

  while (true) {
    const int bytes_read = source.read(buffer, sizeof(buffer));
    if (bytes_read < 0) {
      serial.println("controller.bin read failed during staging");
      write_failed = true;
      break;
    }
    if (bytes_read == 0) {
      break;
    }

    const size_t bytes_written = Update.write(buffer, static_cast<size_t>(bytes_read));
    if (bytes_written != static_cast<size_t>(bytes_read)) {
      serial.print("Firmware staging write failed after ");
      serial.print(total_written);
      serial.println(" bytes");
      Update.printError(serial);
      write_failed = true;
      break;
    }

    total_written += static_cast<uint32_t>(bytes_written);
    yield();
  }

  source.close();

  if (write_failed || total_written != source_size) {
    serial.print("Firmware staging aborted at ");
    serial.print(total_written);
    serial.print(" of ");
    serial.print(source_size);
    serial.println(" bytes");
    clearInternalUpdateArtifacts(serial);
    return false;
  }

  if (!Update.end()) {
    serial.print("Update.end failed: ");
    Update.printError(serial);
    clearInternalUpdateArtifacts(serial);
    return false;
  }

  if (!writeUpdateMarker(source_size, serial)) {
    clearInternalUpdateArtifacts(serial);
    return false;
  }

  serial.println("Controller firmware staged; rebooting into OTA bootloader");
  serial.flush();
  delay(100);
  rp2040.reboot();
  while (true) {
    delay(1);
  }
}

}  // namespace

bool controllerUpdateRuntimeHandleBoot(SdFat& sd,
                                       bool sd_ready,
                                       Stream& serial,
                                       Adafruit_NeoPixel& pixels) {
#if !CONTROLLER_SD_UPDATE_ENABLED
  (void)sd;
  (void)sd_ready;
  (void)serial;
  (void)pixels;
  return false;
#else
  if (!beginLittleFs(serial)) {
    return false;
  }

  UpdateMarker marker = {};
  if (readUpdateMarker(marker)) {
    showUpdateLed(pixels);
    if (!finalizeAppliedUpdateIfReady(sd, sd_ready, marker, serial)) {
      return false;
    }
  }

  if (!sd_ready) {
    return false;
  }

  if (!sd.exists(CONTROLLER_SD_UPDATE_PATH)) {
    return false;
  }

  return stageUpdateFromSd(sd, serial, pixels);
#endif
}
