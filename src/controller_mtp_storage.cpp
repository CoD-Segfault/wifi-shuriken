#include "controller_mtp_storage.h"

#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <hardware/sync.h>
#include <pico/mutex.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>

#include "Configuration.h"
#include "class/mtp/mtp_device.h"
#include "controller_mtp_transport.h"

extern mutex_t __usb_mutex;

namespace {

constexpr uint32_t STORAGE_ID = 0x00010001u;
constexpr size_t MAX_NAME_LEN = 200;

struct MtpObject {
  char name[MAX_NAME_LEN];
  uint64_t size;
  uint16_t format;
  bool directory;
  char created[16];
  char modified[16];
};

class MtpUsbInterface : public Adafruit_USBD_Interface {
 public:
  bool begin() {
    // Must be exactly "MTP": libmtp identifies an MTP responder by matching this
    // interface string descriptor, and gvfs/file managers go through libmtp. Any
    // other value (it used to say "SD") leaves the device classified as a plain
    // PTP camera, which gphoto2 then fails to talk to. The volume name the host
    // actually displays is the storage description sent in GetStorageInfo, not
    // this string, so naming it "MTP" costs nothing user-visible.
    setStringDescriptor("MTP");
    return TinyUSBDevice.addInterface(*this);
  }

  uint16_t getInterfaceDescriptor(uint8_t, uint8_t* buf, uint16_t bufsize) override {
    if (buf == nullptr) {
      return TUD_MTP_DESC_LEN;
    }
    if (bufsize < TUD_MTP_DESC_LEN) {
      return 0;
    }

    const uint8_t itf = TinyUSBDevice.allocInterface(1);
    const uint8_t ep_event = TinyUSBDevice.allocEndpoint(TUSB_DIR_IN);
    const uint8_t ep_out = TinyUSBDevice.allocEndpoint(TUSB_DIR_OUT);
    const uint8_t ep_in = TinyUSBDevice.allocEndpoint(TUSB_DIR_IN);
    const uint8_t desc[] = {
        TUD_MTP_DESCRIPTOR(itf, _strid, ep_event, 64, 1, ep_out, ep_in, 64)};
    memcpy(buf, desc, sizeof(desc));
    return sizeof(desc);
  }
};

MtpUsbInterface mtp_interface;
SdFat* mtp_sd = nullptr;
FsFile* mtp_active_log = nullptr;
mutex_t sd_mutex;
bool mutex_initialized = false;
volatile bool storage_ready = false;
bool session_open = false;
bool object_transfer_active = false;
volatile bool transfer_release_requested = false;
volatile bool session_reset_requested = false;
uint32_t object_count = 0;
uint32_t* object_handles = nullptr;
uint32_t object_handle_capacity = 0;
MtpObject transfer_object = {};
FsFile transfer_file;

enum class PendingCallback : uint8_t {
  NONE,
  COMMAND,
  DATA,
  DATA_COMPLETE,
};

volatile PendingCallback pending_callback = PendingCallback::NONE;
tud_mtp_cb_data_t pending_data = {};
uint32_t pending_generation = 0;

class SdTryLock {
 public:
  SdTryLock() : locked_(mutex_initialized && mutex_try_enter(&sd_mutex, nullptr)) {}
  ~SdTryLock() {
    if (locked_) {
      mutex_exit(&sd_mutex);
    }
  }
  bool locked() const { return locked_; }

 private:
  bool locked_;
};

using StorageInfo = MTP_STORAGE_INFO_STRUCT(3, 9);

StorageInfo storage_info = {
    .storage_type = MTP_STORAGE_TYPE_REMOVABLE_RAM,
    .filesystem_type = MTP_FILESYSTEM_TYPE_GENERIC_HIERARCHICAL,
    .access_capability = MTP_ACCESS_CAPABILITY_READ_ONLY_WITHOUT_OBJECT_DELETION,
    .max_capacity_in_bytes = 0,
    .free_space_in_bytes = 0,
    .free_space_in_objects = 0,
    .storage_description = {.count = 3, .utf16 = {'S', 'D', 0}},
    .volume_identifier = {.count = 9, .utf16 = {'S', 'h', 'u', 'r', 'i', 'k', 'e', 'n', 0}},
};

void flushActiveLog() {
  if (mtp_active_log != nullptr && *mtp_active_log) {
    mtp_active_log->flush();
  }
}

void closeObjectTransfer() {
  if (object_transfer_active) {
    transfer_file.close();
    object_transfer_active = false;
  }
}

void requestTransferRelease(bool reset_session) {
  const uint32_t irq_state = save_and_disable_interrupts();
  transfer_release_requested = true;
  pending_callback = PendingCallback::NONE;
  if (reset_session) {
    session_reset_requested = true;
  }
  restore_interrupts(irq_state);
}

void serviceTransferReleaseRequests() {
  const uint32_t irq_state = save_and_disable_interrupts();
  const bool release_transfer = transfer_release_requested;
  const bool reset_session = session_reset_requested;
  transfer_release_requested = false;
  session_reset_requested = false;
  restore_interrupts(irq_state);

  if (release_transfer) {
    closeObjectTransfer();
  }
  if (reset_session) {
    session_open = false;
    object_count = 0;
  }
}

uint16_t objectFormat(const char* name, bool directory) {
  if (directory) {
    return MTP_OBJ_FORMAT_ASSOCIATION;
  }
  const char* extension = strrchr(name, '.');
  if (extension != nullptr &&
      (strcasecmp(extension, ".csv") == 0 || strcasecmp(extension, ".txt") == 0)) {
    return MTP_OBJ_FORMAT_TEXT;
  }
  return MTP_OBJ_FORMAT_UNDEFINED;
}

bool refreshObjectHandles(uint32_t format) {
  uint32_t count = 0;
  flushActiveLog();

  FsFile root = mtp_sd->open("/", O_RDONLY);
  if (!root) {
    object_count = 0;
    return false;
  }
  FsFile entry;
  char name[MAX_NAME_LEN];
  while (entry.openNext(&root, O_RDONLY)) {
    if (!entry.isDir() && entry.getName(name, sizeof(name)) > 0 && name[0] != '\0' &&
        (format == 0 || format == objectFormat(name, false)) && count < (UINT32_MAX - 16) / 4) {
      count++;
    }
    entry.close();
  }
  root.close();

  if (count > object_handle_capacity) {
    void* allocation = realloc(object_handles, count * sizeof(uint32_t));
    if (allocation == nullptr) {
      object_count = 0;
      return false;
    }
    object_handles = static_cast<uint32_t*>(allocation);
    object_handle_capacity = count;
  }

  object_count = 0;
  root = mtp_sd->open("/", O_RDONLY);
  if (!root) {
    return false;
  }
  while (object_count < count && entry.openNext(&root, O_RDONLY)) {
    if (!entry.isDir() && entry.getName(name, sizeof(name)) > 0 && name[0] != '\0' &&
        (format == 0 || format == objectFormat(name, false))) {
      object_handles[object_count++] = entry.dirIndex() + 1;
    }
    entry.close();
  }
  root.close();
  return true;
}

void formatTimestamp(char* destination, uint16_t date, uint16_t time) {
  unsigned year = 1980 + (date >> 9), month = (date >> 5) & 15, day = date & 31;
  unsigned hour = time >> 11, minute = (time >> 5) & 63, second = (time & 31) * 2;
  if (month < 1 || month > 12 || day < 1 || day > 31 || hour > 23 || minute > 59 || second > 59) {
    year = 1980; month = day = 1; hour = minute = second = 0;
  }
  snprintf(destination, 16, "%04u%02u%02uT%02u%02u%02u", year, month, day, hour, minute, second);
}

bool loadObject(uint32_t handle, MtpObject& object) {
  if (handle == 0) {
    return false;
  }
  FsFile root = mtp_sd->open("/", O_RDONLY);
  if (!root) {
    return false;
  }
  FsFile entry;
  const bool opened = entry.open(&root, handle - 1, O_RDONLY);
  root.close();
  if (!opened) {
    return false;
  }
  memset(&object, 0, sizeof(object));
  if (entry.getName(object.name, sizeof(object.name)) <= 0 || object.name[0] == '\0') {
    entry.close();
    return false;
  }
  object.directory = entry.isDir();
  if (object.directory) {
    entry.close();
    return false;
  }
  const uint64_t size = entry.fileSize();
  object.size = size;
  uint16_t date = 0, time = 0;
  entry.getCreateDateTime(&date, &time);
  formatTimestamp(object.created, date, time);
  date = time = 0;
  entry.getModifyDateTime(&date, &time);
  formatTimestamp(object.modified, date, time);
  object.format = objectFormat(object.name, object.directory);
  entry.close();
  return true;
}

bool openTransferObject(uint32_t handle) {
  if (handle == 0) {
    return false;
  }
  FsFile root = mtp_sd->open("/", O_RDONLY);
  if (!root) {
    return false;
  }
  const bool opened = transfer_file.open(&root, handle - 1, O_RDONLY);
  root.close();
  if (!opened) {
    return false;
  }
  memset(&transfer_object, 0, sizeof(transfer_object));
  if (transfer_file.getName(transfer_object.name, sizeof(transfer_object.name)) <= 0 ||
      transfer_object.name[0] == '\0') {
    transfer_file.close();
    return false;
  }
  transfer_object.directory = transfer_file.isDir();
  const uint64_t size = transfer_file.fileSize();
  transfer_object.size = size;
  transfer_object.format = objectFormat(transfer_object.name, transfer_object.directory);
  if (transfer_object.directory) {
    transfer_file.close();
    return false;
  }
  return true;
}

bool readTransferData(uint8_t* destination, uint32_t length) {
  if (!transfer_file) {
    return false;
  }
  const int bytes_read = transfer_file.read(destination, length);
  return bytes_read == static_cast<int>(length);
}

void fillObjectHandles(uint8_t* destination, uint32_t offset, uint32_t length) {
  for (uint32_t i = 0; i < length; ++i) {
    const uint32_t position = offset + i;
    uint32_t value = object_count;
    if (position >= sizeof(uint32_t)) {
      const uint32_t index = (position - sizeof(uint32_t)) / sizeof(uint32_t);
      if (index < object_count) {
        value = object_handles[index];
      } else {
        value = 0;
      }
    }
    destination[i] = static_cast<uint8_t>(
        value >> ((position % sizeof(uint32_t)) * 8));
  }
}

bool validStorageId(uint32_t id) {
  return id == STORAGE_ID || id == UINT32_MAX;
}

uint32_t remainingBytes(uint32_t total, uint32_t transferred) {
  return transferred < total ? total - transferred : 0;
}

bool objectHandlePayloadSize(uint32_t& total) {
  if (object_count > (UINT32_MAX - sizeof(uint32_t)) / sizeof(uint32_t)) {
    return false;
  }
  total = sizeof(uint32_t) + object_count * sizeof(uint32_t);
  return true;
}

void sendEmptyObjectHandleArray(tud_mtp_cb_data_t* cb) {
  mtp_container_add_uint32(&cb->io_container, 0);
  tud_mtp_data_send(&cb->io_container);
}

int32_t sendDeviceInfo(tud_mtp_cb_data_t* cb) {
  mtp_container_add_cstring(&cb->io_container, "Shuriken");
  mtp_container_add_cstring(&cb->io_container, "SD");
  mtp_container_add_cstring(&cb->io_container, APP_VERSION);
  mtp_container_add_cstring(&cb->io_container, "WiFi-Shuriken");
  return tud_mtp_data_send(&cb->io_container) ? 0 : MTP_RESP_DEVICE_BUSY;
}

int32_t openCloseSession(tud_mtp_cb_data_t* cb) {
  if (cb->command_container->header.code == MTP_OP_OPEN_SESSION) {
    if (session_open) {
      return MTP_RESP_SESSION_ALREADY_OPEN;
    }
    if (cb->command_container->params[0] == 0) return MTP_RESP_INVALID_PARAMETER;
    session_open = true;
    controllerMtpTransportSetSession(cb->command_container->params[0]);
  } else {
    if (!session_open) {
      return MTP_RESP_SESSION_NOT_OPEN;
    }
    session_open = false;
    controllerMtpTransportSetSession(0);
    object_count = 0;
    closeObjectTransfer();
  }
  return MTP_RESP_OK;
}

int32_t sendStorageIds(tud_mtp_cb_data_t* cb) {
  const uint32_t id = STORAGE_ID;
  mtp_container_add_auint32(&cb->io_container, 1, &id);
  tud_mtp_data_send(&cb->io_container);
  return 0;
}

int32_t sendStorageInfo(tud_mtp_cb_data_t* cb) {
  if (cb->command_container->params[0] != STORAGE_ID) {
    return MTP_RESP_INVALID_STORAGE_ID;
  }
  if (!storage_ready) {
    storage_info.max_capacity_in_bytes = 0;
    storage_info.free_space_in_bytes = 0;
    mtp_container_add_raw(&cb->io_container, &storage_info, sizeof(storage_info));
    tud_mtp_data_send(&cb->io_container);
    return 0;
  }
  SdTryLock lock;
  if (!lock.locked()) {
    return MTP_RESP_DEVICE_BUSY;
  }
  storage_info.max_capacity_in_bytes =
      static_cast<uint64_t>(mtp_sd->card()->sectorCount()) * 512u;
  // Avoid a potentially long FAT scan in the foreground MTP service path.
  // This view is read-only: no space is available for host uploads. Avoid
  // reporting the unknown-size sentinel as a literal 16 EB in some clients.
  storage_info.free_space_in_bytes = 0;
  mtp_container_add_raw(&cb->io_container, &storage_info, sizeof(storage_info));
  tud_mtp_data_send(&cb->io_container);
  return 0;
}

int32_t sendDeviceProperty(tud_mtp_cb_data_t* cb) {
  const uint16_t property = static_cast<uint16_t>(cb->command_container->params[0]);
  if (property != MTP_DEV_PROP_DEVICE_FRIENDLY_NAME) {
    return MTP_RESP_PARAMETER_NOT_SUPPORTED;
  }
  if (cb->command_container->header.code == MTP_OP_GET_DEVICE_PROP_DESC) {
    const mtp_device_prop_desc_header_t header = {
        MTP_DEV_PROP_DEVICE_FRIENDLY_NAME, MTP_DATA_TYPE_STR, MTP_MODE_GET};
    mtp_container_add_raw(&cb->io_container, &header, sizeof(header));
    mtp_container_add_cstring(&cb->io_container, "Shuriken");
    mtp_container_add_cstring(&cb->io_container, "Shuriken");
    mtp_container_add_uint8(&cb->io_container, 0);
  } else {
    mtp_container_add_cstring(&cb->io_container, "Shuriken");
  }
  tud_mtp_data_send(&cb->io_container);
  return 0;
}

bool supportedFormat(uint32_t format);

int32_t sendObjectHandles(tud_mtp_cb_data_t* cb) {
  const mtp_container_command_t* command = cb->command_container;
  if (!validStorageId(command->params[0])) {
    return MTP_RESP_INVALID_STORAGE_ID;
  }
  if (!supportedFormat(command->params[1])) return MTP_RESP_INVALID_OBJECT_FORMAT_CODE;
  const uint32_t parent = command->params[2];
  if (parent != 0 && parent != UINT32_MAX) {
    return MTP_RESP_INVALID_PARENT_OBJECT;
  }
  if (!storage_ready) {
    sendEmptyObjectHandleArray(cb);
    return 0;
  }
  uint32_t transferred = 0;
  if (cb->phase == MTP_PHASE_COMMAND) {
    SdTryLock lock;
    if (!lock.locked()) {
      return MTP_RESP_DEVICE_BUSY;
    }
    if (!refreshObjectHandles(command->params[1])) {
      return MTP_RESP_GENERAL_ERROR;
    }
    uint32_t total = 0;
    if (!objectHandlePayloadSize(total)) {
      object_count = 0;
      return MTP_RESP_GENERAL_ERROR;
    }
    cb->io_container.header->len += total;
  } else if (cb->phase == MTP_PHASE_DATA) {
    transferred = cb->total_xferred_bytes - sizeof(mtp_container_header_t);
  }
  uint32_t total = 0;
  if (!objectHandlePayloadSize(total)) {
    return MTP_RESP_GENERAL_ERROR;
  }
  const uint32_t chunk = std::min(remainingBytes(total, transferred),
                                  cb->io_container.payload_bytes);
  if (chunk > 0) {
    fillObjectHandles(cb->io_container.payload, transferred, chunk);
  }
  tud_mtp_data_send(&cb->io_container);
  return 0;
}

int32_t sendObjectInfo(tud_mtp_cb_data_t* cb) {
  if (!storage_ready) return MTP_RESP_STORE_NOT_AVAILABLE;
  SdTryLock lock;
  if (!lock.locked()) {
    return MTP_RESP_DEVICE_BUSY;
  }
  flushActiveLog();
  MtpObject object = {};
  if (!loadObject(cb->command_container->params[0], object)) {
    return MTP_RESP_INVALID_OBJECT_HANDLE;
  }
  const mtp_object_info_header_t info = {
      .storage_id = STORAGE_ID,
      .object_format = object.format,
      .protection_status = MTP_PROTECTION_STATUS_READ_ONLY,
      .object_compressed_size = static_cast<uint32_t>(std::min<uint64_t>(object.size, UINT32_MAX)),
      .thumb_format = MTP_OBJ_FORMAT_UNDEFINED,
      .thumb_compressed_size = 0,
      .thumb_pix_width = 0,
      .thumb_pix_height = 0,
      .image_pix_width = 0,
      .image_pix_height = 0,
      .image_bit_depth = 0,
      .parent_object = 0,
      .association_type = object.directory ? MTP_ASSOCIATION_GENERIC_FOLDER
                                           : MTP_ASSOCIATION_UNDEFINED,
      .association_desc = 0,
      .sequence_number = 0,
  };
  mtp_container_add_raw(&cb->io_container, &info, sizeof(info));
  mtp_container_add_cstring(&cb->io_container, object.name);
  mtp_container_add_cstring(&cb->io_container, object.created);
  mtp_container_add_cstring(&cb->io_container, object.modified);
  mtp_container_add_cstring(&cb->io_container, "");
  tud_mtp_data_send(&cb->io_container);
  return 0;
}

int32_t sendObject(tud_mtp_cb_data_t* cb, bool partial) {
  if (!storage_ready) return MTP_RESP_STORE_NOT_AVAILABLE;
  SdTryLock lock;
  if (!lock.locked()) return MTP_RESP_DEVICE_BUSY;
  const mtp_container_command_t* command = cb->command_container;
  uint32_t transferred = 0;
  if (cb->phase == MTP_PHASE_COMMAND) {
    if (object_transfer_active) {
      return MTP_RESP_DEVICE_BUSY;
    }
    object_transfer_active = true;
    flushActiveLog();
    if (!openTransferObject(command->params[0])) {
      closeObjectTransfer();
      return MTP_RESP_INVALID_OBJECT_HANDLE;
    }
  } else if (cb->phase == MTP_PHASE_DATA) {
    if (!object_transfer_active) {
      return MTP_RESP_GENERAL_ERROR;
    }
    transferred = cb->total_xferred_bytes - sizeof(mtp_container_header_t);
  }
  const uint32_t requested_offset = partial ? command->params[1] : 0;
  const uint64_t available = requested_offset >= transfer_object.size
                                 ? 0
                                 : transfer_object.size - requested_offset;
  const uint64_t requested = partial ? std::min<uint64_t>(available, command->params[2]) : available;
  if (requested > UINT32_MAX - sizeof(mtp_container_header_t)) {
    closeObjectTransfer();
    return MTP_RESP_OBJECT_TOO_LARGE;
  }
  const uint32_t total = static_cast<uint32_t>(requested);
  const uint32_t chunk = std::min(remainingBytes(total, transferred),
                                  cb->io_container.payload_bytes);
  if (cb->phase == MTP_PHASE_COMMAND) {
    if (requested_offset < transfer_object.size && !transfer_file.seekSet(requested_offset)) {
      closeObjectTransfer();
      return MTP_RESP_GENERAL_ERROR;
    }
    cb->io_container.header->len += total;
  }
  // The logger may append between chunks. Synchronize its cache before reading
  // through a second file handle; the transfer still ends at its initial size.
  flushActiveLog();
  if (chunk > 0) {
    if (!readTransferData(cb->io_container.payload, chunk)) {
      closeObjectTransfer();
      return MTP_RESP_GENERAL_ERROR;
    }
  }
  if (!tud_mtp_data_send(&cb->io_container)) {
    closeObjectTransfer();
    return 0;
  }
  return 0;
}

using Handler = int32_t (*)(tud_mtp_cb_data_t*);

constexpr uint16_t object_properties[] = {
    MTP_OBJ_PROP_STORAGE_ID, MTP_OBJ_PROP_OBJECT_FORMAT, MTP_OBJ_PROP_PROTECTION_STATUS,
    MTP_OBJ_PROP_OBJECT_SIZE, MTP_OBJ_PROP_OBJECT_FILE_NAME, MTP_OBJ_PROP_DATE_CREATED,
    MTP_OBJ_PROP_DATE_MODIFIED, MTP_OBJ_PROP_PARENT_OBJECT};

uint16_t propertyType(uint32_t property) {
  switch (property) {
    case MTP_OBJ_PROP_STORAGE_ID: case MTP_OBJ_PROP_PARENT_OBJECT: return MTP_DATA_TYPE_UINT32;
    case MTP_OBJ_PROP_OBJECT_FORMAT: case MTP_OBJ_PROP_PROTECTION_STATUS: return MTP_DATA_TYPE_UINT16;
    case MTP_OBJ_PROP_OBJECT_SIZE: return MTP_DATA_TYPE_UINT64;
    case MTP_OBJ_PROP_OBJECT_FILE_NAME: case MTP_OBJ_PROP_DATE_CREATED:
    case MTP_OBJ_PROP_DATE_MODIFIED: return MTP_DATA_TYPE_STR;
    default: return 0;
  }
}

bool supportedFormat(uint32_t format) {
  return format == 0 || format == MTP_OBJ_FORMAT_UNDEFINED || format == MTP_OBJ_FORMAT_TEXT;
}

int32_t sendObjectProperty(tud_mtp_cb_data_t* cb) {
  const auto* command = cb->command_container;
  auto* container = &cb->io_container;
  if (command->header.code == MTP_OP_GET_OBJECT_PROPS_SUPPORTED) {
    if (!supportedFormat(command->params[0])) return MTP_RESP_INVALID_OBJECT_FORMAT_CODE;
    mtp_container_add_auint16(container, sizeof(object_properties) / sizeof(object_properties[0]), object_properties);
  } else if (command->header.code == MTP_OP_GET_OBJECT_PROP_DESC) {
    const uint32_t property = command->params[0];
    const uint16_t type = propertyType(property);
    if (!type) return MTP_RESP_INVALID_OBJECT_PROP_CODE;
    if (!supportedFormat(command->params[1])) return MTP_RESP_INVALID_OBJECT_FORMAT_CODE;
    mtp_container_add_uint16(container, static_cast<uint16_t>(property));
    mtp_container_add_uint16(container, type);
    mtp_container_add_uint8(container, 0);  // read-only
    if (type == MTP_DATA_TYPE_STR) mtp_container_add_cstring(container, "");
    else if (type == MTP_DATA_TYPE_UINT64) mtp_container_add_uint64(container, 0);
    else if (type == MTP_DATA_TYPE_UINT32) mtp_container_add_uint32(container, 0);
    else mtp_container_add_uint16(container, 0);
    mtp_container_add_uint32(container, 0);  // group code
    mtp_container_add_uint8(container, 0);   // no form constraint
  } else {
    if (!propertyType(command->params[1])) return MTP_RESP_INVALID_OBJECT_PROP_CODE;
    SdTryLock lock;
    if (!lock.locked()) return MTP_RESP_DEVICE_BUSY;
    if (!storage_ready) return MTP_RESP_STORE_NOT_AVAILABLE;
    flushActiveLog();
    MtpObject object = {};
    if (!loadObject(command->params[0], object)) return MTP_RESP_INVALID_OBJECT_HANDLE;
    switch (command->params[1]) {
      case MTP_OBJ_PROP_STORAGE_ID: mtp_container_add_uint32(container, STORAGE_ID); break;
      case MTP_OBJ_PROP_PARENT_OBJECT: mtp_container_add_uint32(container, 0); break;
      case MTP_OBJ_PROP_OBJECT_FORMAT: mtp_container_add_uint16(container, object.format); break;
      case MTP_OBJ_PROP_PROTECTION_STATUS: mtp_container_add_uint16(container, MTP_PROTECTION_STATUS_READ_ONLY); break;
      case MTP_OBJ_PROP_OBJECT_SIZE: mtp_container_add_uint64(container, object.size); break;
      case MTP_OBJ_PROP_OBJECT_FILE_NAME: mtp_container_add_cstring(container, object.name); break;
      case MTP_OBJ_PROP_DATE_CREATED: mtp_container_add_cstring(container, object.created); break;
      case MTP_OBJ_PROP_DATE_MODIFIED: mtp_container_add_cstring(container, object.modified); break;
    }
  }
  tud_mtp_data_send(container);
  return 0;
}

int32_t sendObjectCount(tud_mtp_cb_data_t* cb) {
  const auto* command = cb->command_container;
  if (!validStorageId(command->params[0])) return MTP_RESP_INVALID_STORAGE_ID;
  if (command->params[2] != 0 && command->params[2] != UINT32_MAX) return MTP_RESP_INVALID_PARENT_OBJECT;
  if (!supportedFormat(command->params[1])) return MTP_RESP_INVALID_OBJECT_FORMAT_CODE;
  SdTryLock lock;
  if (!lock.locked()) return MTP_RESP_DEVICE_BUSY;
  if (!storage_ready) return MTP_RESP_STORE_NOT_AVAILABLE;
  if (!refreshObjectHandles(command->params[1])) return MTP_RESP_GENERAL_ERROR;
  mtp_container_add_uint32(&cb->io_container, object_count);
  cb->io_container.header->code = MTP_RESP_OK;
  tud_mtp_response_send(&cb->io_container);
  return 0;
}

Handler findHandler(uint16_t operation) {
  switch (operation) {
    case MTP_OP_GET_DEVICE_INFO: return sendDeviceInfo;
    case MTP_OP_OPEN_SESSION:
    case MTP_OP_CLOSE_SESSION: return openCloseSession;
    case MTP_OP_GET_STORAGE_IDS: return sendStorageIds;
    case MTP_OP_GET_STORAGE_INFO: return sendStorageInfo;
    case MTP_OP_GET_DEVICE_PROP_DESC:
    case MTP_OP_GET_DEVICE_PROP_VALUE: return sendDeviceProperty;
    case MTP_OP_GET_OBJECT_HANDLES: return sendObjectHandles;
    case MTP_OP_GET_NUM_OBJECTS: return sendObjectCount;
    case MTP_OP_GET_OBJECT_PROPS_SUPPORTED:
    case MTP_OP_GET_OBJECT_PROP_DESC:
    case MTP_OP_GET_OBJECT_PROP_VALUE: return sendObjectProperty;
    case MTP_OP_GET_OBJECT_INFO: return sendObjectInfo;
    case MTP_OP_GET_OBJECT: return [](tud_mtp_cb_data_t* cb) { return sendObject(cb, false); };
    case MTP_OP_GET_PARTIAL_OBJECT: return [](tud_mtp_cb_data_t* cb) { return sendObject(cb, true); };
    default: return nullptr;
  }
}

int32_t dispatch(tud_mtp_cb_data_t* cb) {
  const uint16_t operation = cb->command_container->header.code;
  Handler handler = findHandler(operation);
  const bool requires_session = operation != MTP_OP_GET_DEVICE_INFO && operation != MTP_OP_OPEN_SESSION;
  int32_t response;
  if (requires_session && !session_open) {
    response = MTP_RESP_SESSION_NOT_OPEN;
  } else if (requires_session && cb->command_container->header.transaction_id == 0) {
    response = MTP_RESP_INVALID_TRANSACTION_ID;
  } else {
    response = handler == nullptr ? MTP_RESP_OPERATION_NOT_SUPPORTED : handler(cb);
  }
  if (response > MTP_RESP_UNDEFINED) {
    if (cb->phase != MTP_PHASE_COMMAND) {
      // A data phase cannot be replaced with a headerless response. Abort the
      // pipe so the host can recover, without transmitting stale file bytes.
      controllerMtpTransportAbort(false);
      return response;
    }
    cb->io_container.header->len = sizeof(mtp_container_header_t);
    if (response == MTP_RESP_SESSION_ALREADY_OPEN) {
      mtp_container_add_uint32(&cb->io_container, cb->session_id);
    }
    cb->io_container.header->code = static_cast<uint16_t>(response);
    tud_mtp_response_send(&cb->io_container);
  }
  return response;
}

int32_t completeData(tud_mtp_cb_data_t* cb) {
  closeObjectTransfer();
  mtp_container_info_t* response = &cb->io_container;
  if (cb->command_container->header.code == MTP_OP_GET_PARTIAL_OBJECT) {
    const uint32_t bytes = cb->total_xferred_bytes - sizeof(mtp_container_header_t);
    mtp_container_add_uint32(response, bytes);
  }
  response->header->code = cb->xfer_result == XFER_RESULT_SUCCESS
                              ? MTP_RESP_OK
                              : MTP_RESP_GENERAL_ERROR;
  tud_mtp_response_send(response);
  return 0;
}

int32_t deferCallback(PendingCallback callback, tud_mtp_cb_data_t* cb) {
  if (pending_callback != PendingCallback::NONE) {
    return -1;
  }
  pending_data = *cb;
  pending_generation = controllerMtpTransportGeneration();
  __compiler_memory_barrier();
  pending_callback = callback;
  return 0;
}

}  // namespace

bool controllerMtpStorageBegin(SdFat& sd, FsFile& active_log) {
  if (!mutex_initialized) {
    mutex_init(&sd_mutex);
    mutex_initialized = true;
  }
  mtp_sd = &sd;
  mtp_active_log = &active_log;
  return mtp_interface.begin();
}

void controllerMtpStorageSetReady(bool ready) {
  if (!ready && storage_ready) {
    mutex_enter_blocking(&__usb_mutex);
    controllerMtpTransportAbort(true);
    mutex_exit(&__usb_mutex);
    serviceTransferReleaseRequests();
  }
  storage_ready = ready;
}

void controllerMtpStorageTask() {
  mutex_enter_blocking(&__usb_mutex);
  controllerMtpTransportPoll();
  mutex_exit(&__usb_mutex);
  serviceTransferReleaseRequests();

  const uint32_t irq_state = save_and_disable_interrupts();
  const PendingCallback callback = pending_callback;
  if (callback == PendingCallback::NONE) {
    restore_interrupts(irq_state);
    return;
  }
  tud_mtp_cb_data_t data = pending_data;
  const uint32_t generation = pending_generation;
  pending_callback = PendingCallback::NONE;
  restore_interrupts(irq_state);

  mutex_enter_blocking(&__usb_mutex);
  serviceTransferReleaseRequests();
  if (generation != controllerMtpTransportGeneration()) {
    // USB reset/cancel may have reused the backing buffers after we queued it.
  } else if (callback == PendingCallback::COMMAND || callback == PendingCallback::DATA) {
    dispatch(&data);
  } else if (callback == PendingCallback::DATA_COMPLETE) {
    completeData(&data);
  }
  mutex_exit(&__usb_mutex);
  TinyUSB_Device_Task();
}

void controllerMtpStorageLock() {
  if (mutex_initialized) {
    mutex_enter_blocking(&sd_mutex);
  }
}

bool controllerMtpStorageTryLock() {
  return mutex_initialized && mutex_try_enter(&sd_mutex, nullptr);
}

void controllerMtpStorageUnlock() {
  if (mutex_initialized) {
    mutex_exit(&sd_mutex);
  }
}

extern "C" {

void controllerMtpTransportResetCallback(bool reset_session) {
  requestTransferRelease(reset_session);
}

bool tud_mtp_request_cancel_cb(tud_mtp_request_cb_data_t*) {
  requestTransferRelease(false);
  return true;
}
bool tud_mtp_request_device_reset_cb(tud_mtp_request_cb_data_t*) {
  requestTransferRelease(true);
  return true;
}
int32_t tud_mtp_request_get_extended_event_cb(tud_mtp_request_cb_data_t*) { return -1; }
int32_t tud_mtp_request_get_device_status_cb(tud_mtp_request_cb_data_t* cb) {
  uint16_t* status = reinterpret_cast<uint16_t*>(cb->buf);
  status[0] = 4;
  status[1] = controllerMtpTransportStatus();
  return 4;
}
bool tud_mtp_request_vendor_cb(tud_mtp_request_cb_data_t*) { return false; }

int32_t tud_mtp_command_received_cb(tud_mtp_cb_data_t* cb) {
  return deferCallback(PendingCallback::COMMAND, cb);
}
int32_t tud_mtp_data_xfer_cb(tud_mtp_cb_data_t* cb) {
  return deferCallback(PendingCallback::DATA, cb);
}
int32_t tud_mtp_data_complete_cb(tud_mtp_cb_data_t* cb) {
  return deferCallback(PendingCallback::DATA_COMPLETE, cb);
}
int32_t tud_mtp_response_complete_cb(tud_mtp_cb_data_t*) { return 0; }

void tud_umount_cb() {
  requestTransferRelease(true);
}

}  // extern "C"
