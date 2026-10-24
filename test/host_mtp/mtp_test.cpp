// Exercise production application AND transport, with only USB hardware/SD mocked.
#include "../../src/controller_mtp_storage.cpp"
#include "device/usbd_pvt.h"
#include <cassert>
#include <cstdio>
#include <vector>

mutex_t __usb_mutex;
static uint32_t nowMs;
static bool stalls[256], failSubmit;
static uint8_t* outBuffer;
static std::vector<uint8_t> inPacket;
static bool inPending;
static unsigned statusAcks;
static uint8_t* controlBuffer;

extern "C" {
uint32_t tusb_time_millis_api() { return nowMs; }
tusb_speed_t tud_speed_get() { return TUSB_SPEED_FULL; }
bool usbd_edpt_open(uint8_t, tusb_desc_endpoint_t const*) { return true; }
bool usbd_open_edpt_pair(uint8_t, uint8_t const*, uint8_t, uint8_t, uint8_t* out, uint8_t* in) {
  *out = 1; *in = 0x81; return true;
}
bool usbd_edpt_claim(uint8_t, uint8_t ep) { return !stalls[ep]; }
void usbd_edpt_stall(uint8_t, uint8_t ep) { stalls[ep] = true; if (ep == 0x81) inPending = false; }
void usbd_edpt_clear_stall(uint8_t, uint8_t ep) { stalls[ep] = false; }
bool usbd_edpt_stalled(uint8_t, uint8_t ep) { return stalls[ep]; }
bool usbd_edpt_xfer(uint8_t, uint8_t ep, uint8_t* buffer, uint16_t length, bool) {
  if (failSubmit) return false;
  assert(!stalls[ep]);
  if (ep == 1) outBuffer = buffer;
  else if (ep == 0x81) {
    assert(!inPending);
    inPacket.clear();
    if (length) inPacket.assign(buffer, buffer + length);
    inPending = true;
  }
  return true;
}
bool tud_control_status(uint8_t, tusb_control_request_t const*) { ++statusAcks; return true; }
bool tud_control_xfer(uint8_t, tusb_control_request_t const*, void* buffer, uint16_t) {
  controlBuffer = static_cast<uint8_t*>(buffer); return true;
}
}

static void openTransport() {
  mtpd_reset(0);
  controllerMtpStorageTask();
  memset(stalls, 0, sizeof(stalls));
  inPending = false; outBuffer = nullptr; failSubmit = false;
  const uint8_t desc[] = {TUD_MTP_DESCRIPTOR(4, 0, 0x82, 64, 1, 1, 0x81, 64)};
  assert(mtpd_open(0, reinterpret_cast<const tusb_desc_interface_t*>(desc), sizeof(desc)) == sizeof(desc));
  assert(outBuffer);
}

static void command(uint16_t code, std::initializer_list<uint32_t> params = {}, bool dispatchNow = true) {
  mtp_container_command_t c = {};
  c.header = {static_cast<uint32_t>(12 + params.size() * 4), MTP_CONTAINER_TYPE_COMMAND_BLOCK, code, 7};
  if (params.size()) memcpy(reinterpret_cast<uint8_t*>(&c) + 12, params.begin(), params.size() * 4);
  memcpy(outBuffer, &c, c.header.len);
  assert(mtpd_xfer_cb(0, 1, XFER_RESULT_SUCCESS, c.header.len));
  if (dispatchNow) controllerMtpStorageTask();
}

static std::vector<uint8_t> finish() {
  std::vector<uint8_t> bytes;
  unsigned chunks = 0;
  while (inPending) {
    assert(++chunks < 1000);
    const size_t size = inPacket.size();
    bytes.insert(bytes.end(), inPacket.begin(), inPacket.end());
    inPending = false;
    assert(mtpd_xfer_cb(0, 0x81, XFER_RESULT_SUCCESS, size));
    controllerMtpStorageTask();
  }
  return bytes;
}

static uint16_t responseCode(const std::vector<uint8_t>& bytes) {
  assert(bytes.size() >= 12);
  uint16_t code;
  memcpy(&code, bytes.data() + bytes.size() - 6, 2);
  return code;
}
static uint32_t u32(const uint8_t* bytes) { uint32_t v; memcpy(&v, bytes, 4); return v; }
static std::string mtpString(const uint8_t* bytes) {
  std::string value;
  for (unsigned i = 1; i < bytes[0]; ++i) value.push_back(bytes[1 + (i - 1) * 2]);
  return value;
}
static void session() {
  command(MTP_OP_OPEN_SESSION, {1});
  assert(responseCode(finish()) == MTP_RESP_OK);
}
static uint16_t deviceStatus() {
  tusb_control_request_t request = {};
  request.bmRequestType = 0xa1; request.bRequest = MTP_REQ_GET_DEVICE_STATUS;
  request.wIndex = 4; request.wLength = 4;
  assert(mtpd_control_xfer_cb(0, CONTROL_STAGE_SETUP, &request));
  uint16_t status; memcpy(&status, controlBuffer + 2, 2); return status;
}
static void resetClass() {
  tusb_control_request_t request = {};
  request.bmRequestType = 0x21; request.bRequest = MTP_REQ_RESET;
  request.wIndex = 4;
  unsigned previous = statusAcks;
  assert(mtpd_control_xfer_cb(0, CONTROL_STAGE_SETUP, &request));
  assert(statusAcks == previous + 1);
  assert(!inPending && controllerMtpTransportStatus() == MTP_RESP_TRANSACTION_CANCELLED);
  assert(mtpd_control_xfer_cb(0, CONTROL_STAGE_ACK, &request));
  controllerMtpStorageTask();
  assert(!session_open && !object_transfer_active);
}
static void cancel() {
  tusb_control_request_t request = {};
  request.bmRequestType = 0x21; request.bRequest = MTP_REQ_CANCEL; request.wLength = 6;
  request.wIndex = 4;
  assert(mtpd_control_xfer_cb(0, CONTROL_STAGE_SETUP, &request));
  const uint8_t data[] = {1, 0x40, 7, 0, 0, 0};
  memcpy(controlBuffer, data, 6);
  assert(mtpd_control_xfer_cb(0, CONTROL_STAGE_ACK, &request));
  controllerMtpStorageTask();
  assert(stalls[1] && stalls[0x81] && !inPending && !object_transfer_active);
}

static void clearHalts() {
  tusb_control_request_t request = {};
  request.bmRequestType = 0x02; request.bRequest = TUSB_REQ_CLEAR_FEATURE;
  request.wValue = TUSB_REQ_FEATURE_EDPT_HALT;
  for (uint8_t ep : {uint8_t(1), uint8_t(0x81)}) {
    usbd_edpt_clear_stall(0, ep); request.wIndex = ep;
    assert(mtpd_control_xfer_cb(0, CONTROL_STAGE_SETUP, &request));
  }
  assert(controllerMtpTransportStatus() == MTP_RESP_OK);
}

int main() {
  SdFat sd; FsFile log;
  controllerMtpStorageBegin(sd, log);
  controllerMtpStorageSetReady(true);
  fakeFiles = {{"log.csv", 51200}, {"folder", 0, true}, {"firmware.bin", 32}};
  openTransport();
  command(MTP_OP_GET_STORAGE_IDS);
  assert(responseCode(finish()) == MTP_RESP_SESSION_NOT_OPEN);
  command(MTP_OP_OPEN_SESSION, {0});
  assert(responseCode(finish()) == MTP_RESP_INVALID_PARAMETER);
  session();
  assert(deviceStatus() == MTP_RESP_OK);
  command(MTP_OP_GET_DEVICE_INFO);
  auto info = finish(); assert(responseCode(info) == MTP_RESP_OK);
  command(MTP_OP_GET_STORAGE_INFO, {STORAGE_ID});
  auto storage = finish();
  assert(u32(storage.data() + 26) == 0 && u32(storage.data() + 30) == 0); // free bytes
  command(MTP_OP_OPEN_SESSION, {2});
  auto duplicateSession = finish();
  assert(duplicateSession.size() == 16 && u32(duplicateSession.data() + 12) == 1);
  assert(duplicateSession[6] == (MTP_RESP_SESSION_ALREADY_OPEN & 255));
  mtp_container_command_t zeroTransaction = {};
  zeroTransaction.header = {12, MTP_CONTAINER_TYPE_COMMAND_BLOCK, MTP_OP_GET_STORAGE_IDS, 0};
  memcpy(outBuffer, &zeroTransaction, 12);
  mtpd_xfer_cb(0, 1, XFER_RESULT_SUCCESS, 12); controllerMtpStorageTask();
  assert(responseCode(finish()) == MTP_RESP_INVALID_TRANSACTION_ID);

  // Root-only view must never advertise unbrowseable folders; honor format.
  command(MTP_OP_GET_OBJECT_HANDLES, {STORAGE_ID, 0, 0});
  auto bytes = finish();
  assert(u32(bytes.data() + 12) == 2 && u32(bytes.data() + 16) == 1 && u32(bytes.data() + 20) == 3);
  command(MTP_OP_GET_OBJECT_HANDLES, {STORAGE_ID, MTP_OBJ_FORMAT_TEXT, UINT32_MAX});
  bytes = finish(); assert(u32(bytes.data() + 12) == 1);
  command(MTP_OP_GET_NUM_OBJECTS, {STORAGE_ID, 0, 0});
  bytes = finish(); assert(bytes.size() == 16 && u32(bytes.data() + 12) == 2);
  command(MTP_OP_GET_OBJECT_INFO, {2});
  assert(responseCode(finish()) == MTP_RESP_INVALID_OBJECT_HANDLE);

  command(MTP_OP_GET_OBJECT_PROP_VALUE, {1, MTP_OBJ_PROP_OBJECT_FILE_NAME});
  bytes = finish(); assert(mtpString(bytes.data() + 12) == "log.csv");
  command(MTP_OP_GET_OBJECT_PROP_VALUE, {1, MTP_OBJ_PROP_DATE_MODIFIED});
  bytes = finish(); assert(mtpString(bytes.data() + 12) == "20261024T120102");
  fakeFiles[0].date = 0;
  command(MTP_OP_GET_OBJECT_PROP_VALUE, {1, MTP_OBJ_PROP_DATE_CREATED});
  bytes = finish(); assert(mtpString(bytes.data() + 12) == "19800101T000000");
  command(MTP_OP_GET_OBJECT_PROPS_SUPPORTED, {MTP_OBJ_FORMAT_TEXT});
  bytes = finish(); assert(u32(bytes.data() + 12) == 8);
  command(MTP_OP_GET_OBJECT_PROP_DESC, {MTP_OBJ_PROP_OBJECT_FILE_NAME, MTP_OBJ_FORMAT_TEXT});
  bytes = finish(); assert(bytes[16] == 0); // GetSet = readonly

  // Zero length, full USB packet, full buffer, and multiple buffers/ZLP.
  for (uint32_t size : {0u, 52u, 16372u, 32756u, 51200u}) {
    fakeFiles[0].size = size;
    command(MTP_OP_GET_OBJECT, {1});
    assert(controllerMtpStorageTryLock()); controllerMtpStorageUnlock();
    bytes = finish();
    assert(bytes.size() == size + 24 && u32(bytes.data()) == size + 12);
    for (uint32_t i = 0; i < size; ++i) assert(bytes[12 + i] == static_cast<uint8_t>(i));
    assert(responseCode(bytes) == MTP_RESP_OK && !object_transfer_active);
  }
  fakeFiles[0].size = 51200;
  command(MTP_OP_GET_PARTIAL_OBJECT, {1, 512, 20000});
  bytes = finish(); assert(bytes.size() == 20028 && u32(bytes.data() + 20024) == 20000);
  for (unsigned i = 0; i < 20000; ++i) assert(bytes[12 + i] == static_cast<uint8_t>(512 + i));

  // A first-read failure must send a bounded 12-byte response, not file length.
  fakeReadFails = true;
  command(MTP_OP_GET_OBJECT, {1});
  bytes = finish(); assert(bytes.size() == 12 && responseCode(bytes) == MTP_RESP_GENERAL_ERROR);
  fakeReadFails = false;
  command(MTP_OP_GET_OBJECT, {1});
  fakeReadFails = true;
  inPending = false;
  mtpd_xfer_cb(0, 0x81, XFER_RESULT_SUCCESS, inPacket.size());
  controllerMtpStorageTask(); controllerMtpStorageTask();
  assert(stalls[1] && stalls[0x81] && !inPending && !object_transfer_active);
  fakeReadFails = false; resetClass(); session();

  command(MTP_OP_GET_OBJECT, {1}, false);
  const unsigned reads = fakeReads;
  cancel(); assert(fakeReads == reads); // cancellation before deferred dispatch
  resetClass(); session();
  command(MTP_OP_GET_OBJECT, {1}); cancel();
  assert(deviceStatus() == MTP_RESP_TRANSACTION_CANCELLED);
  clearHalts();
  command(MTP_OP_GET_OBJECT_PROP_VALUE, {1, MTP_OBJ_PROP_OBJECT_FILE_NAME});
  assert(mtpString(finish().data() + 12) == "log.csv"); // cancel preserves session
  command(MTP_OP_GET_OBJECT, {1});
  assert(deviceStatus() == MTP_RESP_DEVICE_BUSY);
  resetClass(); session(); // reset during an active hardware transfer
  command(MTP_OP_GET_OBJECT, {1}, false);
  mtpd_reset(0); controllerMtpStorageTask();
  assert(!session_open && !object_transfer_active && !inPending);
  openTransport(); session();
  command(MTP_OP_GET_OBJECT, {1});
  controllerMtpStorageSetReady(false);
  assert(!object_transfer_active && !inPending);
  controllerMtpStorageSetReady(true);
  resetClass(); session();
  command(MTP_OP_GET_OBJECT, {1});
  nowMs += 5000; controllerMtpStorageTask();
  assert(stalls[1] && !object_transfer_active);
  resetClass(); session();
  nowMs = UINT32_MAX - 1000;
  command(MTP_OP_GET_OBJECT, {1});
  nowMs += 5000; controllerMtpStorageTask();
  assert(stalls[1] && !object_transfer_active); // millis rollover
  resetClass(); session();

  command(MTP_OP_GET_OBJECT, {1});
  openTransport(); assert(!object_transfer_active && !session_open); // bus reset
  session();
  fakeFiles[0].size = UINT32_MAX;
  command(MTP_OP_GET_OBJECT, {1});
  bytes = finish(); assert(bytes.size() == 12 && responseCode(bytes) == MTP_RESP_OBJECT_TOO_LARGE);
  command(MTP_OP_GET_OBJECT_PROP_VALUE, {1, MTP_OBJ_PROP_OBJECT_SIZE});
  bytes = finish(); assert(u32(bytes.data() + 12) == UINT32_MAX && u32(bytes.data() + 16) == 0);
  command(MTP_OP_GET_PARTIAL_OBJECT, {1, UINT32_MAX - 10, 10});
  bytes = finish(); assert(u32(bytes.data()) == 22 && bytes.size() == 38);

  fakeFiles[0].size = 51200;
  command(MTP_OP_GET_OBJECT, {1});
  inPending = false;
  mtpd_xfer_cb(0, 0x81, XFER_RESULT_SUCCESS, 32); // successful but short hardware transfer
  controllerMtpStorageTask(); assert(stalls[1] && !object_transfer_active);
  resetClass(); session();

  // A truncated command must not reuse parameters from the preceding command.
  command(MTP_OP_GET_OBJECT, {1}, false); cancel(); resetClass(); session();
  mtp_container_header_t shortCommand = {12, MTP_CONTAINER_TYPE_COMMAND_BLOCK, MTP_OP_GET_OBJECT, 8};
  memcpy(outBuffer, &shortCommand, 12);
  mtpd_xfer_cb(0, 1, XFER_RESULT_SUCCESS, 12);
  controllerMtpStorageTask(); assert(stalls[1] && !inPending);
  resetClass(); session();
  mtpd_xfer_cb(0, 1, XFER_RESULT_FAILED, 0);
  assert(stalls[1]); resetClass(); session();

  fakeFiles[0].size = 51200;
  failSubmit = true;
  command(MTP_OP_GET_OBJECT, {1});
  controllerMtpStorageTask();
  assert(!object_transfer_active && stalls[1] && !inPending);
  failSubmit = false;
  resetClass(); session();
  failSubmit = true;
  command(MTP_OP_GET_OBJECT_PROP_VALUE, {1, MTP_OBJ_PROP_OBJECT_FILE_NAME});
  assert(stalls[1] && !inPending); // metadata submission failure
  failSubmit = false; resetClass(); session();
  failSubmit = true;
  command(MTP_OP_GET_OBJECT_INFO, {999});
  assert(stalls[1] && !inPending); // response submission failure
  failSubmit = false;
  puts("MTP regression tests passed (application + transport).");
}
