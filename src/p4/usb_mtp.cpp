#include "Configuration.h"
#if defined(SHURIKEN_P4_CONTROLLER) && P4_USB_MTP
#include "usb_mtp.h"
#include "controller/mtp.h"
#include "posix/mtp_storage.h"
#include "sdmmc_storage.h"
#include <SD_MMC.h>
#include <esp32-hal-tinyusb.h>
#include "device/usbd_pvt.h"
#include <atomic>
#include <cstring>
#if ARDUINO_USB_MODE || !ARDUINO_USB_CDC_ON_BOOT
#error "P4 MTP requires the native TinyUSB console build"
#endif
namespace {
constexpr uint16_t descriptorLength=30;
constexpr size_t bufferSize=4096;
alignas(4) uint8_t incoming[512], outgoing[bufferSize];
uint8_t rhport=1, epOut=0, epIn=0, epEvent=0, interfaceNumber=0;
std::atomic<bool> configured{false}, resetRequested{false}, txDone{false};
std::atomic<uint32_t> rxLength{0};
std::atomic<bool> transferError{false}, cancelRequested{false};
bool transmitting=false, zlpPending=false, transaction=false;
size_t dataBytes=0;
uint16_t packetSize=512;
void (*flushActiveLog)()=nullptr;
class SdStorage final : public controller::mtp::PosixStorage {
 public:
  SdStorage():PosixStorage("/sdcard") {}
  bool ready() const override { return p4::storageReady(); }
  uint64_t capacity() const override { return SD_MMC.cardSize(); }
  void flush() override { if(flushActiveLog) flushActiveLog(); }
};
SdStorage storage;
controller::mtp::Responder responder(storage);
uint16_t descriptor(uint8_t* dst,uint8_t* itf) {
  const uint8_t bulk=tinyusb_get_free_duplex_endpoint();
  const uint8_t event=tinyusb_get_free_in_endpoint();
  if(!bulk || !event) return 0;
  const uint8_t str=tinyusb_add_string_descriptor("MTP");
  const uint8_t bytes[]={9,TUSB_DESC_INTERFACE,(*itf)++,0,3,6,1,1,str,
      7,TUSB_DESC_ENDPOINT,bulk,2,0,2,0,
      7,TUSB_DESC_ENDPOINT,uint8_t(bulk|0x80),2,0,2,0,
      7,TUSB_DESC_ENDPOINT,uint8_t(event|0x80),3,28,0,6};
  memcpy(dst,bytes,sizeof(bytes)); return sizeof(bytes);
}
struct Register {
  Register() { tinyusb_enable_interface(USB_INTERFACE_CUSTOM,descriptorLength,descriptor); }
} registration;
void init() {}
bool deinit() { configured=false; resetRequested=true; return true; }
void reset(uint8_t) { configured=false; resetRequested=true; }
uint16_t open(uint8_t port,tusb_desc_interface_t const* desc,uint16_t length) {
  if(desc->bInterfaceClass!=6 || desc->bInterfaceSubClass!=1 || desc->bInterfaceProtocol!=1 || length<descriptorLength) return 0;
  rhport=port; interfaceNumber=desc->bInterfaceNumber;
  auto* endpoint=reinterpret_cast<const tusb_desc_endpoint_t*>(reinterpret_cast<const uint8_t*>(desc)+9);
  for(unsigned i=0;i<3;++i,++endpoint) {
    if(!usbd_edpt_open(port,endpoint)) return 0;
    if(endpoint->bmAttributes.xfer==TUSB_XFER_INTERRUPT) epEvent=endpoint->bEndpointAddress;
    else if(endpoint->bEndpointAddress&0x80) { epIn=endpoint->bEndpointAddress; packetSize=tu_edpt_packet_size(endpoint); }
    else epOut=endpoint->bEndpointAddress;
  }
  rxLength=0; txDone=false; transferError=false;
  // Do not read commands until the application has acknowledged the bus reset.
  resetRequested=true; configured=true; return descriptorLength;
}
bool control(uint8_t port,uint8_t stage,tusb_control_request_t const* request) {
  if(request->wIndex!=interfaceNumber || request->bmRequestType_bit.type!=TUSB_REQ_TYPE_CLASS) return false;
  static uint8_t cancelData[6];
  static uint8_t status[]={4,0,1,0x20};
  if(stage!=CONTROL_STAGE_SETUP) return true;
  switch(request->bRequest) {
    // Cancel is acknowledged without touching files in the USB task. The host
    // clears halted bulk endpoints before issuing its next transaction.
    case 0x64:
      if(request->wLength!=sizeof(cancelData)) return false;
      cancelRequested=true; return tud_control_xfer(port,request,cancelData,sizeof(cancelData));
    case 0x66: resetRequested=true; return tud_control_status(port,request);
    case 0x67: return tud_control_xfer(port,request,status,sizeof(status));
    default:return false;
  }
}
bool complete(uint8_t,uint8_t ep,xfer_result_t result,uint32_t length) {
  if(result!=XFER_RESULT_SUCCESS) { transferError=true; return true; }
  if(ep==epOut) rxLength=length;
  else if(ep==epIn) txDone=true;
  return true;
}
const usbd_class_driver_t driver={"MTP",init,deinit,reset,open,control,complete,nullptr,nullptr};
void armCommand() {
  if(configured && !usbd_edpt_busy(rhport,epOut))
    if(!usbd_edpt_xfer(rhport,epOut,incoming,sizeof(incoming),false)) transferError=true;
}
void haltTransaction() {
  responder.reset(); transaction=false; transmitting=false; zlpPending=false;
  usbd_edpt_stall(rhport,epIn); usbd_edpt_stall(rhport,epOut);
}
}
extern "C" usbd_class_driver_t const* usbd_app_driver_get_cb(uint8_t* count) { *count=1; return &driver; }
namespace p4 {
void pollUsbMtp(void (*flushLog)()) {
  flushActiveLog=flushLog;
  const bool busReset=resetRequested.exchange(false);
  if(cancelRequested.exchange(false) || busReset) {
    if(busReset) responder.reset(); else responder.cancel(); transaction=false; transmitting=false; zlpPending=false;
    rxLength=0; txDone=false; dataBytes=0;
    if(configured) {
      // Abort outstanding DMA before reusing either buffer after cancellation.
      usbd_edpt_stall(rhport,epIn); usbd_edpt_stall(rhport,epOut);
      usbd_edpt_clear_stall(rhport,epIn); usbd_edpt_clear_stall(rhport,epOut);
      armCommand();
    }
  }
  if(!configured) return;
  if(transferError.exchange(false)) { haltTransaction(); return; }
  if(transmitting) {
    if(!txDone.exchange(false)) return;
    transmitting=false;
  }
  const uint32_t length=rxLength.exchange(0);
  if(length) {
    if(!responder.command(incoming,length)) { haltTransaction(); return; }
    transaction=true; dataBytes=0;
  }
  if(!transaction) return;
  if(zlpPending) {
    zlpPending=false; transmitting=true;
    if(!usbd_edpt_xfer(rhport,epIn,outgoing,0,false)) { transmitting=false; haltTransaction(); }
    return;
  }
  const size_t count=responder.next(outgoing,sizeof(outgoing));
  if(!count) {
    if(responder.active()) { haltTransaction(); return; }
    transaction=false; armCommand(); return;
  }
  // A short packet (or ZLP) terminates the data container before its response.
  const bool response= !responder.active();
  if(!dataBytes && !response) dataBytes=uint32_t(outgoing[0]) | (uint32_t(outgoing[1])<<8) | (uint32_t(outgoing[2])<<16) | (uint32_t(outgoing[3])<<24);
  if(dataBytes) {
    dataBytes-=count;
    if(!dataBytes && count%packetSize==0) zlpPending=true;
  }
  transmitting=true;
  if(!usbd_edpt_xfer(rhport,epIn,outgoing,count,false)) { transmitting=false; haltTransaction(); }
}
}
#endif
