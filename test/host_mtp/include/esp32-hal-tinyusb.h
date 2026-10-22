#pragma once
#include <cassert>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <vector>
#define TUSB_DESC_INTERFACE 4
#define TUSB_DESC_ENDPOINT 5
#define TUSB_XFER_INTERRUPT 3
#define TUSB_REQ_TYPE_CLASS 1
#define CONTROL_STAGE_SETUP 1
#define USB_INTERFACE_CUSTOM 8
using tinyusb_descriptor_cb_t=uint16_t(*)(uint8_t*,uint8_t*);
namespace fake_mtp {
inline tinyusb_descriptor_cb_t descriptor=nullptr;
inline unsigned registrations=0;
inline uint8_t nextEp=1;
inline uint8_t *outBuffer=nullptr;
inline unsigned arms=0, stalls=0;
inline bool busyIn=false,busyOut=false;
inline std::vector<std::vector<uint8_t>> packets;
inline bool failSend=false;
}
inline int tinyusb_enable_interface(int kind,uint16_t len,tinyusb_descriptor_cb_t cb) { assert(kind==USB_INTERFACE_CUSTOM && len==30);fake_mtp::descriptor=cb;++fake_mtp::registrations;return 0;}
inline uint8_t tinyusb_get_free_duplex_endpoint(){return fake_mtp::nextEp++;}
inline uint8_t tinyusb_get_free_in_endpoint(){return fake_mtp::nextEp++;}
inline uint8_t tinyusb_add_string_descriptor(const char* name){assert(strcmp(name,"MTP")==0);return 7;}
struct __attribute__((packed)) tusb_desc_interface_t {uint8_t bLength,bDescriptorType,bInterfaceNumber,bAlternateSetting,bNumEndpoints,bInterfaceClass,bInterfaceSubClass,bInterfaceProtocol,iInterface;};
struct __attribute__((packed)) tusb_desc_endpoint_t {uint8_t bLength,bDescriptorType,bEndpointAddress;struct {uint8_t xfer:2;uint8_t unused:6;} bmAttributes;uint16_t wMaxPacketSize;uint8_t bInterval;};
struct tusb_control_request_t {struct {uint8_t recipient:5;uint8_t type:2;uint8_t direction:1;} bmRequestType_bit;uint8_t bRequest;uint16_t wValue,wIndex,wLength;};
enum xfer_result_t {XFER_RESULT_SUCCESS,XFER_RESULT_FAILED};
inline uint16_t tu_edpt_packet_size(const tusb_desc_endpoint_t* ep){return ep->wMaxPacketSize;}
inline bool tud_control_xfer(uint8_t,const tusb_control_request_t*,void*,uint16_t){return true;}
inline bool tud_control_status(uint8_t,const tusb_control_request_t*){return true;}
