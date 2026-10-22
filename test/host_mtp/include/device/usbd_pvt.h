#pragma once
#include "esp32-hal-tinyusb.h"
struct usbd_class_driver_t {
 const char* name; void(*init)();bool(*deinit)();void(*reset)(uint8_t);
 uint16_t(*open)(uint8_t,const tusb_desc_interface_t*,uint16_t);
 bool(*control_xfer_cb)(uint8_t,uint8_t,const tusb_control_request_t*);
 bool(*xfer_cb)(uint8_t,uint8_t,xfer_result_t,uint32_t);
 bool(*xfer_isr)(uint8_t,uint8_t,xfer_result_t,uint32_t);void(*sof)(uint8_t,uint32_t);
};
extern "C" const usbd_class_driver_t* usbd_app_driver_get_cb(uint8_t*);
inline bool usbd_edpt_open(uint8_t port,const tusb_desc_endpoint_t*){assert(port==1);return true;}
inline bool usbd_edpt_busy(uint8_t,uint8_t ep){return ep&0x80?fake_mtp::busyIn:fake_mtp::busyOut;}
inline bool usbd_edpt_xfer(uint8_t port,uint8_t ep,uint8_t* data,uint16_t length,bool isr){
 assert(port==1 && !isr);
 if(fake_mtp::failSend){fake_mtp::failSend=false;return false;}
 if(ep&0x80){assert(!fake_mtp::busyIn);fake_mtp::busyIn=true;fake_mtp::packets.emplace_back(data,data+length);}
 else{assert(!fake_mtp::busyOut && length==512);fake_mtp::busyOut=true;fake_mtp::outBuffer=data;++fake_mtp::arms;}
 return true;
}
inline void usbd_edpt_stall(uint8_t,uint8_t ep){++fake_mtp::stalls;if(ep&0x80)fake_mtp::busyIn=false;else fake_mtp::busyOut=false;}
inline void usbd_edpt_clear_stall(uint8_t,uint8_t){}
