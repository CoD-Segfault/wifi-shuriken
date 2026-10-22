#include "device/usbd_pvt.h"
#include "posix/mtp_storage.h"
#include "p4/usb_mtp.h"
#include <array>
#include <iostream>
namespace p4 {bool storageReady(){return true;}}
unsigned flushes=0;
void poll(){p4::pollUsbMtp([]{assert(!fake_mtp::inUsb);++flushes;});}
void put32(uint8_t* p,uint32_t v){for(unsigned i=0;i<4;++i)p[i]=v>>(8*i);}
int main(){
 using namespace fake_mtp;
 assert(registrations==1 && descriptor);uint8_t count=0;auto* driver=usbd_app_driver_get_cb(&count);assert(count==1);
 std::array<uint8_t,30> descriptors{};uint8_t itf=4;assert(descriptor(descriptors.data(),&itf)==30 && itf==5);
 assert(descriptors[5]==6 && descriptors[6]==1 && descriptors[7]==1 && descriptors[8]==7);
 driver->init();inUsb=true;assert(driver->open(1,reinterpret_cast<const tusb_desc_interface_t*>(descriptors.data()),30)==30);inUsb=false;
 assert(arms==0 && lists==0);poll();assert(arms==1);
 auto send=[&](uint16_t op,std::initializer_list<uint32_t> params){
   assert(busyOut);memset(outBuffer,0,512);put32(outBuffer,12+params.size()*4);outBuffer[4]=1;outBuffer[6]=op;outBuffer[7]=op>>8;
   size_t offset=12;for(auto p:params){put32(outBuffer+offset,p);offset+=4;}
   busyOut=false;inUsb=true;assert(driver->xfer_cb(1,1,XFER_RESULT_SUCCESS,offset));inUsb=false;poll();
 };
 auto ack=[&]{assert(busyIn);busyIn=false;inUsb=true;assert(driver->xfer_cb(1,0x81,XFER_RESULT_SUCCESS,packets.back().size()));inUsb=false;poll();};
 auto drain=[&]{while(busyIn)ack();assert(busyOut);};
 send(0x1002,{1});drain();send(0x1007,{0x10001,0,0});drain();assert(lists==1);
 const auto start=packets.size();send(0x1009,{1});assert(reads==1 && packets.back().size()==512);
 poll();assert(packets.size()==start+1 && reads==1); // A stalled host never blocks the loop or reuses its DMA buffer.
 ack();assert(packets.back().empty());ack();assert(packets.back().size()==12);drain();
 assert(packets.size()==start+3 && flushes>0);
 // Unsupported writes get only a response and never touch the filesystem.
 auto oldReads=reads,oldLists=lists;send(0x100b,{1});assert(packets.back()[6]==5 && packets.back()[7]==0x20);drain();assert(reads==oldReads && lists==oldLists);
 // Cancellation releases the file on the application thread and keeps the session usable.
 send(0x1009,{1});auto before=closes;
 tusb_control_request_t request{};request.bmRequestType_bit.type=TUSB_REQ_TYPE_CLASS;request.wIndex=4;request.bRequest=0x64;request.wLength=6;
 inUsb=true;assert(driver->control_xfer_cb(1,CONTROL_STAGE_SETUP,&request));inUsb=false;assert(closes==before);poll();assert(closes>before && busyOut);
 send(0x1009,{1});drain();
 // Disconnect during a transfer closes on core0; re-enumeration starts a new session.
 send(0x1009,{1});before=closes;inUsb=true;driver->reset(1);inUsb=false;assert(closes==before);busyIn=busyOut=false;poll();assert(closes>before);
 inUsb=true;driver->open(1,reinterpret_cast<const tusb_desc_interface_t*>(descriptors.data()),30);inUsb=false;poll();
 send(0x1009,{1});assert(packets.back()[6]==3 && packets.back()[7]==0x20);drain();
 // Failed IN submission halts both endpoints and releases transaction state.
 send(0x1002,{2});drain();send(0x1007,{0x10001,0,0});drain();failSend=true;send(0x1009,{1});assert(!busyIn && !busyOut);
 std::cout<<"P4 MTP USB descriptor, bounded transfers, ZLP, cancel and disconnect passed\n";
}
