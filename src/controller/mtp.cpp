#include "controller/mtp.h"
#include "controller/mtp_time.h"
#include <algorithm>
#include <cstring>
#include <strings.h>

namespace controller::mtp {
namespace {
uint16_t get16(const uint8_t* p) { return uint16_t(p[0]) | (uint16_t(p[1]) << 8); }
uint32_t get32(const uint8_t* p) { return get16(p) | (uint32_t(get16(p + 2)) << 16); }
void put16(uint8_t* p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
void put32(uint8_t* p, uint32_t v) { put16(p, v); put16(p + 2, v >> 16); }
constexpr uint16_t operations[] = {0x1001,0x1002,0x1003,0x1004,0x1005,0x1006,0x1007,0x1008,0x1009,0x1014,0x1015,0x101b};
uint16_t format(const Object& object) {
  if (object.directory) return 0x3001;
  const char* suffix = strrchr(object.name.c_str(), '.');
  return suffix && (!strcasecmp(suffix, ".csv") || !strcasecmp(suffix, ".txt")) ? 0x3004 : 0x3000;
}
}
void Responder::u16(uint16_t v) { payload_.push_back(v); payload_.push_back(v >> 8); }
void Responder::u32(uint32_t v) { u16(v); u16(v >> 16); }
void Responder::u64(uint64_t v) { u32(v); u32(v >> 32); }
void Responder::codes(const uint16_t* values, size_t count) { u32(count); for (size_t i=0;i<count;++i) u16(values[i]); }
void Responder::string(const std::string& value) {
  // Decode UTF-8 filenames into the UTF-16LE representation required by MTP.
  std::vector<uint16_t> units;
  for (size_t i=0; i<value.size() && units.size()<253;) {
    uint32_t cp = static_cast<uint8_t>(value[i++]);
    unsigned tails = cp < 0x80 ? 0 : (cp & 0xe0) == 0xc0 ? 1 : (cp & 0xf0) == 0xe0 ? 2 : (cp & 0xf8) == 0xf0 ? 3 : 4;
    if (tails == 4) cp = 0xfffd;
    else if (tails) {
      cp &= (1u << (6-tails))-1;
      bool valid = true;
      for (unsigned n=0;n<tails;++n) {
        if (i==value.size() || (static_cast<uint8_t>(value[i]) & 0xc0)!=0x80) { valid=false; break; }
        cp = (cp<<6) | (static_cast<uint8_t>(value[i++]) & 0x3f);
      }
      const uint32_t minimum[] = {0,0x80,0x800,0x10000};
      if (!valid || cp<minimum[tails] || cp>0x10ffff || (cp>=0xd800 && cp<=0xdfff)) cp=0xfffd;
    }
    if (cp>0xffff) { cp-=0x10000; units.push_back(0xd800+(cp>>10)); units.push_back(0xdc00+(cp&0x3ff)); }
    else units.push_back(cp);
  }
  if (units.empty()) { payload_.push_back(0); return; }
  payload_.push_back(units.size()+1);
  for (auto unit:units) u16(unit);
  u16(0);
}
void Responder::cancel() {
  storage_.close(); file_=false; phase_=Phase::Idle; payload_.clear();
}
void Responder::reset() {
  cancel(); session_=false; objects_.clear();
}
void Responder::fail(uint16_t code) { response_=code; phase_=Phase::Response; payload_.clear(); storage_.close(); file_=false; }
bool Responder::command(const uint8_t* data, size_t length) {
  if (active()) return false;
  if (!data || length<12 || length>32 || get32(data)!=length || get16(data+4)!=1 || (length-12)%4) return false;
  operation_=get16(data+6); transaction_=get32(data+8); params_.fill(0);
  static constexpr uint8_t parameterCounts[] = {0,0,1,0,0,1,3,3,1,1};
  size_t expected = operation_>=0x1001 && operation_<=0x1009 ? parameterCounts[operation_-0x1000] :
      operation_==0x101b ? 3 : (operation_==0x1014 || operation_==0x1015) ? 1 : (length-12)/4;
  for (size_t i=0;i<(length-12)/4;++i) params_[i]=get32(data+12+i*4);
  response_=0x2001; partial_=false; file_=false; sent_=0; payloadSize_=0; offset_=0; payload_.clear(); phase_=Phase::Response;
  if ((length-12)/4!=expected) { fail(0x201d); return true; }
  prepare();
  if (phase_==Phase::Data) payloadSize_=file_ ? payloadSize_ : static_cast<uint32_t>(payload_.size());
  return true;
}
bool Responder::catalog() {
  storage_.flush();
  std::vector<Object> fresh;
  if (!storage_.list(fresh)) return false;
  // Preserve assigned handles throughout the session, even when new logs appear.
  for (auto& item:fresh) {
    auto found=std::find_if(objects_.begin(),objects_.end(),[&](const Object& old){return old.name==item.name;});
    if (found==objects_.end()) { if (objects_.size()==1024) return false; objects_.push_back(std::move(item)); }
    else *found=std::move(item);
  }
  return true;
}
Object* Responder::object(uint32_t handle) {
  if (!handle || handle>objects_.size()) return nullptr;
  auto& item=objects_[handle-1];
  return storage_.stat(item) ? &item : nullptr;
}
void Responder::prepare() {
  if (operation_==0x1001) {
    phase_=Phase::Data;
    u16(100); u32(6); u16(100); string("microsoft.com: 1.0; "); u16(0);
    codes(operations,sizeof(operations)/sizeof(*operations)); codes(nullptr,0);
    const uint16_t properties[]={0xd402}; codes(properties,1);
    const uint16_t formats[]={0x3000,0x3001,0x3004}; codes(formats,3); codes(formats,3);
    string("Shuriken"); string("SD"); string("0.2"); string("WiFi-Shuriken"); return;
  }
  if (operation_==0x1002) {
    if (!params_[0]) { fail(0x201d); return; }
    if (session_) { fail(0x201e); return; }
    objects_.clear(); session_=true; return;
  }
  if (!session_) { fail(0x2003); return; }
  if (operation_==0x1003) { storage_.close(); session_=false; objects_.clear(); return; }
  if (operation_==0x1004) { phase_=Phase::Data; u32(storage_.ready()?1:0); if(storage_.ready()) u32(storageId); return; }
  if (operation_==0x1014 || operation_==0x1015) {
    if (params_[0]!=0xd402) { fail(0x200a); return; }
    phase_=Phase::Data;
    if (operation_==0x1014) { u16(0xd402); u16(0xffff); payload_.push_back(0); string("Shuriken"); }
    string("Shuriken"); if(operation_==0x1014) payload_.push_back(0); return;
  }
  if (std::find(std::begin(operations),std::end(operations),operation_)==std::end(operations)) { fail(0x2005); return; }
  if (!storage_.ready()) { fail(0x2013); return; }
  if (operation_>=0x1005 && operation_<=0x1007) {
    if (params_[0]!=storageId && !(operation_!=0x1005 && params_[0]==UINT32_MAX)) { fail(0x2008); return; }
    if (operation_==0x1005) {
      phase_=Phase::Data; u16(4); u16(2); u16(1); u64(storage_.capacity()); u64(0); u32(0); string("SD"); string("Shuriken"); return;
    }
    if (params_[1]) { fail(0x2014); return; }
    if (!catalog()) { fail(0x2002); return; }
    const uint32_t count=params_[2]==0 || params_[2]==UINT32_MAX ? objects_.size() : 0;
    if (operation_==0x1006) { u32(count); return; }
    phase_=Phase::Data; u32(count); for(uint32_t i=0;i<count;++i) u32(i+1); return;
  }
  storage_.flush();
  Object* item=object(params_[0]);
  if (!item) { fail(0x2009); return; }
  if (operation_==0x1008) {
    phase_=Phase::Data; u32(storageId); u16(format(*item)); u16(1); u32(item->size);
    u16(0x3000); for(unsigned i=0;i<7;++i) u32(0);
    u16(item->directory?1:0); u32(0); u32(0);
    char modified[17];
    formatTimestamp(item->modified, modified);
    string(item->name); string(""); string(modified); string(""); return;
  }
  if (item->directory || !storage_.open(*item)) { fail(0x2009); return; }
  partial_=operation_==0x101b; offset_=partial_?params_[1]:0;
  if (offset_>item->size) { fail(0x201d); return; }
  payloadSize_=std::min(item->size-offset_,UINT32_MAX-12u);
  if (partial_) payloadSize_=std::min(payloadSize_,params_[2]);
  file_=true; phase_=Phase::Data;
}
size_t Responder::next(uint8_t* data, size_t capacity) {
  if (!data || capacity<32 || phase_==Phase::Idle) return 0;
  if (phase_==Phase::Response) {
    const uint32_t parameter = operation_==0x1006 && response_==0x2001 ? (payload_.size()==4 ? get32(payload_.data()):0) : payloadSize_;
    const bool extra=response_==0x2001 && (partial_ || operation_==0x1006);
    put32(data,extra?16:12); put16(data+4,3); put16(data+6,response_); put32(data+8,transaction_);
    if(extra) put32(data+12,parameter);
    storage_.close(); file_=false; phase_=Phase::Idle; return extra?16:12;
  }
  const size_t header=sent_==0?12:0;
  if(header) { put32(data,payloadSize_+12); put16(data+4,2); put16(data+6,operation_); put32(data+8,transaction_); }
  const size_t count=std::min<size_t>(capacity-header,payloadSize_-sent_);
  if (count) {
    if (file_) {
      if (!storage_.read(offset_+sent_,data+header,count)) { fail(0x2002); return 0; }
    } else memcpy(data+header,payload_.data()+sent_,count);
  }
  sent_+=count;
  if(sent_==payloadSize_) { storage_.close(); file_=false; phase_=Phase::Response; }
  return count+header;
}
}
