#include "controller/mtp.h"
#include "controller/mtp_time.h"
#include "posix/mtp_storage.h"
#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <array>
#include <unistd.h>
#include <fcntl.h>
namespace fs=std::filesystem;
uint32_t get32(const uint8_t* p) { return uint32_t(p[0]) | (uint32_t(p[1])<<8) | (uint32_t(p[2])<<16) | (uint32_t(p[3])<<24); }
uint16_t get16(const uint8_t* p) { return p[0] | (uint16_t(p[1])<<8); }
void put32(uint8_t* p,uint32_t v) { for(unsigned i=0;i<4;++i) p[i]=v>>(8*i); }
struct Result { std::vector<uint8_t> data; uint16_t code=0; uint32_t parameter=0; };
Result run(controller::mtp::Responder& server,uint16_t op,std::initializer_list<uint32_t> params={}) {
  std::array<uint8_t,32> command{}; put32(command.data(),12+4*params.size()); command[4]=1; command[6]=op; command[7]=op>>8; put32(command.data()+8,17);
  size_t pos=12; for(auto p:params) {put32(command.data()+pos,p);pos+=4;}
  assert(server.command(command.data(),pos));
  assert(!server.command(command.data(),pos));
  Result result; std::array<uint8_t,512> chunk{};
  while(server.active()) {
    const size_t count=server.next(chunk.data(),chunk.size()); assert(count);
    if(!server.active()) {assert(get32(chunk.data())==count && get16(chunk.data()+4)==3 && get32(chunk.data()+8)==17); result.code=get16(chunk.data()+6);if(count==16)result.parameter=get32(chunk.data()+12);}
    else result.data.insert(result.data.end(),chunk.begin(),chunk.begin()+count);
  }
  if(!result.data.empty()) {assert(get32(result.data.data())==result.data.size());assert(get16(result.data.data()+4)==2);result.data.erase(result.data.begin(),result.data.begin()+12);}
  return result;
}

std::string readString(const std::vector<uint8_t>& data, size_t& offset) {
  assert(offset<data.size()); const uint8_t count=data[offset++];
  assert(offset+2u*count<=data.size()); std::string result;
  for(unsigned i=0;i+1<count;++i) result.push_back(char(get16(data.data()+offset+2*i)));
  offset+=2u*count; return result;
}
class Storage:public controller::mtp::PosixStorage {
 public:
  using PosixStorage::PosixStorage;
  void flush() override {++flushes;}
  void close() override {++closes;PosixStorage::close();}
  unsigned flushes=0,closes=0;
};
int main() {
  char pattern[]="/tmp/shuriken-mtp-XXXXXX";char* root=mkdtemp(pattern);assert(root);
  std::string contents; for(unsigned i=0;i<13000;++i)contents.push_back(char(i%251));
  {std::ofstream file(fs::path(root)/"scan.csv",std::ios::binary);file<<contents;}
  {std::ofstream file(fs::path(root)/u8"é.txt");file<<"hello";}
  fs::create_directory(fs::path(root)/"folder");fs::create_symlink("/etc/passwd",fs::path(root)/"escape");
  // Pin a real on-disk modification time, independent of the test machine clock.
  controller::hal::EpochSeconds modified;
  assert(controller::hal::epochFromUtc({2026,10,21,20,9,58}, modified));
  timespec times[2]={{static_cast<time_t>(modified),0},{static_cast<time_t>(modified),0}};
  assert(utimensat(AT_FDCWD,(fs::path(root)/"scan.csv").c_str(),times,0)==0);
  Storage storage(root);controller::mtp::Responder server(storage);
  auto result=run(server,0x1001);assert(result.code==0x2001 && result.data.size()>100);
  assert(run(server,0x1004).code==0x2003);
  assert(run(server,0x1002,{0}).code==0x201d);
  assert(run(server,0x1002,{1}).code==0x2001);
  assert(run(server,0x1002,{1}).code==0x201e);
  result=run(server,0x1004);assert(get32(result.data.data())==1 && get32(result.data.data()+4)==server.storageId);
  assert(run(server,0x1005,{42}).code==0x2008);
  result=run(server,0x1005,{server.storageId});assert(get16(result.data.data()+4)==1); // read-only
  result=run(server,0x1007,{server.storageId,0,0});assert(get32(result.data.data())==3);
  assert(run(server,0x1006,{server.storageId,0,0}).parameter==3);
  assert(run(server,0x1007,{server.storageId,0x3004,0}).code==0x2014);
  assert(run(server,0x1007,{server.storageId,0,99}).data.size()==4);
  uint32_t csv=0,folder=0,unicode=0;
  for(uint32_t handle=1;handle<=3;++handle) {
    result=run(server,0x1008,{handle});assert(result.code==0x2001 && result.data.size()>52);
    if(get16(result.data.data()+4)==0x3001)folder=handle;
    else if(get32(result.data.data()+8)==contents.size())csv=handle;
    else unicode=handle;
  }
  assert(csv && folder && unicode);
  result=run(server,0x1008,{csv}); size_t stringOffset=52;
  assert(readString(result.data,stringOffset)=="scan.csv");
  assert(readString(result.data,stringOffset).empty()); // POSIX ctime is not creation time.
  assert(readString(result.data,stringOffset)=="20261021T200958Z");
  // Restat each GetObjectInfo: a later append/mtime update must not use stale catalog metadata.
  times[1].tv_sec+=60;
  assert(utimensat(AT_FDCWD,(fs::path(root)/"scan.csv").c_str(),times,0)==0);
  setenv("TZ","America/Chicago",1); tzset();
  result=run(server,0x1008,{csv}); stringOffset=52;
  readString(result.data,stringOffset); readString(result.data,stringOffset);
  assert(readString(result.data,stringOffset)=="20261021T201058Z");
  char timestamp[17]; assert(!controller::mtp::formatTimestamp(0,timestamp) && !timestamp[0]);
  assert(!controller::mtp::formatTimestamp(-1,timestamp));
  assert(!controller::mtp::formatTimestamp(INT64_MAX,timestamp));
  const uint16_t fatDate=((2026-1980)<<9)|(10<<5)|21;
  const uint16_t fatTime=(20<<11)|(9<<5)|29;
  assert(controller::mtp::fatTimestamp(fatDate,fatTime)==modified);
  assert(controller::mtp::fatTimestamp(0,0)==0);
  assert(controller::mtp::formatTimestamp(modified,timestamp));
  assert(std::string(timestamp)=="20261021T200958Z");
  result=run(server,0x1008,{unicode});assert(result.data[52]==6 && get16(result.data.data()+53)==0xe9);
  result=run(server,0x1009,{csv});assert(std::string(result.data.begin(),result.data.end())==contents);
  result=run(server,0x101b,{csv,511,9000});assert(result.parameter==9000 && std::string(result.data.begin(),result.data.end())==contents.substr(511,9000));
  result=run(server,0x101b,{csv,uint32_t(contents.size()-2),500});assert(result.parameter==2 && result.data.size()==2);
  result=run(server,0x101b,{csv,uint32_t(contents.size()),1});assert(result.parameter==0 && result.data.empty());
  assert(run(server,0x101b,{csv,UINT32_MAX,4}).code==0x201d);
  assert(run(server,0x1009,{folder}).code==0x2009);
  assert(run(server,0x1009,{99}).code==0x2009);
  for(uint16_t write:{0x100b,0x100c,0x100d,0x1016,0x1017})assert(run(server,write).code==0x2005);
  assert(fs::file_size(fs::path(root)/"scan.csv")==contents.size());
  {std::ofstream file(fs::path(root)/"new.csv");file<<"new";}
  result=run(server,0x1007,{server.storageId,0,UINT32_MAX});assert(get32(result.data.data())==4);
  result=run(server,0x1009,{csv});assert(std::string(result.data.begin(),result.data.end())==contents);
  // Cancel/reset must release open files and session/catalog state.
  std::array<uint8_t,16> command{};put32(command.data(),16);command[4]=1;command[6]=9;command[7]=0x10;put32(command.data()+12,csv);
  assert(server.command(command.data(),16));unsigned before=storage.closes;server.reset();assert(storage.closes>before && !server.active());
  assert(run(server,0x1009,{csv}).code==0x2003);
  assert(run(server,0x1002,{2}).code==0x2001);
  assert(run(server,0x1009,{csv}).code==0x2009);
  // Malformed containers never become transactions; missing params receive a response.
  assert(!server.command(nullptr,16)); assert(!server.command(command.data(),11));
  command[0]=15;assert(!server.command(command.data(),16));command[0]=16;command[4]=2;assert(!server.command(command.data(),16));
  assert(run(server,0x1009).code==0x201d);
  controller::mtp::Object traversal{"../outside",0,false}; assert(!storage.stat(traversal) && !storage.open(traversal));
  controller::mtp::Object symlink{"escape",0,false};assert(!storage.stat(symlink));
  assert(storage.flushes>0);
  fs::remove_all(root);
  assert(run(server,0x1004).data.size()==4);
  assert(run(server,0x1005,{server.storageId}).code==0x2013);
  std::cout<<"Read-only MTP contracts passed\n";
}
