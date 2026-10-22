#pragma once
#include "hal/mtp_storage.h"
#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

namespace controller::mtp {
class PosixStorage : public Storage {
 public:
  explicit PosixStorage(std::string root) : root_(std::move(root)) {}
  ~PosixStorage() override { close(); }
  bool ready() const override { struct stat info{}; return ::stat(root_.c_str(),&info)==0 && S_ISDIR(info.st_mode); }
  uint64_t capacity() const override { return 0; }
  void flush() override {}
  bool list(std::vector<Object>& objects) override {
    DIR* dir=opendir(root_.c_str());
    if(!dir) return false;
    bool ok=true;
    while(true) {
      errno=0; auto* entry=readdir(dir);
      if(!entry) { ok=errno==0; break; }
      Object object{entry->d_name,0,false};
      if (!validName(object.name) || !stat(object)) continue;
      if(objects.size()==1024) { ok=false; break; }
      objects.push_back(std::move(object));
    }
    closedir(dir); return ok;
  }
  bool stat(Object& object) override {
    if(!validName(object.name)) return false;
    struct stat info{};
#if defined(ESP_PLATFORM)
    // ESP VFS/FatFs has no symlinks and does not provide lstat().
    const int result=::stat(path(object).c_str(),&info);
#else
    const int result=::lstat(path(object).c_str(),&info);
#endif
    if(result!=0 || (!S_ISREG(info.st_mode) && !S_ISDIR(info.st_mode))) return false;
    // MTP's ordinary object size field cannot describe a file above 4 GiB.
    if(S_ISREG(info.st_mode) && (info.st_size<0 || uint64_t(info.st_size)>UINT32_MAX-12u)) return false;
    object.directory=S_ISDIR(info.st_mode); object.size=object.directory?0:static_cast<uint32_t>(info.st_size);
    object.modified=static_cast<int64_t>(info.st_mtime);
    return true;
  }
  bool open(const Object& object) override {
    close(); Object checked=object;
    if(!stat(checked) || checked.directory) return false;
    file_=fopen(path(object).c_str(),"rb"); return file_!=nullptr;
  }
  bool read(uint32_t offset, uint8_t* data, size_t length) override {
    return file_ && fseeko(file_,offset,SEEK_SET)==0 && fread(data,1,length,file_)==length;
  }
  void close() override { if(file_) fclose(file_); file_=nullptr; }
 private:
  static bool validName(const std::string& name) { return !name.empty() && name!="." && name!=".." && name.find('/')==std::string::npos && name.find('\\')==std::string::npos; }
  std::string path(const Object& object) const { return root_+"/"+object.name; }
  std::string root_;
  FILE* file_=nullptr;
};
}
