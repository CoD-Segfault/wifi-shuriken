#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <string>
constexpr int O_RDONLY = 0;
struct FakeFile { std::string name; uint64_t size; bool directory = false; uint16_t date = 0x5d58, time = 0x6021; };
inline std::vector<FakeFile> fakeFiles;
inline bool fakeReadFails = false;
inline unsigned fakeReads = 0;
class FsFile {
 bool opened_ = false, root_ = false;
 uint32_t index_ = 0, next_ = 0;
 uint64_t position_ = 0;
 public:
 FsFile() = default;
 explicit FsFile(bool root) : opened_(true), root_(root) {}
 explicit operator bool() const { return opened_; }
 bool open(FsFile*, uint32_t index, int) {
   opened_ = index < fakeFiles.size(); root_ = false; index_ = index; position_ = 0; return opened_;
 }
 bool openNext(FsFile* root, int flags) { return open(root, root->next_++, flags); }
 void close() { opened_ = false; }
 void flush() {}
 size_t getName(char* out, size_t length) { return snprintf(out, length, "%s", fakeFiles.at(index_).name.c_str()); }
 bool isDir() const { return root_ || fakeFiles.at(index_).directory; }
 uint64_t fileSize() const { return root_ ? 0 : fakeFiles.at(index_).size; }
 uint32_t dirIndex() const { return index_; }
 bool seekSet(uint64_t offset) { position_ = offset; return offset <= fileSize(); }
 bool getCreateDateTime(uint16_t* date, uint16_t* time) { *date = fakeFiles.at(index_).date; *time = fakeFiles.at(index_).time; return *date != 0; }
 bool getModifyDateTime(uint16_t* date, uint16_t* time) { return getCreateDateTime(date, time); }
 int read(uint8_t* out, uint32_t length) {
   ++fakeReads;
   if (fakeReadFails) return -1;
   for (uint32_t i = 0; i < length; ++i) out[i] = static_cast<uint8_t>(position_ + i);
   position_ += length; return length;
 }
};
struct FakeCard { uint64_t sectorCount() { return 8000000; } };
class SdFat {
 FakeCard card_;
 public:
 FsFile open(const char*, int) { return FsFile(true); }
 FakeCard* card() { return &card_; }
};
