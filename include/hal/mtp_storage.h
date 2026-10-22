#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace controller::mtp {
struct Object { std::string name; uint32_t size = 0; bool directory = false; int64_t modified = 0; };
// Read-only object access. Implementations own filesystem paths and file handles.
class Storage {
 public:
  virtual ~Storage() = default;
  virtual bool ready() const = 0;
  virtual uint64_t capacity() const = 0;
  virtual void flush() = 0;
  virtual bool list(std::vector<Object>& objects) = 0;
  virtual bool stat(Object& object) = 0;
  virtual bool open(const Object& object) = 0;
  virtual bool read(uint32_t offset, uint8_t* destination, size_t length) = 0;
  virtual void close() = 0;
};
}
