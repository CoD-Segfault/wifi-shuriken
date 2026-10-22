// This test intentionally includes no Arduino, SDK or controller runtime headers.
#include "posix/log_storage.h"
#include <cassert>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
static std::string read(const std::string& path) {
  std::ifstream file(path); return {std::istreambuf_iterator<char>(file), {}};
}
int main() {
  char root[] = "/tmp/shuriken-storage-XXXXXX";
  assert(mkdtemp(root));
  const auto original = std::filesystem::current_path();
  const std::string folder = std::string(root) + "/local logs";
  controller::storage::PosixLogStorage storage(folder.c_str());
  assert(!storage.mount());
  std::filesystem::create_directory(folder);
  assert(storage.recover());
  assert(!storage.exists("/append.csv"));
  auto first = storage.openAppend("/append.csv"); assert(first && *first && first->size() == 0);
  first->print("first\n"); assert(first->flush());
  auto second = storage.openAppend("/append.csv"); assert(second && second->size() == 6);
  second->print("second\n"); first->print("third\n");
  assert(first->flush() && second->flush());
  assert(read(folder + "/append.csv") == "first\nsecond\nthird\n");
  assert(first->close() && first->close() && !*first && !first->hasError());
  auto owner = std::move(second); assert(!second && owner && owner->size() == 19);
  owner.reset();
  auto reopened = storage.openAppend("/append.csv");
  assert(reopened && reopened->size() == 19 && reopened->close());
  assert(!storage.openAppend("relative.csv"));
  assert(!storage.openAppend("/../escape.csv"));
  assert(!storage.openAppend("/x/../../escape.csv"));
  assert(!storage.openAppend("/./escape.csv"));
  assert(!storage.openAppend(("/" + std::string(1100, 'x')).c_str()));
  assert(!storage.openAppend("/absent/file.csv"));
  assert(!storage.openAppend("/"));
  assert(storage.exists("/append.csv") && !storage.exists("/missing.csv"));
  assert(std::filesystem::current_path() == original);
  // Fsync errors, invalid handles and destructor ownership are real OS paths.
  controller::storage::PosixLogFile invalid;
  assert(invalid.print("x") == 0 && invalid.hasError() && !invalid.flush() && !invalid.close());
  controller::storage::PosixLogFile null(::open("/dev/null", O_WRONLY));
  assert(null.print("x") == 1 && !null.flush() && null.hasError() && !null.close());
  const int fd = ::open((folder + "/append.csv").c_str(), O_RDONLY);
  assert(fd >= 0);
  { controller::storage::PosixLogFile owned(fd); assert(owned.size() == 19); }
  assert(fcntl(fd, F_GETFD) == -1 && errno == EBADF);
  controller::storage::PosixLogFile readonly(::open((folder + "/append.csv").c_str(), O_RDONLY));
  assert(readonly.print("x") == 0 && readonly.hasError());
  controller::storage::PosixLogFile closedSize;
  assert(closedSize.size() == 0 && closedSize.hasError());
  std::filesystem::remove_all(root);
  puts("PASS: neutral storage/local directories/append/ownership/path errors/durability errors");
}
