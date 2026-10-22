#pragma once
#include <array>
#include <cassert>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>
namespace fake_usb {
inline bool mounted = true;
inline std::array<bool, 2> registered{}, dtr{}, reboot{{true, true}};
inline std::array<unsigned, 2> sdk_begins{}, flushes{}, reads{}, writes{};
inline std::array<size_t, 2> room{{512,512}};
inline std::array<std::deque<uint8_t>, 2> rx;
inline std::array<std::string, 2> tx;
inline void feed(uint8_t interface, const std::string& text) {
  assert(interface < 2 && rx[interface].size() + text.size() <= 512);
  for (const auto byte : text) rx[interface].push_back(static_cast<uint8_t>(byte));
}
}
class USBCDC {
 public:
  explicit USBCDC(uint8_t interface = 0) : interface_(interface) {
    assert(interface < 2); fake_usb::registered[interface] = true;
  }
  void enableReboot(bool value) { fake_usb::reboot[interface_] = value; }
  void begin(unsigned long = 0) { ++fake_usb::sdk_begins[interface_]; }
 private:
  uint8_t interface_;
};
