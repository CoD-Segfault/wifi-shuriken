#pragma once
#include <cstdint>
struct Sd { uint64_t cardSize() const {return 12345678;} };
inline Sd SD_MMC;
