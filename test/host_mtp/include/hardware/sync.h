#pragma once
#include <stdint.h>
inline uint32_t save_and_disable_interrupts() { return 0; }
inline void restore_interrupts(uint32_t) {}
inline void __compiler_memory_barrier() { asm volatile("" ::: "memory"); }
