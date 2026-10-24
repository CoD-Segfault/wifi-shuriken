#pragma once
#include <cassert>
struct mutex_t { bool locked = false; };
inline void mutex_init(mutex_t* p) { p->locked = false; }
inline bool mutex_try_enter(mutex_t* p, void*) { if (p->locked) return false; p->locked = true; return true; }
inline void mutex_enter_blocking(mutex_t* p) { assert(!p->locked); p->locked = true; }
inline void mutex_exit(mutex_t* p) { assert(p->locked); p->locked = false; }
