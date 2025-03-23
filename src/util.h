#pragma once

#include <cstdint>

// time since boot (rounded)
uint64_t gettime_ns();
uint64_t gettime_us();
uint64_t gettime_ms();
uint32_t gettime_s();
