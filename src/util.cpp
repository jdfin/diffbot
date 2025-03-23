
#include <time.h>
#include <cstdint>
#include "util.h"


uint64_t gettime_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000000ull + ts.tv_nsec;
}


uint64_t gettime_us()
{
    return (gettime_ns() + 500) / 1000;
}


uint64_t gettime_ms()
{
    return (gettime_us() + 500) / 1000;
}


uint32_t gettime_s()
{
    return (gettime_ms() + 500) / 1000;
}
