#pragma once

// The host builds' stand-in for the ESP32's microsecond timer: time only moves when the caller
// advances it, so a host run is deterministic and can run faster or slower than real time.

#include <cstdint>

namespace host_clock {

inline int64_t &nowUs() {
    static int64_t us = 1;
    return us;
}

inline void advance(int64_t us) { nowUs() += us; }

}  // namespace host_clock
