#pragma once

// Host stand-in for ESP-IDF's esp_timer.h, read from host_clock.

#include <host_clock.h>

inline int64_t esp_timer_get_time() { return host_clock::nowUs(); }
