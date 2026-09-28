#pragma once

#include "esp_timer.h"

#define CONCAT(a, b) a##b

#define UNIQUE_VAR(base) CONCAT(base, __LINE__)

#define EXECUTE_EVERY_N_MS(n, code)                                                               \
    do {                                                                                          \
        static volatile uint64_t UNIQUE_VAR(lastExecution_) = 0;                                  \
        uint64_t currentMillis = esp_timer_get_time() / 1000;                                     \
        if (UNIQUE_VAR(lastExecution_) == 0 || currentMillis - UNIQUE_VAR(lastExecution_) >= n) { \
            code;                                                                                 \
            UNIQUE_VAR(lastExecution_) = currentMillis;                                           \
        }                                                                                         \
    } while (0)
