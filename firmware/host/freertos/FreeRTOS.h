#pragma once

// Host stand-in for the FreeRTOS types the motion code names. The host builds run the robot's tasks on one thread, so
// a task never needs to yield; a semaphore still excludes (freertos/semphr.h) for a test that runs two.

#include <cstdint>

using BaseType_t = int;
using UBaseType_t = unsigned;
using TickType_t = uint32_t;

constexpr BaseType_t pdTRUE = 1;
constexpr BaseType_t pdFALSE = 0;
constexpr BaseType_t pdPASS = 1;
constexpr TickType_t portMAX_DELAY = UINT32_MAX;

#define pdMS_TO_TICKS(ms) static_cast<TickType_t>(ms)

using portMUX_TYPE = int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(mux) ((void)(mux))
#define portEXIT_CRITICAL(mux) ((void)(mux))
