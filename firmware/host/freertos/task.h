#pragma once

#include <freertos/FreeRTOS.h>

using TaskHandle_t = void *;

inline UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t) { return 0; }
inline void taskYIELD() {}
