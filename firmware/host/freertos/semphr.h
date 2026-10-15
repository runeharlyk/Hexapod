#pragma once

#include <freertos/FreeRTOS.h>

using SemaphoreHandle_t = void *;

inline SemaphoreHandle_t xSemaphoreCreateMutex() {
    static int token;
    return &token;
}
inline BaseType_t xSemaphoreTake(SemaphoreHandle_t, TickType_t) { return pdTRUE; }
inline BaseType_t xSemaphoreGive(SemaphoreHandle_t) { return pdTRUE; }
