#pragma once

// Mutexes that exclude as the robot's do, so a host test can run two tasks on two threads against one lock
// (test/test_topic_snapshot). A wait is in milliseconds, one tick being one millisecond here as on the robot.

#include <chrono>
#include <mutex>
#include <freertos/FreeRTOS.h>

struct HostSemaphore {
    std::timed_mutex plain;
    std::recursive_timed_mutex nested;
};

using SemaphoreHandle_t = HostSemaphore *;

inline SemaphoreHandle_t xSemaphoreCreateMutex() { return new HostSemaphore; }
inline SemaphoreHandle_t xSemaphoreCreateRecursiveMutex() { return new HostSemaphore; }

template <typename Mutex>
inline BaseType_t hostSemaphoreTake(Mutex &mutex, TickType_t ticks) {
    if (ticks == portMAX_DELAY) {
        mutex.lock();
        return pdTRUE;
    }
    return mutex.try_lock_for(std::chrono::milliseconds(ticks)) ? pdTRUE : pdFALSE;
}

inline BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t ticks) { return hostSemaphoreTake(s->plain, ticks); }
inline BaseType_t xSemaphoreGive(SemaphoreHandle_t s) {
    s->plain.unlock();
    return pdTRUE;
}
inline BaseType_t xSemaphoreTakeRecursive(SemaphoreHandle_t s, TickType_t ticks) {
    return hostSemaphoreTake(s->nested, ticks);
}
inline BaseType_t xSemaphoreGiveRecursive(SemaphoreHandle_t s) {
    s->nested.unlock();
    return pdTRUE;
}
inline void vSemaphoreDelete(SemaphoreHandle_t s) { delete s; }
