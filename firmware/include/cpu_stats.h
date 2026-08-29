#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>

// Per-core CPU usage from the idle-task run-time counters: usage = 100 * (1 - idle_delta / elapsed).
// Requires CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS + CONFIG_FREERTOS_USE_TRACE_FACILITY.
class CpuMonitor {
  public:
    struct Usage {
        float core0;
        float core1;
        float total;
    };

    // Recomputes the window. The baselines below are not shared state -- exactly one task may call
    // this; every other reader takes latest().
    Usage sample() {
        Usage usage{0.f, 0.f, 0.f};
        int64_t now = esp_timer_get_time();
        uint32_t idle0 = idleRuntime(0);
        uint32_t idle1 = idleRuntime(1);

        if (last_ != 0) {
            int64_t elapsed = now - last_;
            if (elapsed > 0) {
                usage.core0 = coreUsage(idle0 - lastIdle0_, elapsed);
                usage.core1 = coreUsage(idle1 - lastIdle1_, elapsed);
                usage.total = 0.5f * (usage.core0 + usage.core1);
            }
        }

        last_ = now;
        lastIdle0_ = idle0;
        lastIdle1_ = idle1;

        portENTER_CRITICAL(&lock_);
        latest_ = usage;
        portEXIT_CRITICAL(&lock_);
        return usage;
    }

    Usage latest() {
        portENTER_CRITICAL(&lock_);
        Usage usage = latest_;
        portEXIT_CRITICAL(&lock_);
        return usage;
    }

  private:
    // uint32 truncation is safe: a window's delta is far below 2^32 us.
    static uint32_t idleRuntime(BaseType_t core) {
        TaskHandle_t idle = xTaskGetIdleTaskHandleForCore(core);
        if (!idle) return 0;
        TaskStatus_t status;
        vTaskGetInfo(idle, &status, pdFALSE, eInvalid);
        return (uint32_t)status.ulRunTimeCounter;
    }

    static float coreUsage(uint32_t idleDelta, int64_t elapsedUs) {
        float busy = 100.f * (1.f - (float)idleDelta / (float)elapsedUs);
        if (busy < 0.f) return 0.f;
        if (busy > 100.f) return 100.f;
        return busy;
    }

    int64_t last_ = 0;
    uint32_t lastIdle0_ = 0;
    uint32_t lastIdle1_ = 0;

    Usage latest_{0.f, 0.f, 0.f};
    portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
};
