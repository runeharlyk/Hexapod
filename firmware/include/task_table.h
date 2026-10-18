#pragma once

// The task table of the metrics page: each FreeRTOS task's least free stack, and its CPU share over the window
// between two snapshots. Free of FreeRTOS so the host tests run it; include/task_stats.h takes the snapshots.

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace task_table {

constexpr size_t NAME_SIZE = 16; // configMAX_TASK_NAME_LEN on the robot, the terminator included

/** One task as a snapshot found it. */
struct Task {
    uint32_t number; // FreeRTOS's task number, given to no other task since boot, where a handle's memory is reused
    char name[NAME_SIZE];
    uint8_t state;    // eTaskState
    uint8_t priority; // current: raised while the task holds a mutex that a higher one waits for
    uint8_t basePriority;
    int8_t core;           // the core it is pinned to, -1 for either
    uint32_t stackFreeMin; // bytes, the least free stack since the task started
    uint32_t runTime;      // us it has run since it started, modulo 2^32
    float cpuPercent;      // of one core over the window, set by CpuWindow::measure
};

/**
 * Each task's share of one core over the window since the previous snapshot. The run-time counters are 32-bit
 * microseconds on the esp_timer clock (CONFIG_FREERTOS_RUN_TIME_COUNTER_TYPE_U32), so they wrap every 71.6 minutes: a
 * difference modulo 2^32 is exact while the window is shorter than that, and a longer window has no figures.
 */
template <size_t Capacity>
class CpuWindow {
  public:
    static constexpr uint64_t COUNTER_SPAN_US = uint64_t(1) << 32;

    /**
     * Sets each task's cpuPercent from its run time since the previous call, or since it started for a task that call
     * did not see, and keeps this snapshot as the next one's baseline. Returns the window in us; 0 when there is none,
     * on the first call, on one 2^32 us or more after the previous or after a snapshot larger than Capacity, and every
     * cpuPercent is then 0.
     */
    uint32_t measure(Task* tasks, size_t count, uint64_t nowUs) {
        const uint64_t window = baselineCount_ > 0 ? nowUs - baselineAt_ : 0;
        const bool measurable = window > 0 && window < COUNTER_SPAN_US;
        for (size_t i = 0; i < count; i++) tasks[i].cpuPercent = measurable ? share(ranSince(tasks[i]), window) : 0.f;
        keep(tasks, count, nowUs);
        return measurable ? static_cast<uint32_t>(window) : 0;
    }

  private:
    struct Baseline {
        uint32_t number;
        uint32_t runTime;
    };

    // A task the baseline lacks started within the window, since the baseline held every task: its counter began at 0.
    uint32_t ranSince(const Task& task) const {
        for (size_t i = 0; i < baselineCount_; i++)
            if (baselines_[i].number == task.number) return task.runTime - baselines_[i].runTime;
        return task.runTime;
    }

    static float share(uint32_t ranUs, uint64_t windowUs) {
        return std::min(100.f, 100.f * static_cast<float>(ranUs) / static_cast<float>(windowUs));
    }

    void keep(const Task* tasks, size_t count, uint64_t nowUs) {
        baselineCount_ = count <= Capacity ? count : 0;
        for (size_t i = 0; i < baselineCount_; i++) baselines_[i] = {tasks[i].number, tasks[i].runTime};
        baselineAt_ = nowUs;
    }

    std::array<Baseline, Capacity> baselines_ {};
    size_t baselineCount_ = 0;
    uint64_t baselineAt_ = 0;
};

/**
 * Orders the tasks with the least stack to spare first, the ones closest to an overflow, ties by name and then by
 * number so two replies list them alike. Returns how many of them fit `capacity`: the roomiest are the ones cut.
 */
inline size_t tightestFirst(Task* tasks, size_t count, size_t capacity) {
    std::sort(tasks, tasks + count, [](const Task& a, const Task& b) {
        if (a.stackFreeMin != b.stackFreeMin) return a.stackFreeMin < b.stackFreeMin;
        if (const int byName = std::strcmp(a.name, b.name)) return byName < 0;
        return a.number < b.number;
    });
    return std::min(count, capacity);
}

} // namespace task_table
