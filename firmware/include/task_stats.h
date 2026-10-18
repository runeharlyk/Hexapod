#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <mutex>
#include <new>

#include <platform_shared/message.pb.h>
#include <task_table.h>

#if !configTASKLIST_INCLUDE_COREID
#error "the task table needs each task's core in its snapshot: CONFIG_FREERTOS_VTASKLIST_INCLUDE_COREID"
#endif
static_assert(configMAX_TASK_NAME_LEN <= task_table::NAME_SIZE);
static_assert(sizeof(socket_message_TaskEntry::name) == task_table::NAME_SIZE);
// TaskState is eTaskState's positions, so a state crosses as it is.
static_assert(int(eRunning) == socket_message_TaskState_TASK_RUNNING &&
              int(eReady) == socket_message_TaskState_TASK_READY &&
              int(eBlocked) == socket_message_TaskState_TASK_BLOCKED &&
              int(eSuspended) == socket_message_TaskState_TASK_SUSPENDED &&
              int(eDeleted) == socket_message_TaskState_TASK_DELETED);

/**
 * Answers the metrics page's TaskStatsRequest from one uxTaskGetSystemState snapshot, the CPU shares measured against
 * the previous snapshot's (task_table.h). A snapshot is taken at most once per task_table::MIN_SNAPSHOT_INTERVAL_US;
 * a request in between gets the last reply. The transports' tasks take requests concurrently and share the one
 * baseline, so a reply runs under a lock; the control task never answers one. The snapshot holds the kernel's lock,
 * interrupts masked on the calling core, while it reads every task's untouched stack bytes, and the reply says for
 * how long.
 */
class TaskStats {
  public:
    static constexpr size_t CAPACITY = 64; // tasks a snapshot holds; some 35 run

    /** Fills the reply; returns its status: 200, 503 without the PSRAM buffers, 507 with more tasks than CAPACITY. */
    uint32_t fill(socket_message_TaskStats& reply) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!buffers_) buffers_ = allocate();
        if (!buffers_) return 503;
        Buffers& b = *buffers_;
        const uint64_t now = esp_timer_get_time();
        return b.last.serve(now, reply, [&](socket_message_TaskStats& r) { return snapshot(b, now, r); });
    }

  private:
    struct Buffers {
        TaskStatus_t statuses[CAPACITY];
        task_table::Task tasks[CAPACITY];
        task_table::CpuWindow<CAPACITY> window;
        task_table::SnapshotCache<socket_message_TaskStats> last;
    };

    static uint32_t snapshot(Buffers& b, uint64_t before, socket_message_TaskStats& reply) {
        configRUN_TIME_COUNTER_TYPE takenAt = 0; // the counters' clock, the esp_timer's low word, read under the lock
        const UBaseType_t count = uxTaskGetSystemState(b.statuses, CAPACITY, &takenAt);
        if (count == 0) {
            reply.task_count = uxTaskGetNumberOfTasks();
            return 507;
        }
        // A status names its task by a pointer into the task's memory, which is read here, after the lock is
        // released: a task deleted and freed in between gives one garbled name, never a fault, as vTaskList's does.
        for (UBaseType_t i = 0; i < count; i++) b.tasks[i] = taskOf(b.statuses[i]);

        const uint32_t snapshotUs = takenAt - static_cast<uint32_t>(before);
        reply.task_count = count;
        reply.snapshot_us = snapshotUs;
        reply.window_us = b.window.measure(b.tasks, count, before + snapshotUs);
        reply.tasks_count = task_table::tightestFirst(b.tasks, count, std::size(reply.tasks));
        for (pb_size_t i = 0; i < reply.tasks_count; i++) reply.tasks[i] = entryOf(b.tasks[i]);
        return 200;
    }

    // Some 7 KB, in PSRAM: internal RAM is the robot's binding limit (docs/firmware.md, RAM budget).
    static Buffers* allocate() {
        void* memory = heap_caps_calloc(1, sizeof(Buffers), MALLOC_CAP_SPIRAM);
        return memory ? new (memory) Buffers() : nullptr;
    }

    static task_table::Task taskOf(const TaskStatus_t& status) {
        task_table::Task task {};
        task.number = status.xTaskNumber;
        std::strncpy(task.name, status.pcTaskName, sizeof(task.name) - 1);
        task.state = static_cast<uint8_t>(status.eCurrentState);
        task.priority = static_cast<uint8_t>(status.uxCurrentPriority);
        task.basePriority = static_cast<uint8_t>(status.uxBasePriority);
        task.core = status.xCoreID == tskNO_AFFINITY ? -1 : static_cast<int8_t>(status.xCoreID);
        task.stackFreeMin = status.usStackHighWaterMark; // bytes: StackType_t is a byte on the ESP32 ports
        task.runTime = status.ulRunTimeCounter;
        return task;
    }

    static socket_message_TaskEntry entryOf(const task_table::Task& task) {
        socket_message_TaskEntry entry = socket_message_TaskEntry_init_zero;
        std::memcpy(entry.name, task.name, sizeof(entry.name));
        entry.number = task.number;
        entry.state = static_cast<socket_message_TaskState>(task.state);
        entry.priority = task.priority;
        entry.base_priority = task.basePriority;
        entry.core = task.core;
        entry.stack_free_min = task.stackFreeMin;
        entry.cpu_percent = task.cpuPercent;
        return entry;
    }

    std::mutex mutex_;
    Buffers* buffers_ = nullptr;
};
