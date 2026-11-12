#pragma once

#include <esp_sleep.h>
#include <esp_system.h>
#include <esp_timer.h>

#include <atomic>

#include <system_command.h>

// Restarts or sleeps a moment after a reply has gone out (a finished update, a system command), from a timer made at
// boot: a task made at the end of an update found no internal RAM for its stack with the camera running, so the robot
// kept running the old image and refused every later update as already running.
namespace after_reply {

inline esp_timer_handle_t timer = nullptr;
inline std::atomic<AfterReply> pending {AfterReply::NOTHING};

inline void act(AfterReply after) {
    if (after == AfterReply::RESTART) esp_restart();
    if (after == AfterReply::SLEEP) esp_deep_sleep_start();
}

inline void prepare() {
    esp_timer_create_args_t args = {};
    args.callback = [](void *) { act(pending.load()); };
    args.name = "after_reply";
    if (esp_timer_create(&args, &timer) != ESP_OK) timer = nullptr;
}

// Without the timer, or with it already running for an earlier reply, acts at once: the reply may then be lost, but
// the robot does what it was asked.
inline void schedule(AfterReply after) {
    if (after == AfterReply::NOTHING) return;
    pending = after;
    if (!timer || esp_timer_start_once(timer, 1500 * 1000) != ESP_OK) act(after);
}

} // namespace after_reply
