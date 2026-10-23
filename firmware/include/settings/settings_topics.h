#pragma once

// The settings groups as state topics (communication/topics.h). A subscribe's snapshot and the push after every stored
// change both read the group under its service's lock and redact it, as the replies do, then send it. Passwords and the
// merge fingerprint never leave the robot.
//
// Locks, in the order taken, so that no two paths wait on each other:
// - snapshot: the adapter's encodeMutex_ (held by CommAdapterBase::subscribe), the snapshot scratch, the service lock;
// - push: the push scratch, the service lock, released before the emit takes an adapter's encodeMutex_.
// A push runs as an update handler, after StatefulService::update has unlocked, on the updating task, which may be a
// transport task inside its correlation handler: no adapter lock is held there.
//
// Each group is held twice in PSRAM, for the snapshots and for the pushes, never on a task's stack: a correlation
// handler already holds a ~2 KB response on its transport task's stack, and internal RAM is the binding limit.

#include <esp_heap_caps.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <cstring>
#include <string>
#include <communication/comm_base.hpp>
#include <platform_shared/api.pb.h>
#include <template/stateful_service.h>

namespace settings_topics {

// The groups without a secret go out as stored.
template <typename T>
void redact(T&) {}

// The update paths treat an empty password as "keep the stored one" (WiFiSettings_update, APSettings_update), so a
// client can send a group back unchanged. The WiFi fingerprint is a hash of the secrets.h password.
inline void redact(api_WifiSettings& settings) {
    settings.applied_secret = 0;
    for (pb_size_t i = 0; i < settings.wifi_networks_count; i++)
        memset(settings.wifi_networks[i].password, 0, sizeof(settings.wifi_networks[i].password));
}

inline void redact(api_APSettings& settings) { memset(settings.password, 0, sizeof(settings.password)); }

// Written into `out` rather than returned: a correlation handler fills its ~2 KB response in place, and a temporary
// copy on its transport task's stack would push it past the stack budget.
template <typename T>
void readRedacted(StatefulService<T>& service, T& out) {
    service.read([&out](const T& state) { out = state; });
    redact(out);
}

// One group's copy in PSRAM and the lock that serialises its users, allocated on first use and kept.
template <typename T>
class Scratch {
  public:
    // Runs `use` on the copy under its lock; false, without running it, when PSRAM has no room for the copy.
    template <typename Use>
    bool with(Use use) {
        xSemaphoreTake(mutex_, portMAX_DELAY);
        if (!value_) value_ = static_cast<T*>(heap_caps_malloc(sizeof(T), MALLOC_CAP_SPIRAM));
        const bool held = value_ != nullptr;
        if (held) use(*value_);
        xSemaphoreGive(mutex_);
        if (!held) ESP_LOGE("SettingsTopics", "No PSRAM for a %u byte settings copy", (unsigned)sizeof(T));
        return held;
    }

  private:
    SemaphoreHandle_t mutex_ = xSemaphoreCreateMutex();
    T* value_ = nullptr;
};

template <typename T>
Scratch<T>& snapshotScratch() {
    static Scratch<T> scratch;
    return scratch;
}

template <typename T>
Scratch<T>& pushScratch() {
    static Scratch<T> scratch;
    return scratch;
}

// A snapshot without room for its copy puts nothing: the subscribe is left unacked and resent (CommAdapterBase).
template <typename T>
void snapshot(StatefulService<T>& service, SnapshotSink& sink) {
    snapshotScratch<T>().with([&](T& value) {
        readRedacted(service, value);
        sink.put(value);
    });
}

// Pushes the stored group with `emit` after every update that changed it, from every update path; an unchanged or
// refused update pushes nothing, and the reply settles the write.
template <typename T, typename Emit>
HandlerId pushOnChange(StatefulService<T>& service, Emit emit) {
    return service.addUpdateHandler([&service, emit](const std::string&) {
        pushScratch<T>().with([&](T& value) {
            readRedacted(service, value);
            emit(static_cast<const T&>(value));
        });
    });
}

} // namespace settings_topics
