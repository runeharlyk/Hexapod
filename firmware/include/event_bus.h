#pragma once
#include <ArduinoJson.h>
#include <vector>
#include <functional>
#include <type_traits>

static_assert(!(USE_JSON == 1 && USE_MSGPACK == 1), "Cannot set both USE_JSON and USE_MSGPACK to 1 simultaneously");

class EventBus {
    template <typename Msg>
    struct Sub {
        std::function<void(const Msg&)> cb;
        TickType_t interval {0};
        TickType_t last {0};
        Msg pending;
        TimerHandle_t timer {nullptr};
    };

    template <typename Msg>
    static std::vector<Sub<Msg>*>& subs() {
        static std::vector<Sub<Msg>*> v;
        static bool init = [] {
            v.reserve(4);
            return true;
        }();
        return v;
    }

    template <typename Msg>
    static SemaphoreHandle_t& mtx() {
        static SemaphoreHandle_t m = xSemaphoreCreateMutex();
        return m;
    }

    template <typename Msg>
    static void timerCb(TimerHandle_t t) {
        auto s = static_cast<Sub<Msg>*>(pvTimerGetTimerID(t));
        TickType_t now = xTaskGetTickCount();
        s->cb(s->pending);
        s->last = now;
    }

  public:
    template <typename Msg, typename C>
    static void* subscribe(uint32_t ms, C&& cb) {
        auto* s = new Sub<Msg> {std::forward<C>(cb), pdMS_TO_TICKS(ms), 0, Msg {}, nullptr};
        if (ms > 0) {
            s->timer = xTimerCreate("t", s->interval, pdFALSE, s, timerCb<Msg>);
        }
        auto& v = subs<Msg>();
        xSemaphoreTake(mtx<Msg>(), portMAX_DELAY);
        v.push_back(s);
        xSemaphoreGive(mtx<Msg>());
        return static_cast<void*>(s);
    }

    template <typename Msg, typename C>
    static void* subscribe(C&& c) {
        return subscribe<Msg>(0, std::forward<C>(c));
    }

    template <typename Msg>
    static void unsubscribe(void* handle) {
        auto* s = static_cast<Sub<Msg>*>(handle);
        if (!s) return;
        auto& v = subs<Msg>();
        xSemaphoreTake(mtx<Msg>(), portMAX_DELAY);
        v.erase(std::remove(v.begin(), v.end(), s), v.end());
        xSemaphoreGive(mtx<Msg>());
        if (s->timer) xTimerDelete(s->timer, portMAX_DELAY);
        delete s;
    }

    template <typename Msg>
    static void publish(const Msg& msg, void* excludeHandle = nullptr) {
        TickType_t now = xTaskGetTickCount();
        auto* ex = static_cast<Sub<Msg>*>(excludeHandle);

        auto& v = subs<Msg>();
        auto& lock = mtx<Msg>();
        std::vector<std::function<void(const Msg&)>> ready;

        xSemaphoreTake(lock, portMAX_DELAY);
        for (auto* s : v) {
            if (s == ex) continue;
            TickType_t elapsed = now - s->last;
            if (s->interval == 0 || elapsed >= s->interval) {
                ready.push_back(s->cb);
                s->last = now;
            } else {
                s->pending = msg;
                if (!xTimerIsTimerActive(s->timer)) {
                    xTimerChangePeriod(s->timer, s->interval - elapsed, portMAX_DELAY);
                    xTimerStart(s->timer, portMAX_DELAY);
                }
            }
        }
        xSemaphoreGive(lock);

        for (auto& fn : ready) fn(msg);
    }
};