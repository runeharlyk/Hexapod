#pragma once

// Host stand-in for firmware/include/event_bus.h with the same interface. On the robot each message type
// has its own queue and worker task; here a publish only queues, and host_bus::drainAll() delivers
// everything queued, including what the deliveries publish in turn, on the calling thread. The queue
// depth is the real one, so a publish onto a full queue fails as it does on the robot. Every bus lives for the
// whole process, so host_bus::resetAll() empties them all, except the subscriptions consume() holds for the
// program's lifetime; a Handle from before the reset is inert afterwards.

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <type_traits>
#include <utility>
#include <vector>
#include <freertos/FreeRTOS.h>

enum class EmitMode { Latest, Batch };

namespace host_bus {

inline std::vector<bool (*)()> &drains() {
    static std::vector<bool (*)()> registered;
    return registered;
}

inline std::vector<void (*)()> &resets() {
    static std::vector<void (*)()> registered;
    return registered;
}

// Indexed, because a delivery that first touches a bus enrols it in drains() mid-loop.
inline void drainAll() {
    for (bool delivered = true; delivered;) {
        delivered = false;
        for (size_t i = 0; i < drains().size(); ++i) delivered |= drains()[i]();
    }
}

inline void resetAll() {
    for (void (*reset)() : resets()) reset();
}

}  // namespace host_bus

template <typename Msg, size_t QueueDepth = 10, size_t MaxSubs = 4, size_t BatchSize = 2>
class EventBus {
    struct Item {
        Msg payload;
        size_t exclude;
    };
    static constexpr size_t NO_EX = MaxSubs;
    using Callback = std::function<void(const Msg *, size_t)>;

    static void enrol() {
        static const bool registered =
            (host_bus::drains().push_back(&drain), host_bus::resets().push_back(&reset), true);
        (void)registered;
    }

    static std::deque<Item> &queue() {
        enrol();
        static std::deque<Item> items;
        return items;
    }

    static std::array<std::optional<Callback>, MaxSubs> &subs() {
        enrol();
        static std::array<std::optional<Callback>, MaxSubs> table;
        return table;
    }

    // Counts the resets that cleared a slot, so a Handle can tell its own subscription from a later one in it.
    static std::array<uint32_t, MaxSubs> &generations() {
        static std::array<uint32_t, MaxSubs> table{};
        return table;
    }

    static std::array<bool, MaxSubs> &permanent() {
        static std::array<bool, MaxSubs> table{};
        return table;
    }

    static std::optional<Msg> &latest() {
        enrol();
        static std::optional<Msg> value;
        return value;
    }

    static void reset() {
        queue().clear();
        latest().reset();
        for (size_t i = 0; i < MaxSubs; ++i) {
            if (permanent()[i]) continue;
            subs()[i].reset();
            ++generations()[i];
        }
    }

    static bool drain() {
        auto &items = queue();
        if (items.empty()) return false;
        while (!items.empty()) {
            const Item item = items.front();
            items.pop_front();
            for (size_t i = 0; i < MaxSubs; ++i) {
                if (i != item.exclude && subs()[i]) (*subs()[i])(&item.payload, 1);
            }
        }
        return true;
    }

    static bool push(const Msg &m, size_t exclude) {
        auto &items = queue();
        if (items.size() >= QueueDepth) return false;
        items.push_back({m, exclude});
        return true;
    }

  public:
    class Handle {
        size_t idx{NO_EX};
        uint32_t generation{0};
        friend class EventBus;
        Handle(size_t i, uint32_t g) : idx(i), generation(g) {}

      public:
        Handle() = default;
        Handle(const Handle &) = delete;
        Handle &operator=(const Handle &) = delete;
        Handle(Handle &&o) noexcept : idx(o.idx), generation(o.generation) { o.idx = NO_EX; }
        Handle &operator=(Handle &&o) noexcept {
            if (this != &o) {
                unsubscribe();
                idx = o.idx;
                generation = o.generation;
                o.idx = NO_EX;
            }
            return *this;
        }
        ~Handle() { unsubscribe(); }
        void unsubscribe() {
            if (valid()) subs()[idx].reset();
            idx = NO_EX;
        }
        bool valid() const { return idx < MaxSubs && generations()[idx] == generation; }
    };

    static void store(const Msg &m) { latest() = m; }

    template <typename C>
    static void consume(C fn) {
        static Handle h = subscribe(std::forward<C>(fn));
        if (h.valid()) permanent()[h.idx] = true;
    }

    // Rate limits are not modelled: every message is delivered on the next drain.
    template <typename C>
    static Handle subscribe(uint32_t, EmitMode, C fn) {
        for (size_t i = 0; i < MaxSubs; ++i) {
            if (!subs()[i]) {
                subs()[i].emplace(std::move(fn));
                return Handle(i, generations()[i]);
            }
        }
        return Handle(NO_EX, 0);
    }

    template <typename C>
    static Handle subscribe(C fn) {
        if constexpr (std::is_invocable_v<C, const Msg *, size_t>)
            return subscribe(0, EmitMode::Latest, std::move(fn));
        else
            return subscribe(0, EmitMode::Latest, [fn = std::move(fn)](const Msg *p, size_t n) {
                for (size_t i = 0; i < n; ++i) fn(p[i]);
            });
    }

    template <typename C>
    static Handle subscribe(uint32_t ms, C fn) {
        if constexpr (std::is_invocable_v<C, const Msg *, size_t>)
            return subscribe(ms, EmitMode::Batch, std::move(fn));
        else
            return subscribe(ms, EmitMode::Batch, [fn = std::move(fn)](const Msg *p, size_t n) {
                for (size_t i = 0; i < n; ++i) fn(p[i]);
            });
    }

    static bool publish(const Msg &m) {
        store(m);
        return push(m, NO_EX);
    }

    static bool publish(const Msg &m, const Handle &h) {
        store(m);
        return push(m, h.valid() ? h.idx : NO_EX);
    }

    static bool peek(Msg &out) {
        if (!latest()) return false;
        out = *latest();
        return true;
    }

    static bool hasSubscribers() {
        for (const auto &s : subs())
            if (s) return true;
        return false;
    }
};
