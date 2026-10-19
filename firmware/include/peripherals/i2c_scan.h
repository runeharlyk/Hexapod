#pragma once

// A sweep of the I2C bus for the addresses that acknowledge, run on the control task. The bus's interrupt is bound to
// the core that opened the bus, the control task's, and in IDF 5.4.2 a probe from the other core can take a stale
// completion for its own and report a device that is not there (docs/firmware.md, I2C scan). So a requester posts
// the sweep and waits, and the control task probes a few addresses after each tick's work: the sweep never holds the
// bus across a tick, and the control task never waits on the requester.

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <peripherals/i2c_bus.h>

class I2CScan {
  public:
    // 0x00-0x07 and 0x78-0x7F are reserved by the I2C specification.
    static constexpr uint8_t FIRST = 0x08;
    static constexpr uint8_t LAST = 0x77;
    static constexpr size_t RANGE = LAST - FIRST + 1;
    // The driver probes at 100 kHz, about 0.1 ms an address: by estimate 1 ms of a 5 ms tick, and 14 ticks a sweep.
    static constexpr int PROBES_PER_TICK = 8;

    enum class Outcome { DONE, ALREADY_RUNNING, BUS_BUSY, TIMED_OUT };

    struct Found {
        size_t count = 0;
        uint8_t addresses[RANGE] = {};
    };

    /**
     * Posts a sweep and waits for it, calling `wait` up to `waits` times while the control task sweeps; one sweep runs
     * at a time. `found` holds the addresses that answered, in order, when the outcome is DONE, and none otherwise. A
     * sweep that has not ended after the last wait is abandoned, and the control task drops it at its next tick.
     */
    template <typename Wait>
    Outcome request(Found& found, Wait&& wait, int waits) {
        found = Found {};
        State idle = State::IDLE;
        if (!state_.compare_exchange_strong(idle, State::CLAIMED)) return Outcome::ALREADY_RUNNING;
        next_ = FIRST;
        found_ = Found {};
        state_.store(State::RUNNING, std::memory_order_release);
        for (int i = 0; i < waits; i++) {
            wait();
            if (ended()) return collect(found);
        }
        State running = State::RUNNING;
        if (state_.compare_exchange_strong(running, State::ABANDONED)) return Outcome::TIMED_OUT;
        return collect(found);
    }

    // The control task, once a tick: probes up to `budget` addresses of a posted sweep, and nothing without one.
    void step(int budget = PROBES_PER_TICK) {
        const State state = state_.load(std::memory_order_acquire);
        if (state == State::ABANDONED) state_.store(State::IDLE, std::memory_order_release);
        if (state != State::RUNNING) return;
        for (int i = 0; i < budget; i++) {
            switch (I2CBus::instance().tryProbe(next_)) {
                case i2c_health::Probe::BUSY: return end(State::BUS_BUSY);
                case i2c_health::Probe::PRESENT: found_.addresses[found_.count++] = next_; break;
                case i2c_health::Probe::ABSENT: break;
            }
            if (next_++ == LAST) return end(State::DONE);
        }
    }

  private:
    // CLAIMED keeps a second requester out while the first resets the sweep; only the control task leaves ABANDONED.
    enum class State : uint8_t { IDLE, CLAIMED, RUNNING, DONE, BUS_BUSY, ABANDONED };

    bool ended() const {
        const State state = state_.load(std::memory_order_acquire);
        return state == State::DONE || state == State::BUS_BUSY;
    }

    Outcome collect(Found& found) {
        const bool done = state_.load(std::memory_order_acquire) == State::DONE;
        if (done) found = found_;
        state_.store(State::IDLE, std::memory_order_release);
        return done ? Outcome::DONE : Outcome::BUS_BUSY;
    }

    // A requester that gave up has left the sweep ABANDONED; it reads nothing more, so the scan is free again.
    void end(State outcome) {
        State running = State::RUNNING;
        if (!state_.compare_exchange_strong(running, outcome, std::memory_order_acq_rel))
            state_.store(State::IDLE, std::memory_order_release);
    }

    std::atomic<State> state_ {State::IDLE};
    uint8_t next_ = FIRST;
    Found found_;
};
