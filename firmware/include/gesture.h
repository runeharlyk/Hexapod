#pragma once

// A hand moved over the PAJ7620U2 on top of the robot, facing up, turned into a goal move: a sweep walks 40 cm that
// way, a circle turns the robot around in its direction. Only a robot standing still takes one, so a gesture made
// while it walks, which the sensor riding on the body reads poorly, is ignored.

#include <cstdint>
#include <optional>
#include <utility>
#include <goal_controller.h>
#include <message_types.h>
#include <motion_inbox.h>
#include <utils/math_utils.h>

// Quarter turns clockwise, seen from above, from the robot's forward to the chip's UP sweep: how the sensor is mounted.
#ifndef GESTURE_SENSOR_QUARTERS
#define GESTURE_SENSOR_QUARTERS 0
#endif

namespace gesture {

// What the chip reports, in its own frame: a sweep across it, a circle, the hand coming toward it or going away, or
// a wave. Its sweeps and circles are named as the vendor's sample names the flags.
enum class Hand : uint8_t { NONE, UP, RIGHT, DOWN, LEFT, CLOCKWISE, COUNTER_CLOCKWISE, TOWARD, AWAY, WAVE };

// One gesture out of the flags register 0x44 over 0x43 holds (PAJ7620Driver::readFlags), the first in the vendor's
// order when the chip set several.
inline Hand decode(uint16_t flags) {
    if (flags & 0x100) return Hand::WAVE;
    constexpr Hand BITS[8] = {Hand::RIGHT,  Hand::LEFT, Hand::UP,        Hand::DOWN,
                              Hand::TOWARD, Hand::AWAY, Hand::CLOCKWISE, Hand::COUNTER_CLOCKWISE};
    for (int bit = 0; bit < 8; ++bit)
        if (flags & 1 << bit) return BITS[bit];
    return Hand::NONE;
}

/*
 * The vendor's entry and quit delays without their waits. A sweep or a circle is acted on CONFIRM_MS after it was
 * seen, unless the hand came toward the chip, went away from it or waved meanwhile: a hand reaching down to pick the
 * robot up may cross the chip first. After any of those, nothing is taken for QUIET_MS, while the hand leaves.
 */
class Filter {
  public:
    static constexpr unsigned long CONFIRM_MS = 800;
    static constexpr unsigned long QUIET_MS = 1000;

    // Every poll, with what the chip reported since the last (NONE included): the gesture confirmed now, if any.
    std::optional<Hand> update(Hand seen, unsigned long nowMs) {
        if (seen == Hand::TOWARD || seen == Hand::AWAY || seen == Hand::WAVE) {
            _pending.reset();
            _quiet = true;
            _quietSinceMs = nowMs;
            return std::nullopt;
        }
        if (_quiet && elapsed(_quietSinceMs, nowMs) < static_cast<long>(QUIET_MS)) return std::nullopt;
        _quiet = false;
        if (_pending && elapsed(_pendingSinceMs, nowMs) >= static_cast<long>(CONFIRM_MS))
            return std::exchange(_pending, std::nullopt);
        if (seen != Hand::NONE && !_pending) {
            _pending = seen;
            _pendingSinceMs = nowMs;
        }
        return std::nullopt;
    }

  private:
    std::optional<Hand> _pending;
    unsigned long _pendingSinceMs = 0;
    bool _quiet = false;
    unsigned long _quietSinceMs = 0;

    static long elapsed(unsigned long sinceMs, unsigned long nowMs) { return static_cast<long>(nowMs - sinceMs); }
};

constexpr float SWEEP_M = 0.4f;
constexpr float SPEED = 0.5f; // the facade's default share of the gait's top speed

// The move a gesture asks for: a sweep walks SWEEP_M the way the hand went, a circle turns half a turn its way. The
// rest ask for none.
inline std::optional<Goal> goalFor(Hand hand, int quarters = GESTURE_SENSOR_QUARTERS) {
    int sweep;
    switch (hand) {
        case Hand::UP: sweep = 0; break;
        case Hand::RIGHT: sweep = 1; break;
        case Hand::DOWN: sweep = 2; break;
        case Hand::LEFT: sweep = 3; break;
        case Hand::CLOCKWISE: return Goal {GoalKind::TURN, 0, -PI_F, 0, SPEED};
        case Hand::COUNTER_CLOCKWISE: return Goal {GoalKind::TURN, 0, PI_F, 0, SPEED};
        default: return std::nullopt;
    }
    switch ((sweep + quarters % 4 + 4) % 4) {
        case 0: return Goal {GoalKind::FORWARD, SWEEP_M, 0, 0, SPEED};
        case 1: return Goal {GoalKind::SIDEWAYS, SWEEP_M, 0, 0, SPEED};
        case 2: return Goal {GoalKind::FORWARD, -SWEEP_M, 0, 0, SPEED};
        default: return Goal {GoalKind::SIDEWAYS, -SWEEP_M, 0, 0, SPEED};
    }
}

/*
 * Starts the move a confirmed gesture asks for, as a goal no client owns, so no client's silence or departure ends
 * it; an operator's stick, a stop or the handheld does. It walks the tripod gait at the sliders' centre, the facade's
 * default goal. Returns 0 for a gesture that asks for no move, 409 when the robot is not standing still, and the
 * goal's reply otherwise (MotionService::startGoal). `robot` provides `still()` and `startGoal()` as Hexapod does.
 */
template <typename Robot>
int act(Hand hand, Robot &robot) {
    const std::optional<Goal> goal = goalFor(hand);
    if (!goal) return 0;
    if (!robot.still()) return 409;
    return robot.startGoal(*goal, CommandMsg {}, GaitType::TRI_GATE, GoalOwner {});
}

} // namespace gesture
