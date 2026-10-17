#pragma once

#include <odometry.h>
#include <algorithm>
#include <cmath>
#include <cstdint>

// Ported from SpotMicroESP32-Leika (esp32/include/goal_controller.h) to the body frame of docs/conventions.md, with
// heading hold, arcs, and corrective passes judged on the settled pose.

enum class GoalKind : uint8_t { FORWARD, SIDEWAYS, TURN, ARC };

// The values of the protocol's GoalState.
enum class GoalState : uint8_t { IDLE, SETTLING, RUNNING, REACHED, CANCELLED, TIMED_OUT, REFUSED };

/** A move the robot makes on its own odometry, in the frame it faced when the move started. */
struct Goal {
    GoalKind kind = GoalKind::FORWARD;
    float distance = 0; // m: FORWARD (negative is backwards) and SIDEWAYS (positive is right)
    float angle = 0;    // rad, counter-clockwise: TURN, and ARC (positive curves left)
    float radius = 0;   // m, ARC
    float speed = 0;    // share of the gait's top speed at the posture, in (0, 1]
};

/** Sticks in MotionService's convention: ly > 0 walks forward, lx > 0 left and rx > 0 turns counter-clockwise. */
struct GoalSticks {
    float lx = 0, ly = 0, rx = 0;
};

/**
 * Drives one goal to its target on the odometry, called every control tick. The remaining distance along the goal's
 * axis in the frame it started in, or the remaining turn on the unwrapped fused yaw, sets the driving stick: the asked
 * speed, slowed in proportion over the last SLOW_DOWN, never below MIN_FRACTION of it. It walks on until it is past
 * the target, centres its sticks and waits for the robot to come to rest, since the stop coasts on by an amount that
 * depends on the gait's phase. The settled pose is judged: within the tolerance it has REACHED; otherwise a corrective
 * pass walks the remainder, up to MAX_PASSES passes in all, and then it has TIMED_OUT. So has a pass that takes
 * longer than twice its expected time plus TIMEOUT_MARGIN_S.
 *
 * A straight walk holds the start heading with its turn stick. An arc walks forward with the turn stick its radius
 * implies, slows on its remaining angle and steers its heading toward the arc's heading at the distance travelled.
 */
class GoalController {
  public:
    static constexpr float SLOW_DOWN_M = 0.05f;
    static constexpr float SLOW_DOWN_RAD = 30.0f * static_cast<float>(M_PI) / 180.0f;
    static constexpr float MIN_FRACTION = 0.2f;
    static constexpr float REACHED_M = 0.01f;
    static constexpr float REACHED_RAD = 3.0f * static_cast<float>(M_PI) / 180.0f;
    static constexpr float ARC_REACHED_M = 0.03f;
    static constexpr int MAX_PASSES = 3;
    static constexpr float HEADING_GAIN = 2.0f; // turn stick per rad of heading error
    static constexpr float HEADING_STICK_MAX = 0.3f;
    static constexpr float TIMEOUT_MARGIN_S = 2.0f;

    /** A finite goal with a nonzero target, a positive radius for an arc, at a speed in (0, 1]. */
    static bool valid(const Goal& goal) {
        for (const float value : {goal.distance, goal.angle, goal.radius, goal.speed})
            if (!std::isfinite(value)) return false;
        if (!(goal.speed > 0 && goal.speed <= 1)) return false;
        switch (goal.kind) {
            case GoalKind::FORWARD:
            case GoalKind::SIDEWAYS: return goal.distance != 0;
            case GoalKind::TURN: return goal.angle != 0;
            case GoalKind::ARC: return goal.angle != 0 && goal.radius > 0;
        }
        return false;
    }

    /** Starts `goal` from `pose`; `topSpeed` (mm/s) and `topTurn` (rad/s) are what a full stick gives at the posture.
     */
    void start(const Goal& goal, const OdometryEstimate& pose, float topSpeed, float topTurn) {
        if (!valid(goal)) {
            current = GoalState::REFUSED;
            return;
        }
        target = goal;
        origin = pose;
        lastYaw = pose.yaw;
        turned = 0;
        remaining = aim();
        topSpeedMs = topSpeed / 1000.0f;
        topTurnRads = topTurn;
        passCount = 0;
        beginPass(pose);
    }

    /** One control tick: the odometry and whether the robot is at rest. Returns the sticks, centred unless running. */
    GoalSticks update(const OdometryEstimate& pose, bool atRest, float dt) {
        if (!active()) return {};
        // The odometry's yaw wraps at pi; a turn of any size is followed on the sum of its wrapped changes.
        turned += std::remainder(pose.yaw - lastYaw, 2.0f * static_cast<float>(M_PI));
        lastYaw = pose.yaw;
        remaining = aim() - covered(pose);
        elapsed += dt;
        if (elapsed > deadline) {
            current = GoalState::TIMED_OUT;
            return {};
        }
        if (current == GoalState::SETTLING) {
            if (!atRest) return {};
            if (within(pose)) {
                current = GoalState::REACHED;
                return {};
            }
            // An arc whose angle is reached but whose end is not has no remainder for a pass to walk.
            if (passCount == MAX_PASSES || std::fabs(remaining) <= axisTolerance()) {
                current = GoalState::TIMED_OUT;
                return {};
            }
            beginPass(pose);
        }
        if (remaining * direction <= 0) {
            current = GoalState::SETTLING;
            return {};
        }
        return sticks(pose);
    }

    void cancel() {
        if (active()) current = GoalState::CANCELLED;
    }

    GoalState state() const { return current; }
    /** The share of the target covered, from 0 to 1. */
    float progress() const {
        const bool started = current != GoalState::IDLE && current != GoalState::REFUSED;
        return started ? std::clamp(1 - remaining / aim(), 0.0f, 1.0f) : 0;
    }
    /** The passes started, the first included. */
    int passes() const { return passCount; }

  private:
    Goal target;
    GoalState current = GoalState::IDLE;
    OdometryEstimate origin;
    float lastYaw = 0, turned = 0; // the fused yaw turned since the start, unwrapped
    float remaining = 0;           // of the target, signed as it
    float topSpeedMs = 0, topTurnRads = 0;
    int passCount = 0;
    // The pass in progress: the sign of the remainder it walks, where it began and its clock.
    float direction = 0, passTurned = 0, passDistance = 0, elapsed = 0, deadline = 0;

    bool active() const { return current == GoalState::RUNNING || current == GoalState::SETTLING; }
    bool linear() const { return target.kind == GoalKind::FORWARD || target.kind == GoalKind::SIDEWAYS; }
    float aim() const { return linear() ? target.distance : target.angle; }
    float zone() const { return linear() ? SLOW_DOWN_M : SLOW_DOWN_RAD; }
    float axisTolerance() const { return linear() ? REACHED_M : REACHED_RAD; }
    float turnPerForward() const { return topSpeedMs / target.radius / topTurnRads; }

    // The pose relative to the start: x right and y forward of the heading it started on.
    float startRight(const OdometryEstimate& p) const {
        return std::cos(origin.yaw) * (p.x - origin.x) + std::sin(origin.yaw) * (p.y - origin.y);
    }
    float startForward(const OdometryEstimate& p) const {
        return -std::sin(origin.yaw) * (p.x - origin.x) + std::cos(origin.yaw) * (p.y - origin.y);
    }

    float covered(const OdometryEstimate& p) const {
        switch (target.kind) {
            case GoalKind::FORWARD: return startForward(p);
            case GoalKind::SIDEWAYS: return startRight(p);
            default: return turned;
        }
    }

    bool within(const OdometryEstimate& p) const {
        if (std::fabs(remaining) > axisTolerance()) return false;
        if (target.kind != GoalKind::ARC) return true;
        const float endRight = -std::copysign(target.radius * (1 - std::cos(target.angle)), target.angle);
        const float endForward = target.radius * std::sin(std::fabs(target.angle));
        return std::hypot(startRight(p) - endRight, startForward(p) - endForward) <= ARC_REACHED_M;
    }

    float headingStick(float error) const {
        return std::clamp(HEADING_GAIN * error, -HEADING_STICK_MAX, HEADING_STICK_MAX);
    }

    // An arc's sticks, the forward one slowed when the turn one would pass a full stick, so the curvature is kept.
    GoalSticks arcSticks(float forward, float turn) const {
        const float over = std::max(1.0f, std::fabs(turn));
        return {0, forward / over, turn / over};
    }

    // The rate the pass's axis moves at with a full share of the asked speed: m/s, or rad/s for a turn or an arc.
    float pace() const {
        switch (target.kind) {
            case GoalKind::TURN: return target.speed * topTurnRads;
            case GoalKind::ARC: {
                // MotionService shares a combined stick out over its L1 norm, which slows the turn with the walk.
                const GoalSticks full = arcSticks(target.speed, target.speed * turnPerForward());
                return full.rx / std::max(1.0f, full.ly + full.rx) * topTurnRads;
            }
            default: return target.speed * topSpeedMs;
        }
    }

    void beginPass(const OdometryEstimate& pose) {
        passCount++;
        direction = std::copysign(1.0f, remaining);
        passTurned = turned;
        passDistance = pose.distance;
        elapsed = 0;
        const float d = std::fabs(remaining);
        const float expected = (std::max(d - zone(), 0.0f) + std::min(d, zone()) / MIN_FRACTION) / pace();
        deadline = 2 * expected + TIMEOUT_MARGIN_S;
        current = GoalState::RUNNING;
    }

    GoalSticks sticks(const OdometryEstimate& pose) const {
        const float drive =
            std::copysign(target.speed * std::clamp(std::fabs(remaining) / zone(), MIN_FRACTION, 1.0f), remaining);
        switch (target.kind) {
            case GoalKind::FORWARD: return {0, drive, headingStick(-turned)};
            case GoalKind::SIDEWAYS:
                return {-drive, 0, headingStick(-turned)}; // positive is right, and lx > 0 walks left
            case GoalKind::TURN: return {0, 0, drive};
            case GoalKind::ARC: {
                // Forward along the arc turns its way; turning back walks back along it.
                const float ideal = passTurned + direction * (pose.distance - passDistance) / target.radius;
                return arcSticks(std::copysign(1.0f, target.angle) * drive,
                                 drive * turnPerForward() + headingStick(ideal - turned));
            }
        }
        return {};
    }
};
