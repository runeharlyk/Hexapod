#pragma once

#include <odometry.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <initializer_list>

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

/** Sticks in MotionService's convention: ly > 0 walks forward, lx > 0 right and rx > 0 turns clockwise. */
struct GoalSticks {
    float lx = 0, ly = 0, rx = 0;
};

/**
 * Drives one goal to its target, called every time the caller predicts where the robot would come to rest if its sticks
 * centred now: the goal is judged on that rest pose, never on where the robot is. The remaining distance along the
 * goal's axis in the frame it started in, or the remaining turn on the unwrapped fused yaw, sets the driving stick: the
 * asked speed, slowed in proportion over the slowdown zone, never below MIN_FRACTION of it. The zone is SLOW_DOWN, or
 * the distance the asked speed covers in two stops' coasts when that is longer, so a fast walk has room to slow. Once
 * stopping now lands nearer the target than stopping on the next call would, the pass has arrived for good: its sticks
 * centre and it waits for the robot to come to rest. The settled pose is judged: within the tolerance it has REACHED;
 * otherwise a corrective pass walks the remainder, up to MAX_PASSES passes in all, and then it has TIMED_OUT. So has a
 * pass that takes longer than twice its expected time plus TIMEOUT_MARGIN_S.
 *
 * A straight walk holds the start heading with its turn stick and its start line with the stick across it, and is
 * judged on its cross-track as well; a settled pose on the target but off the line gets a pass that walks only across.
 * An arc walks forward with the turn stick its radius implies, scaled by what it learns of the gait's turn, slows on
 * the angle its position has left to sweep about the arc's centre and steers its heading toward the arc's tangent
 * there. A settled arc on its end point whose heading misses gets a pass that turns in place.
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
    // An arc's turn stick learns the gait's turn against what it asks, at this rate per rad of heading lag a second:
    // BI's feet slip in a turn on the simulation's arc, where its kinematic yaw turns 13-19 % more than the IMU's, and
    // the heading hold alone holds a lag that lays the walk outside the circle.
    static constexpr float TURN_GAIN_RATE = 5.0f;
    static constexpr float MIN_TURN_GAIN = 0.5f, MAX_TURN_GAIN = 2.0f;
    static constexpr float CROSS_GAIN = 10.0f; // stick across the line per m off it
    static constexpr float CROSS_STICK_MAX = 0.3f;
    static constexpr float CROSS_REACHED_M = 0.02f;
    static constexpr float TIMEOUT_MARGIN_S = 2.0f;
    // MotionService's GAIT_COMMAND_TAU_S: the gait follows a stick's speed through a first-order lag of this time.
    static constexpr float RAMP_TAU_S = 0.15f;
    // How far the robot walks on once its sticks centre, as a time at the speed it walked, which sizes the slowdown
    // zone. The stop ramp (gait.h GaitCommandRamp) eases the stroke out through two first-order stages of RAMP_TAU_S,
    // leaving the share s = (1 + t / tau) exp(-t / tau), and slows the cadence with it, so the speed falls as s^2: its
    // integral is 1.25 tau. The virtual robot coasts 0.19 s of its speed at full speed, measured on TRI and BI.
    static constexpr float STOP_S = 1.25f * RAMP_TAU_S;

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

    /**
     * Starts `goal` from the rest pose `rest`; `topSpeed` (mm/s) and `topTurn` (rad/s) are what a full stick gives at
     * the posture. An unwalkable posture's empty envelope is refused: its goal could never time out.
     */
    void start(const Goal& goal, const OdometryEstimate& rest, float topSpeed, float topTurn) {
        const auto positive = [](float top) { return std::isfinite(top) && top > 0; };
        if (!valid(goal) || !positive(topSpeed) || !positive(topTurn)) {
            current = GoalState::REFUSED;
            passCount = 0;
            return;
        }
        target = goal;
        origin = rest;
        lastYaw = rest.yaw;
        turned = 0;
        lastSwept = swept = 0;
        furthest = 0;
        turnGain = 1;
        topSpeedMs = topSpeed / 1000.0f;
        topTurnRads = topTurn;
        passCount = 0;
        beginPass(rest, false);
    }

    /**
     * One call: where the robot would come to rest if its sticks centred now, whether it is at rest, and the time since
     * the last call. Returns the sticks, centred unless running.
     */
    GoalSticks update(const OdometryEstimate& rest, bool atRest, float dt) {
        if (!active()) return {};
        // The odometry's yaw wraps at pi; a turn of any size is followed on the sum of its wrapped changes.
        turned += std::remainder(rest.yaw - lastYaw, 2.0f * static_cast<float>(M_PI));
        lastYaw = rest.yaw;
        if (target.kind == GoalKind::ARC) {
            const float around = aroundCentre(rest);
            swept += std::remainder(around - lastSwept, 2.0f * static_cast<float>(M_PI));
            lastSwept = around;
        }
        const float before = remaining;
        remaining = aim() - covered(rest);
        restAdvance = std::max((before - remaining) * direction, 0.0f);
        furthest = std::max(furthest, 1 - remaining / aim());
        elapsed += dt;
        if (elapsed > deadline) {
            current = GoalState::TIMED_OUT;
            return {};
        }
        if (current == GoalState::SETTLING) {
            if (!atRest) return {};
            if (within(rest)) {
                current = GoalState::REACHED;
                return {};
            }
            if (passCount == MAX_PASSES) {
                current = GoalState::TIMED_OUT;
                return {};
            }
            if (target.kind != GoalKind::ARC || std::fabs(target.angle - swept) > sweptTolerance()) {
                beginPass(rest, false);
            } else if (endMiss(rest) <= ARC_REACHED_M) {
                beginPass(rest, true);
            } else {
                // An arc whose angle is swept off its circle has no remainder for a pass to walk.
                current = GoalState::TIMED_OUT;
                return {};
            }
        }
        // It arrives once that rest is nearer the target than one more call's would be, by how far the last call moved
        // it, and stays arrived: the rest moves back as the sticks centre, and a pass driving again would pulse.
        if (!arrived) arrived = remaining * direction <= 0.5f * restAdvance;
        if (arrived && std::fabs(cross(rest)) <= CROSS_REACHED_M) {
            current = GoalState::SETTLING;
            return {};
        }
        const GoalSticks out = sticks(rest);
        if (passKind() == GoalKind::ARC && !arrived)
            turnGain = std::clamp(turnGain + TURN_GAIN_RATE * direction * (idealHeading() - turned) * dt, MIN_TURN_GAIN,
                                  MAX_TURN_GAIN);
        return out;
    }

    void cancel() {
        if (active()) current = GoalState::CANCELLED;
    }

    GoalState state() const { return current; }
    /**
     * The share of the target covered, from 0 to 1, by the furthest the judged pose has come: where a stop would come
     * to rest moves back and forth by a few millimetres within each step, which is no going back. So it reads 1 from an
     * overshoot on, through a corrective pass back and a goal that then TIMED_OUT.
     */
    float progress() const {
        if (current == GoalState::REACHED) return 1;
        const bool started = current != GoalState::IDLE && current != GoalState::REFUSED;
        return started ? std::clamp(furthest, 0.0f, 1.0f) : 0;
    }
    /** The passes started, the first included. */
    int passes() const { return passCount; }

  private:
    Goal target;
    GoalState current = GoalState::IDLE;
    OdometryEstimate origin;
    float lastYaw = 0, turned = 0;  // the fused yaw turned since the start, unwrapped
    float lastSwept = 0, swept = 0; // an arc's position swept about its centre since the start, unwrapped
    float remaining = 0;            // of the pass's axis, signed as the target
    float furthest = 0;             // the most of the target covered, as a share of it
    float topSpeedMs = 0, topTurnRads = 0;
    float turnGain = 1; // what an arc's turn stick is scaled by, learnt from the heading's lag behind its ideal
    int passCount = 0;
    // The pass in progress: the sign of the remainder it walks (0 for a pass only across the line), whether it only
    // turns an arc's heading in place, whether it has arrived, how far the rest moved toward the target on the last
    // call, and its clock.
    float direction = 0;
    bool turnInPlace = false, arrived = false;
    float restAdvance = 0, elapsed = 0, deadline = 0;

    bool active() const { return current == GoalState::RUNNING || current == GoalState::SETTLING; }
    bool linear() const { return target.kind == GoalKind::FORWARD || target.kind == GoalKind::SIDEWAYS; }
    // What the pass walks: the goal's own kind, or a turn for an arc's pass on its heading.
    GoalKind passKind() const { return turnInPlace ? GoalKind::TURN : target.kind; }
    float aim() const { return linear() ? target.distance : target.angle; }
    float zone() const { return std::max(linear() ? SLOW_DOWN_M : SLOW_DOWN_RAD, 2 * pace() * STOP_S); }
    // The share of the asked speed the remainder drives at.
    float share() const { return std::clamp(std::fabs(remaining) / zone(), MIN_FRACTION, 1.0f); }
    float turnPerForward() const { return topSpeedMs / target.radius / topTurnRads; }
    // An arc's swept angle is held to REACHED_RAD, or on a tight arc to what ARC_REACHED_M of its end allows.
    float sweptTolerance() const { return std::max(REACHED_RAD, ARC_REACHED_M / target.radius); }

    // The pose relative to the start: x right and y forward of the heading it started on.
    float startRight(const OdometryEstimate& p) const {
        return std::cos(origin.yaw) * (p.x - origin.x) + std::sin(origin.yaw) * (p.y - origin.y);
    }
    float startForward(const OdometryEstimate& p) const {
        return -std::sin(origin.yaw) * (p.x - origin.x) + std::cos(origin.yaw) * (p.y - origin.y);
    }

    float covered(const OdometryEstimate& p) const {
        switch (passKind()) {
            case GoalKind::FORWARD: return startForward(p);
            case GoalKind::SIDEWAYS: return startRight(p);
            case GoalKind::TURN: return turned;
            case GoalKind::ARC: return swept;
        }
        return 0;
    }

    // The heading an arc steers toward: the arc's tangent at the angle its position has swept. The path distance over
    // the radius would count the gait's sway and any way walked outside the circle: on BI's arc in the simulation it
    // ran 3 deg past the swept angle, and a heading steered toward it caught up with it as the arc slowed.
    float idealHeading() const { return swept; }

    // An arc's position as an angle about its centre, signed as the arc's angle and 0 at the start. An arc is judged on
    // it rather than on its yaw, so that it ends on its end point.
    float aroundCentre(const OdometryEstimate& p) const {
        const float side = std::copysign(1.0f, target.angle);
        return side * std::atan2(startForward(p), target.radius + side * startRight(p));
    }

    // How far an arc's pose is from its end point.
    float endMiss(const OdometryEstimate& p) const {
        const float endRight = -std::copysign(target.radius * (1 - std::cos(target.angle)), target.angle);
        const float endForward = target.radius * std::sin(std::fabs(target.angle));
        return std::hypot(startRight(p) - endRight, startForward(p) - endForward);
    }

    // How far off the start line a straight walk is: right of it for a forward walk, ahead of it for a sideways one.
    float cross(const OdometryEstimate& p) const {
        switch (target.kind) {
            case GoalKind::FORWARD: return startRight(p);
            case GoalKind::SIDEWAYS: return startForward(p);
            default: return 0;
        }
    }

    bool within(const OdometryEstimate& p) const {
        switch (target.kind) {
            case GoalKind::FORWARD:
            case GoalKind::SIDEWAYS: return std::fabs(remaining) <= REACHED_M && std::fabs(cross(p)) <= CROSS_REACHED_M;
            case GoalKind::TURN: return std::fabs(remaining) <= REACHED_RAD;
            case GoalKind::ARC:
                return std::fabs(target.angle - swept) <= sweptTolerance() &&
                       std::fabs(target.angle - turned) <= REACHED_RAD && endMiss(p) <= ARC_REACHED_M;
        }
        return false;
    }

    float headingStick(float error) const {
        return std::clamp(HEADING_GAIN * error, -HEADING_STICK_MAX, HEADING_STICK_MAX);
    }

    // The sticks for a walk right, forward and counter-clockwise, as the odometry measures them.
    static GoalSticks walk(float right, float forward, float counterClockwise) {
        return {right, forward, -counterClockwise};
    }

    // An arc's sticks, the forward one slowed when the turn one would pass a full stick, so the curvature is kept.
    static GoalSticks arcSticks(float forward, float counterClockwise) {
        const float over = std::max(1.0f, std::fabs(counterClockwise));
        return walk(0, forward / over, counterClockwise / over);
    }

    // The rate the pass's axis moves at with a full share of the asked speed: m/s, or rad/s for a turn or an arc.
    float pace() const {
        switch (passKind()) {
            case GoalKind::TURN: return target.speed * topTurnRads;
            case GoalKind::ARC: {
                // MotionService shares a combined stick out over its L1 norm, which slows the turn with the walk.
                const GoalSticks full = arcSticks(target.speed, target.speed * turnPerForward());
                return std::fabs(full.rx) / std::max(1.0f, full.ly + std::fabs(full.rx)) * topTurnRads;
            }
            default: return target.speed * topSpeedMs;
        }
    }

    // A pass from the rest pose `rest`: on the goal's own axis, or on an arc's heading alone when `turnOnly`.
    void beginPass(const OdometryEstimate& rest, bool turnOnly) {
        passCount++;
        turnInPlace = turnOnly;
        remaining = aim() - covered(rest);
        direction = linear() && std::fabs(remaining) <= REACHED_M ? 0 : std::copysign(1.0f, remaining);
        arrived = false;
        restAdvance = 0;
        elapsed = 0;
        const float d = std::fabs(remaining);
        const float expected = (std::max(d - zone(), 0.0f) + std::min(d, zone()) / MIN_FRACTION) / pace();
        deadline = 2 * expected + TIMEOUT_MARGIN_S;
        current = GoalState::RUNNING;
    }

    GoalSticks sticks(const OdometryEstimate& rest) const {
        // Arrived, a straight walk still off its line walks only across it.
        const float drive = arrived ? 0 : std::copysign(target.speed * share(), remaining);
        const float across = std::clamp(CROSS_GAIN * cross(rest), -CROSS_STICK_MAX, CROSS_STICK_MAX);
        switch (passKind()) {
            // Right of the line walks left; ahead of a sideways walk's line walks back.
            case GoalKind::FORWARD: return walk(-across, drive, headingStick(-turned));
            case GoalKind::SIDEWAYS: return walk(drive, -across, headingStick(-turned));
            case GoalKind::TURN: return walk(0, 0, drive);
            case GoalKind::ARC: {
                // Forward along the arc turns its way; turning back walks back along it.
                return arcSticks(std::copysign(1.0f, target.angle) * drive,
                                 drive * turnPerForward() * turnGain + headingStick(idealHeading() - turned));
            }
        }
        return {};
    }
};
