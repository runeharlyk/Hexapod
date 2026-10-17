#pragma once

#include <message_types.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

// Ported from SpotMicroESP32-Leika (esp32/include/odometry.h) to six legs and the body frame of docs/conventions.md.

/** Where the robot has gone since the last reset, in the frame it faced then: x right, y forward. */
struct OdometryEstimate {
    float x = 0, y = 0;     // m
    float yaw = 0;          // rad, counter-clockwise, in [-pi, pi]
    float kinematicYaw = 0; // rad, as yaw: the heading the feet alone give, for diagnosis
    float distance = 0;     // m travelled along the path, never decreasing
};

/** The robot's motion over one tick, in the frame it faced at the start of the tick. */
struct BodyStep {
    float right = 0, forward = 0; // m
    float turn = 0;               // rad, counter-clockwise
};

/**
 * Dead reckoning from the commanded feet. A foot on the floor is fixed to it, so the robot's motion is the inverse of
 * the rigid motion that best carries those feet from the last tick to this one: a least-squares fit over the feet on
 * the floor in both.
 *
 * The feet are taken as the gait places them, before the body's tilt and shift: those move the body over the planted
 * feet, and the robot travels only as far as the gait strokes the feet.
 *
 * It knows only what the gait commanded: servo lag, slip and short steps make the robot travel otherwise.
 */
class KinematicOdometry {
  public:
    // A foot this far above the lowest one is in the air, whatever its phase: one a stopped walk eases down.
    static constexpr float LIFTED_MM = 3.f;

    /** Zeroes the estimate; the feet of the last tick still count, so the tick in progress is not lost. */
    void reset() { current = {}; }
    /** The feet moved in ticks this was not shown: the next update takes them as it finds them. */
    void pause() { primed = false; }

    const OdometryEstimate &estimate() const { return current; }
    /** Whether the last tick's turn was seen: two feet or more were down in it and the tick before. */
    bool turnSeen() const { return sawTurn; }
    /** Whether every foot was on the floor in the last tick: all six in stance and none being lowered. */
    bool allDown() const { return everyFootDown; }

    /** One control tick: the feet the gait left and which legs it has in stance. Returns the step the robot took. */
    BodyStep update(const BodyStateMsg &body, const std::array<bool, 6> &stance) {
        float feet[6][2];
        bool contact[6];
        float lowest = body.feet[0][2];
        for (int leg = 1; leg < 6; leg++) lowest = std::min(lowest, body.feet[leg][2]);
        for (int leg = 0; leg < 6; leg++) {
            feet[leg][0] = body.feet[leg][0] / 1000.f;
            feet[leg][1] = body.feet[leg][1] / 1000.f;
            contact[leg] = stance[leg] && body.feet[leg][2] - lowest < LIFTED_MM;
        }
        everyFootDown = std::all_of(std::begin(contact), std::end(contact), [](bool down) { return down; });
        sawTurn = false;
        const BodyStep step = primed ? integrate(feet, contact) : BodyStep {};
        for (int leg = 0; leg < 6; leg++) {
            previous[leg][0] = feet[leg][0];
            previous[leg][1] = feet[leg][1];
            previousContact[leg] = contact[leg];
        }
        primed = true;
        return step;
    }

  private:
    OdometryEstimate current;
    float previous[6][2] = {};
    bool previousContact[6] = {};
    bool primed = false;
    bool sawTurn = false;
    bool everyFootDown = false;

    BodyStep integrate(const float feet[6][2], const bool contact[6]) {
        int count = 0;
        float before[2] = {0, 0}, after[2] = {0, 0};
        for (int leg = 0; leg < 6; leg++) {
            if (!contact[leg] || !previousContact[leg]) continue;
            count++;
            before[0] += previous[leg][0], before[1] += previous[leg][1];
            after[0] += feet[leg][0], after[1] += feet[leg][1];
        }
        if (count == 0) return {};
        before[0] /= count, before[1] /= count, after[0] /= count, after[1] /= count;

        // The feet turn by -turn about their centroid; one foot alone shows no turn.
        float cross = 0, dot = 0;
        for (int leg = 0; leg < 6; leg++) {
            if (!contact[leg] || !previousContact[leg]) continue;
            const float px = previous[leg][0] - before[0], py = previous[leg][1] - before[1];
            const float qx = feet[leg][0] - after[0], qy = feet[leg][1] - after[1];
            cross += px * qy - py * qx;
            dot += px * qx + py * qy;
        }
        sawTurn = count > 1;
        const float turn = sawTurn ? -std::atan2(cross, dot) : 0.0f;

        // A floor point p in the last frame is R(-turn) (p - step) in this one; the centroids give the step.
        const float c = std::cos(turn), s = std::sin(turn);
        const BodyStep step {before[0] - (c * after[0] - s * after[1]), before[1] - (s * after[0] + c * after[1]),
                             turn};

        const float heading = current.yaw;
        current.x += std::cos(heading) * step.right - std::sin(heading) * step.forward;
        current.y += std::sin(heading) * step.right + std::cos(heading) * step.forward;
        current.yaw = current.kinematicYaw = std::remainder(heading + turn, 2.0f * static_cast<float>(M_PI));
        current.distance += std::hypot(step.right, step.forward);
        return step;
    }
};

/**
 * The kinematic odometry with its heading corrected by the gyro.
 *
 * The feet and the gyro each measure the yaw rate. The gyro's z rate has white noise and a residual bias the boot
 * calibration left; the feet's has no bias, but misses what the gait does not command: the walk rocks the body in yaw,
 * and slip makes a turn come up short. While the robot keeps still, a scalar Kalman filter estimates the bias from the
 * difference of the two rates; the heading integrates the two rates weighted by their variances, the gyro's while
 * walking and the feet's while standing. The position integrates the feet's steps in the fused heading.
 *
 * The gyro is read every 25 ms, the control runs every 5: a sample is fused on the tick it was read, over the time
 * since the last one, and the heading follows the feet's turn in between. Without a gyro sample it is the kinematic
 * odometry, and the bias stays where it was.
 */
class FusedOdometry {
  public:
    // The constants are Leika's. The gyro noise was measured on Leika's still Pico on 2026-10-01; the feet's error
    // model below was fitted in Leika's MuJoCo model. Neither is measured on this robot.
    static constexpr float GYRO_NOISE = 0.0012f; // rad/s
    // The bias the filter starts out expecting: the scale of one a boot calibration leaves when the robot moved
    // through it.
    static constexpr float BIAS_PRIOR = 0.05f; // rad/s
    // How fast the bias wanders, assumed: about 0.006 rad/s over an hour, a few degrees' warming.
    static constexpr float BIAS_WALK = 1e-4f; // rad/s per square-root second
    // The feet's yaw rate error at a standstill (planted feet barely turn: assumed), per m/s walked and per rad/s
    // turned.
    static constexpr float KIN_STILL = 0.0002f;  // rad/s
    static constexpr float KIN_PER_SPEED = 2.0f; // rad/s per m/s
    static constexpr float KIN_PER_TURN = 0.4f;  // rad/s per rad/s
    // The feet's motion counts on, fading with this time constant, once they stop: the body coasts on and the servos
    // lag the commanded feet. The robot counts as still only once it has kept still this long.
    static constexpr float SETTLE_S = 0.2f;
    // Feet that move on the floor by less than this keep the robot still; a stopped walk's ease home moves them more.
    static constexpr float STILL_RATE = 0.01f; // rad/s
    // The body's yaw over the planted feet (psi, eased back to 0 after an animation) turns the gyro and not the feet:
    // faster than the gyro resolves, it is no stillness. Tilt and shift turn the body about x and y or not at all, so
    // they never reach the gyro's z.
    static constexpr float PSI_STILL_RATE = GYRO_NOISE; // rad/s
    // A sample counts over the time since the sample before, at most this long; a longer gap, a stalled read or a long
    // tick, is left to the feet.
    static constexpr float GYRO_STALE_S = 0.05f;

    /** Zeroes the estimate; the bias, a property of the gyro, stays. */
    void reset() {
        kinematic.reset();
        current = {};
        // The next sample counts from here, so no turn from before the reset lands in the new frame.
        sinceSampleS = 0;
        feetTurn = feetTravel = 0;
        feetSawTurn = true;
    }

    /** The motion left the gait for a while: the feet are taken afresh, and no gyro sample spans the gap. */
    void pause() {
        kinematic.pause();
        sinceSampleS = INFINITY;
    }

    OdometryEstimate estimate() const {
        OdometryEstimate e = current;
        e.kinematicYaw = kinematic.estimate().yaw;
        return e;
    }

    /**
     * One control tick of `dt` seconds: the body and stance as KinematicOdometry takes them, and the gyro's z rate when
     * it was read on this tick. Returns the step it took, its turn as the fused heading took it.
     */
    BodyStep update(const BodyStateMsg &body, const std::array<bool, 6> &stance, std::optional<float> gyroZ, float dt) {
        const BodyStep step = kinematic.update(body, stance);
        noteStill(body, dt);
        sinceSampleS += dt;
        feetTurn += step.turn;
        feetTravel += std::hypot(step.right, step.forward);
        feetSawTurn = feetSawTurn && kinematic.turnSeen();
        float turn = step.turn;
        if (gyroZ) {
            // The feet's turns since the last sample were taken provisionally; the fused turn replaces them.
            if (sinceSampleS > 0 && sinceSampleS <= GYRO_STALE_S) turn += fusedTurn(*gyroZ) - feetTurn;
            sinceSampleS = 0;
            feetTurn = feetTravel = 0;
            feetSawTurn = true;
        }
        const float c = std::cos(current.yaw), s = std::sin(current.yaw);
        current.x += c * step.right - s * step.forward;
        current.y += s * step.right + c * step.forward;
        current.yaw = std::remainder(current.yaw + turn, 2.0f * static_cast<float>(M_PI));
        current.distance += std::hypot(step.right, step.forward);
        return {step.right, step.forward, turn};
    }

  private:
    KinematicOdometry kinematic;
    OdometryEstimate current;
    float bias = 0;
    float biasVariance = BIAS_PRIOR * BIAS_PRIOR;
    float motionVariance = 0; // of the feet's rate, from their motion
    float stillS = 0;         // how long every foot has been down under a body yaw that keeps still
    float lastPsi = 0;
    // Since the last gyro sample: its age, the feet's turn and travel, and whether every tick saw the feet's turn.
    float sinceSampleS = INFINITY;
    float feetTurn = 0, feetTravel = 0;
    bool feetSawTurn = true;

    static float square(float value) { return value * value; }

    // A gait that cycles swings a foot, and one that stopped sets every stance flag (GaitController::inStance), so
    // every foot down means the gait is not cycling, no stance change is stepping and no foot is being lowered.
    void noteStill(const BodyStateMsg &body, float dt) {
        const bool psiStill = dt > 0 && std::fabs(body.psi - lastPsi) / dt < PSI_STILL_RATE;
        stillS = kinematic.allDown() && psiStill ? stillS + dt : 0;
        lastPsi = body.psi;
    }

    // The gyro reads the feet's rate plus the bias, give or take both their noises. On a robot that keeps still, their
    // difference updates the bias; on a moving one it would also carry the feet's undercount, which is no noise. The
    // two rates, weighted by their variances, give the turn since the last sample.
    float fusedTurn(float gyroRate) {
        const float interval = sinceSampleS;
        biasVariance += square(BIAS_WALK) * interval;
        // Feet that did not see the turn leave it to the gyro.
        if (!feetSawTurn) return (gyroRate - bias) * interval;
        const float feetRate = feetTurn / interval, speed = feetTravel / interval;
        motionVariance = std::max(square(KIN_PER_SPEED * speed) + square(KIN_PER_TURN * feetRate),
                                  motionVariance * std::exp(-interval / SETTLE_S));
        const float feetVariance = square(KIN_STILL) + motionVariance;
        if (stillS >= SETTLE_S && motionVariance < square(STILL_RATE)) {
            const float gain = biasVariance / (biasVariance + square(GYRO_NOISE) + feetVariance);
            bias += gain * (gyroRate - feetRate - bias);
            biasVariance *= 1 - gain;
        }
        const float gyroVariance = square(GYRO_NOISE) + biasVariance;
        const float gyroWeight = feetVariance / (feetVariance + gyroVariance);
        return (gyroWeight * (gyroRate - bias) + (1 - gyroWeight) * feetRate) * interval;
    }
};
