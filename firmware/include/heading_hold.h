#pragma once

#include <goal_controller.h>
#include <algorithm>
#include <cmath>

/**
 * Holds the heading of a walk driven by the sticks, on the fused odometry's yaw, while its client asks for it: the
 * classic gait drifts in yaw, as slip and the body's rocking turn it where the feet do not, so a straight stick walks
 * an arc.
 *
 * It holds while the walk stick is off centre and the turn stick centred. A turn stick let go leaves the robot turning
 * on through the stop ramp's two stages (GaitCommandRamp, gait.h), so the heading is taken once the turn stick has been
 * centred for SETTLE_S; from then on it steers the turn stick against the yaw's error from that heading, by the goals'
 * law (GoalController::HEADING_GAIN, at most HEADING_STICK_MAX), outside a deadband that keeps the fused yaw's noise
 * from stepping the gait. A turn stick, a centred walk stick or a client that stops asking lets the heading go, and the
 * next walk takes its own.
 */
class HeadingHold {
  public:
    // Two of the ramp's time constants: the turn left of a let-go stick is then (1 + 2) exp(-2), 41 % of it, and
    // falling.
    static constexpr float SETTLE_S = 2 * GoalController::RAMP_TAU_S;
    static constexpr float DEADBAND_RAD = 1.0f * static_cast<float>(M_PI) / 180.0f;

    /**
     * One control tick: the client's sticks (MotionService's convention), whether it asks for the hold, and the fused
     * yaw. Returns the turn stick to add to the client's, clockwise positive as rx.
     */
    float update(float lx, float ly, float rx, bool asked, float yaw, float dt) {
        if (!asked || rx != 0 || (lx == 0 && ly == 0)) {
            release();
            return 0;
        }
        if (!holding) {
            settledS += dt;
            if (settledS < SETTLE_S) return 0;
            holding = true;
            heading = yaw;
        }
        const float error = std::remainder(heading - yaw, 2.0f * static_cast<float>(M_PI));
        if (std::fabs(error) <= DEADBAND_RAD) return 0;
        // Counter-clockwise error turns counter-clockwise: rx < 0.
        return -std::clamp(GoalController::HEADING_GAIN * error, -GoalController::HEADING_STICK_MAX,
                           GoalController::HEADING_STICK_MAX);
    }

    void release() {
        holding = false;
        settledS = 0;
    }

    bool held() const { return holding; }

  private:
    bool holding = false;
    float settledS = 0;
    float heading = 0;
};
