#pragma once

#include <imu_frame.h>
#include <message_types.h>
#include <utils/math_utils.h>

// Walking keeps its original tilt range; a planted stand permits a larger commanded attitude.
static constexpr float BODY_TILT_LIMIT_RAD = 0.254f;
static constexpr float STAND_TILT_LIMIT_RAD = 0.31415927f; // 18 degrees
// The fraction of the way the body pose moves toward its target per 5 ms control tick.
static constexpr float BODY_SMOOTHING = 0.06f;

// Self-levelling adds the measured tilt to the commanded attitude. On a slope steeper than the body can lean, or with
// a tumbling IMU, the sum would ask the legs for a pose past the joint limits; it is held to the stick's own range
// instead.
inline float levelled(float commanded, float measured, float limit = BODY_TILT_LIMIT_RAD) {
    return CLIP(commanded + measured, -limit, limit);
}

// One control tick of the body attitude's approach to the commanded pitch and roll plus the measured tilt (radians,
// as Peripherals reports it, so turned into the body frame first); yaw eases back to 0, since only ANIMATE sets it.
inline void levelAttitude(BodyStateMsg &body, float phi, float omega, float measuredRoll, float measuredPitch,
                          float limit = BODY_TILT_LIMIT_RAD) {
    body.phi = lerpf(body.phi, levelled(phi, imu_frame::PITCH_SIGN * measuredPitch, limit), BODY_SMOOTHING);
    body.omega = lerpf(body.omega, levelled(omega, imu_frame::ROLL_SIGN * measuredRoll, limit), BODY_SMOOTHING);
    body.psi = lerpf(body.psi, 0.f, BODY_SMOOTHING);
}
