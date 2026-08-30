#pragma once

#include <cstdint>
#include <utils/math_utils.h>

#ifndef NUM_SERVO
#define NUM_SERVO 18
#endif

// Internal EventBus message structs (in-RAM only; the wire protocol is protobuf).

enum class MOTION_STATE { DEACTIVATED, IDLE, POSE, STAND, WALK };

struct ModeMsg {
    MOTION_STATE mode;
};

// TUNED is the CMA-ES-searched gait from simulation/src/resources/gait_library.json, emitted into
// gait_tuned.h by simulation/export_gait.py. Appended last so the existing wire values are stable.
enum class GaitType { TRI_GATE, BI_GATE, WAVE, RIPPLE, TUNED, AUTO };

struct GaitMsg {
    GaitType gait;
};

struct ServoAnglesMsg {
    float angles[NUM_SERVO];
};

struct ServoSignalMsg {
    int8_t id;
    uint16_t pwm;
};

struct ServoStateMsg {
    bool active;
};

struct IMUAnglesMsg {
    float rpy[3]{0, 0, 0};
    float temperature{-1};
    float heading{0};  // magnetometer compass heading in degrees; 0 when no magnetometer
    bool success{false};
};

struct CommandMsg {
    float lx, ly, rx, ry, h, s, s1, fd;
};

struct BodyStateMsg {
    float omega, phi, psi, xm, ym, zm;
    float feet[6][4];

    void updateFeet(const float newFeet[6][4]) { COPY_2D_ARRAY_6x4(feet, newFeet); }

    bool operator==(const BodyStateMsg &other) const {
        if (!IS_ALMOST_EQUAL(omega, other.omega) || !IS_ALMOST_EQUAL(phi, other.phi) ||
            !IS_ALMOST_EQUAL(psi, other.psi) || !IS_ALMOST_EQUAL(xm, other.xm) || !IS_ALMOST_EQUAL(ym, other.ym) ||
            !IS_ALMOST_EQUAL(zm, other.zm)) {
            return false;
        }
        return arrayEqual(feet, other.feet, 0.1);
    }
};
