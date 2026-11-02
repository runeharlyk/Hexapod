#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <kinematics.h>
#include <settings/servo_factory.h>
#include <utils/math_utils.h>

#ifndef FACTORY_SERVO_PWM_FREQUENCY
#define FACTORY_SERVO_PWM_FREQUENCY 50
#endif

// The MG92B's safe pulse span, in microseconds: 0.5 to 2.5 ms, the usual full range of a 180 deg hobby servo. On the
// bench (2026-11-01) the right front coxa, centred at 317 counts, turned freely from 160 to 526 counts, about 170 to
// 180 deg, 2 counts a degree, before it met the middle leg and the body; the corner coxas' 90 deg (JOINT_LIMIT_DEG)
// lie within the span. Adafruit gives about 1 and 2 ms as the servo's ends at 90 deg either side and warns that a pulse
// too far can break it (https://www.adafruit.com/product/2307); TowerPro states no range
// (https://www.towerpro.com.tw/product/mg92b/).
constexpr uint32_t SERVO_SAFE_PULSE_MIN_US = 500;
constexpr uint32_t SERVO_SAFE_PULSE_MAX_US = 2500;

namespace servo_travel_detail {
constexpr uint64_t PCA9685_COUNTS = 4096; // per PWM period
constexpr uint64_t countsTimesMillion(uint32_t us) {
    return static_cast<uint64_t>(us) * FACTORY_SERVO_PWM_FREQUENCY * PCA9685_COUNTS;
}
} // namespace servo_travel_detail

// The span in PCA9685 counts, rounded inward: 102.4 and 512 at 50 Hz, so 103 to 512. The prescale's rounding runs the
// chip at 50.03 Hz (pca9685.h), 102.5 and 512.3 counts, the same whole counts.
constexpr uint16_t SERVO_SAFE_PULSE_MIN =
    (servo_travel_detail::countsTimesMillion(SERVO_SAFE_PULSE_MIN_US) + 999999) / 1000000;
constexpr uint16_t SERVO_SAFE_PULSE_MAX = servo_travel_detail::countsTimesMillion(SERVO_SAFE_PULSE_MAX_US) / 1000000;

struct JointTravel {
    float min, max; // degrees
};

// The angles of leg `leg`'s coxa whose pulses lie within the safe span, through ServoController::toPwm's
// pwm = (angle + centerAngle) * direction * conversion + centerPwm, and within the leg's JOINT_LIMIT_DEG. Each side
// apart: a calibrated centre off the nominal 1.5 ms leaves the coxa more travel one way than the other.
constexpr JointTravel coxaTravel(int leg, float centerPwm, float conversion, float direction, float centerAngle) {
    const float scale = direction * conversion;
    const float a = (SERVO_SAFE_PULSE_MIN - centerPwm) / scale - centerAngle;
    const float b = (SERVO_SAFE_PULSE_MAX - centerPwm) / scale - centerAngle;
    const float limit = JOINT_LIMIT_DEG[leg][0];
    return {std::max(std::min(a, b), -limit), std::min(std::max(a, b), limit)};
}

// At the factory calibration, the centre at 306 counts and 2 counts per degree, the safe pulses span -101.5 to +103
// deg, so every coxa turns its leg's full JOINT_LIMIT_DEG.
constexpr bool factoryCoxasTurnTheirLimits() {
    for (int leg = 0; leg < 6; ++leg) {
        const JointTravel travel =
            coxaTravel(leg, FACTORY_CENTER_PWM, FACTORY_CONVERSION, FACTORY_DIRECTION[leg * 3], FACTORY_CENTER_ANGLE);
        if (travel.min != -JOINT_LIMIT_DEG[leg][0] || travel.max != JOINT_LIMIT_DEG[leg][0]) return false;
    }
    return true;
}
static_assert(factoryCoxasTurnTheirLimits(),
              "the safe pulses cover every coxa's JOINT_LIMIT_DEG at the factory centre");

// Each coxa side with less than its leg's JOINT_LIMIT_DEG of travel, per leg in `travel`, written into `out` as its
// servo and signed travel (" servo 6 +87.0"); how many sides. SHORT_COXA_SIDES_SIZE holds all twelve.
constexpr size_t SHORT_COXA_SIDES_SIZE = 12 * 18;
inline int shortCoxaSides(const JointTravel (&travel)[6], char *out, size_t size) {
    int sides = 0;
    size_t used = 0;
    out[0] = '\0';
    for (int leg = 0; leg < 6; leg++)
        for (const float side : {travel[leg].min, travel[leg].max}) {
            if (std::fabs(side) >= JOINT_LIMIT_DEG[leg][0]) continue;
            ++sides;
            if (used < size) used += std::snprintf(out + used, size - used, " servo %d %+.1f", leg * 3, side);
        }
    return sides;
}

// Which coxa travel holds a pose: the servo's safe pulses, or for TUNED and WALK_NN within them TRAINED_COXA_LIMIT_DEG.
enum class CoxaTravel : uint8_t { SERVO, TRAINED };

// Servo `servo`'s joint angle held within its travel: a coxa within `coxa`, its leg's travel within its servo's safe
// pulses, and within TRAINED_COXA_LIMIT_DEG too for the trained travel; a femur or tibia within JOINT_LIMIT_DEG.
inline float jointWithinTravel(float angleDeg, int servo, JointTravel coxa, CoxaTravel travel) {
    const int joint = servo % 3;
    if (joint != 0) return CLIP(angleDeg, -JOINT_LIMIT_DEG[servo / 3][joint], JOINT_LIMIT_DEG[servo / 3][joint]);
    if (travel == CoxaTravel::TRAINED) {
        coxa.min = std::max(coxa.min, -TRAINED_COXA_LIMIT_DEG);
        coxa.max = std::min(coxa.max, TRAINED_COXA_LIMIT_DEG);
    }
    return CLIP(angleDeg, coxa.min, coxa.max);
}
