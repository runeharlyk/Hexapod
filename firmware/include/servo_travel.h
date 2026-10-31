#pragma once

#include <algorithm>
#include <cstdint>
#include <kinematics.h>
#include <settings/servo_factory.h>
#include <utils/math_utils.h>

#ifndef FACTORY_SERVO_PWM_FREQUENCY
#define FACTORY_SERVO_PWM_FREQUENCY 50
#endif

// The MG92B's safe pulse span, in microseconds. Adafruit gives about 1 ms and 2 ms as the servo's two ends, about 90
// deg either side of the 1.5 ms centre, and says to try 0.75 to 2.25 ms for more travel, warning that a pulse too far
// can break the servo, which has no mechanical stops (https://www.adafruit.com/product/2307). TowerPro's own page
// states no pulse range (https://www.towerpro.com.tw/product/mg92b/). The span is the 1 to 2 ms, 250 us inside each end
// of that.
constexpr uint32_t SERVO_SAFE_PULSE_MIN_US = 1000;
constexpr uint32_t SERVO_SAFE_PULSE_MAX_US = 2000;

namespace servo_travel_detail {
constexpr uint64_t PCA9685_COUNTS = 4096; // per PWM period
constexpr uint64_t countsTimesMillion(uint32_t us) {
    return static_cast<uint64_t>(us) * FACTORY_SERVO_PWM_FREQUENCY * PCA9685_COUNTS;
}
} // namespace servo_travel_detail

// The span in PCA9685 counts, rounded inward: 204.8 and 409.6 at 50 Hz, so 205 to 409. The prescale's rounding runs
// the chip at 50.03 Hz (pca9685.h), 204.9 and 409.8 counts, the same whole counts.
constexpr uint16_t SERVO_SAFE_PULSE_MIN =
    (servo_travel_detail::countsTimesMillion(SERVO_SAFE_PULSE_MIN_US) + 999999) / 1000000;
constexpr uint16_t SERVO_SAFE_PULSE_MAX = servo_travel_detail::countsTimesMillion(SERVO_SAFE_PULSE_MAX_US) / 1000000;

struct JointTravel {
    float min, max; // degrees
};

// The coxa angles whose pulses lie within the safe span, through ServoController::toPwm's
// pwm = (angle + centerAngle) * direction * conversion + centerPwm. Each side apart: a calibrated centre off the
// nominal 1.5 ms leaves the coxa more travel one way than the other.
constexpr JointTravel coxaTravel(float centerPwm, float conversion, float direction, float centerAngle) {
    const float scale = direction * conversion;
    const float a = (SERVO_SAFE_PULSE_MIN - centerPwm) / scale - centerAngle;
    const float b = (SERVO_SAFE_PULSE_MAX - centerPwm) / scale - centerAngle;
    return a < b ? JointTravel {a, b} : JointTravel {b, a};
}

// At the factory calibration, the centre at 306 counts and 2 counts per degree: -50.5 to +51.5 deg, every coxa turning
// the same way. JOINT_LIMIT_DEG's symmetric coxa travel is the narrower side.
constexpr JointTravel FACTORY_COXA_TRAVEL =
    coxaTravel(FACTORY_CENTER_PWM, FACTORY_CONVERSION, FACTORY_DIRECTION[0], FACTORY_CENTER_ANGLE);
static_assert(JOINT_LIMIT_DEG[0] == std::min(-FACTORY_COXA_TRAVEL.min, FACTORY_COXA_TRAVEL.max),
              "kinematics.h's coxa travel is the factory calibration's narrower side of the safe pulses");

// Which coxa travel holds a pose: the servo's safe pulses, or for TUNED and WALK_NN within them TRAINED_COXA_LIMIT_DEG.
enum class CoxaTravel : uint8_t { SERVO, TRAINED };

// A joint angle (joint 0 coxa, 1 femur, 2 tibia) held within its travel: a coxa within `coxa`, its servo's safe
// pulses, and within TRAINED_COXA_LIMIT_DEG too for the trained travel; a femur or tibia within JOINT_LIMIT_DEG.
inline float jointWithinTravel(float angleDeg, int joint, JointTravel coxa, CoxaTravel travel) {
    if (joint != 0) return CLIP(angleDeg, -JOINT_LIMIT_DEG[joint], JOINT_LIMIT_DEG[joint]);
    if (travel == CoxaTravel::TRAINED) {
        coxa.min = std::max(coxa.min, -TRAINED_COXA_LIMIT_DEG);
        coxa.max = std::min(coxa.max, TRAINED_COXA_LIMIT_DEG);
    }
    return CLIP(angleDeg, coxa.min, coxa.max);
}
