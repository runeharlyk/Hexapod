#pragma once

#include <cmath>
#include <numbers>

// The compass maths that does not touch the chip: removing the hard-iron offset and taking the heading.
namespace compass {

// The HMC5883L saturates near 190 uT at the gain the driver sets, so an offset beyond this is not a hard iron
// but a wrong fit.
constexpr float OFFSET_LIMIT_UT = 200.0f;

struct Offset {
    float x {0};
    float y {0};
    float z {0};
};

inline bool valid(const Offset& offset) {
    return std::fabs(offset.x) <= OFFSET_LIMIT_UT && std::fabs(offset.y) <= OFFSET_LIMIT_UT &&
           std::fabs(offset.z) <= OFFSET_LIMIT_UT; // false for NaN as well
}

inline void correct(const float raw[3], const Offset& offset, float corrected[3]) {
    corrected[0] = raw[0] - offset.x;
    corrected[1] = raw[1] - offset.y;
    corrected[2] = raw[2] - offset.z;
}

// Degrees in [0, 360): the angle of the field in the chip's XY plane plus the declination (radians, east positive).
// Clockwise from north only if the chip's +x points forward and +y to the left; that is opposite in sign to the body
// yaw (counter-clockwise), and the chip's mounting rotation relative to the body frame is unverified (docs/conventions.md).
inline float headingDegrees(const float field[3], float declination) {
    constexpr float TWO_PI = 2 * std::numbers::pi_v<float>;
    float heading = std::fmod(std::atan2(field[1], field[0]) + declination, TWO_PI);
    if (heading < 0) heading += TWO_PI;
    return heading * (180.0f / std::numbers::pi_v<float>);
}

} // namespace compass
