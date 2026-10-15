#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

// Whether the MPU6050 may be calibrated now. The calibration drives the gyro to zero and the
// accelerometer to (0, 0, +1 g), so it is only right for a robot that lies still with the chip level:
// calibrating a moving or tilted robot would bake the motion or the tilt into every later reading.
namespace imu_calibration {

enum class Check { OK, MOVING, TILTED };

// Raw counts at the ranges the driver sets: +-2000 deg/s gyro, +-2 g accelerometer.
constexpr float GYRO_LSB_PER_DEG_S = 16.4f;
constexpr float ACCEL_LSB_PER_G = 16384.0f;

// The sensor's noise at rest is ~0.05 deg/s RMS; a servo twitch or a hand on the robot is far above
// this. The spread is judged, not the mean: the mean is the very bias being calibrated away.
constexpr float STILL_GYRO_STD_DEG_S = 0.5f;
// cos(25 deg): a robot lying on its belly on a floor is level well within this.
constexpr float LEVEL_MIN_Z_G = 0.906f;

// `gyro` holds `count` raw xyz triples, `accel` one raw xyz triple.
inline Check check(const int16_t (*gyro)[3], size_t count, const int16_t accel[3]) {
    for (int axis = 0; axis < 3; ++axis) {
        float sum = 0.f, sumSq = 0.f;
        for (size_t i = 0; i < count; ++i) {
            const float v = gyro[i][axis] / GYRO_LSB_PER_DEG_S;
            sum += v;
            sumSq += v * v;
        }
        const float mean = sum / count;
        const float variance = sumSq / count - mean * mean;
        if (variance > STILL_GYRO_STD_DEG_S * STILL_GYRO_STD_DEG_S) return Check::MOVING;
    }
    const float x = accel[0] / ACCEL_LSB_PER_G, y = accel[1] / ACCEL_LSB_PER_G, z = accel[2] / ACCEL_LSB_PER_G;
    const float norm = std::sqrt(x * x + y * y + z * z);
    if (norm < 0.5f || z / norm < LEVEL_MIN_Z_G) return Check::TILTED;
    return Check::OK;
}

}  // namespace imu_calibration
