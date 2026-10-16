#pragma once

// The MPU6050 DMP's roll, pitch and yaw (Peripherals::angleX/Y/Z) against the body frame of docs/conventions.md, as
// the simulation measures them: roll about x (nose down negative) agrees, while pitch about y (right side down
// positive) and yaw (counter-clockwise positive) come out mirrored. The gravity vector and the gyro agree. Measured
// on the robot on 2026-10-16 with the sensor mounted square to the body.
namespace imu_frame {

constexpr float ROLL_SIGN = +1.0f;
constexpr float PITCH_SIGN = -1.0f;
constexpr float YAW_SIGN = -1.0f;

} // namespace imu_frame
