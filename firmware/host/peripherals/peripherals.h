#pragma once

// Host stand-in for firmware/include/peripherals/peripherals.h with the interface Hexapod and the motion
// code use. No chip is read: the host sets the IMU state, for instance from a physics simulation. Hexapod
// owns its Peripherals, so the state is shared by every instance and set through setImu().

#include <algorithm>
#include <cstdint>
#include <vector>
#include <message_types.h>
#include <peripherals/imu_calibration.h>
#include <platform_shared/message.pb.h>

using ImuCalibration = socket_message_ImuCalibration;

class Peripherals {
  public:
    struct Imu {
        float rpyRad[3] = {0, 0, 0};       // roll, pitch, yaw in radians, as the MPU6050 driver reports them
        float gravityUp[3] = {0, 0, 1};    // body-frame unit vector, DMP convention (world up)
        float gyroRad[3] = {0, 0, 0};      // body-frame rates, rad/s
    };

    static void setImu(const Imu &imu) { state() = imu; }

    void begin() {}

    bool readIMU() { return true; }
    bool readMagnetometer() { return false; }

    float angleX() { return state().rpyRad[0]; }
    float angleY() { return state().rpyRad[1]; }
    float angleZ() { return state().rpyRad[2]; }
    // The host's yaw has no FIFO to reset.
    uint32_t imuFifoResets() const { return 0; }
    void gravityBody(float out[3]) { std::copy_n(state().gravityUp, 3, out); }
    void gyroRad(float out[3]) { std::copy_n(state().gyroRad, 3, out); }

    std::vector<uint8_t> scanI2C() { return {}; }
    bool imuActive() const { return true; }
    bool magActive() const { return false; }
    imu_calibration::Check calibrateIMU(ImuCalibration &) { return imu_calibration::Check::MOVING; }

    void fillTelemetry(socket_message_TickSample &tick) {
        std::copy_n(state().rpyRad, 3, tick.rpy);
        gyroRad(tick.gyro);
        gravityBody(tick.gravity);
    }

    IMUAnglesMsg getIMUAngles() {
        IMUAnglesMsg msg;
        std::copy_n(state().rpyRad, 3, msg.rpy);
        msg.success = true;
        return msg;
    }

  private:
    static Imu &state() {
        static Imu imu;
        return imu;
    }
};
