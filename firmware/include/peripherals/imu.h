#pragma once

#include <list>
#include <SPI.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <utils/math_utils.h>

#include <MPU6050_6Axis_MotionApps612.h>
#include <Adafruit_BNO055.h>

#include <message_types.h>

class IMU {
  public:
    IMU()
#if FT_ENABLED(USE_BNO055)
        : _imu(55)
#endif
    {
    }
    bool initialize() {
#if FT_ENABLED(USE_MPU6050)
        _imu.initialize();
        imuMsg.success = _imu.testConnection();
        devStatus = _imu.dmpInitialize();
        if (!imuMsg.success) return false;
        _imu.CalibrateAccel(6);
        _imu.CalibrateGyro(6);
        _imu.setDMPEnabled(true);
        _imu.setI2CMasterModeEnabled(false);
        _imu.setI2CBypassEnabled(true);
        _imu.setSleepEnabled(false);
#endif
#if FT_ENABLED(USE_BNO055)
        imuMsg.success = _imu.begin();
        if (!imuMsg.success) return false;
        _imu.setExtCrystalUse(true);
#endif
        return true;
    }

    bool readIMU() {
        if (!imuMsg.success) return false;
#if FT_ENABLED(USE_MPU6050)
        bool updated = _imu.dmpGetCurrentFIFOPacket(fifoBuffer);
        _imu.dmpGetQuaternion(&q, fifoBuffer);
        _imu.dmpGetGravity(&gravity, &q);
        _imu.dmpGetYawPitchRoll(imuMsg.rpy, &q, &gravity);
        _imu.dmpGetGyro(&gyroRaw, fifoBuffer);
        return updated;
#endif
#if FT_ENABLED(USE_BNO055)
        sensors_event_t event;
        _imu.getEvent(&event);
        imu_temperature = event.temperature;
        ypr[0] = event.orientation.x;
        ypr[1] = event.orientation.y;
        ypr[2] = event.orientation.z;
#endif
        return true;
    }

    float getTemperature() { return imuMsg.temperature; }

    float getAngleX() { return imuMsg.rpy[2]; }

    float getAngleY() { return imuMsg.rpy[1]; }

    float getAngleZ() { return imuMsg.rpy[0]; }

    bool active() { return imuMsg.success; }

    IMUAnglesMsg getIMUAngles() { return imuMsg; }

    void getGyroRad(float out[3]) {
#if FT_ENABLED(USE_MPU6050)
        constexpr float GYRO_RAD_PER_LSB = (2000.0f / 32768.0f) * ((float)M_PI / 180.0f);
        out[0] = gyroRaw.x * GYRO_RAD_PER_LSB;
        out[1] = gyroRaw.y * GYRO_RAD_PER_LSB;
        out[2] = gyroRaw.z * GYRO_RAD_PER_LSB;
#else
        out[0] = out[1] = out[2] = 0.0f;
#endif
    }

    void getGravity(float out[3]) {
#if FT_ENABLED(USE_MPU6050)
        out[0] = gravity.x;
        out[1] = gravity.y;
        out[2] = gravity.z;
#else
        out[0] = out[1] = 0.0f;
        out[2] = 1.0f;
#endif
    }

  private:
#if FT_ENABLED(USE_MPU6050)
    MPU6050 _imu;
    uint8_t devStatus {false};
    Quaternion q;
    uint8_t fifoBuffer[64];
    VectorFloat gravity;
    VectorInt16 gyroRaw;
#endif
#if FT_ENABLED(USE_BNO055)
    Adafruit_BNO055 _imu;
#endif
    IMUAnglesMsg imuMsg;
};