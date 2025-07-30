#pragma once

#include <list>
#include <SPI.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <utils/math_utils.h>

#include <MPU6050_6Axis_MotionApps612.h>
#include <Adafruit_BNO055.h>

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
        imu_success = _imu.testConnection();
        devStatus = _imu.dmpInitialize();
        if (!imu_success) return false;
        _imu.CalibrateAccel(6);
        _imu.CalibrateGyro(6);
        _imu.setDMPEnabled(true);
        _imu.setI2CMasterModeEnabled(false);
        _imu.setI2CBypassEnabled(true);
        _imu.setSleepEnabled(false);
#endif
#if FT_ENABLED(USE_BNO055)
        imu_success = _imu.begin();
        if (!imu_success) return false;
        _imu.setExtCrystalUse(true);
#endif
        return true;
    }

    bool readIMU() {
        if (!imu_success) return false;
#if FT_ENABLED(USE_MPU6050)
        bool updated = _imu.dmpGetCurrentFIFOPacket(fifoBuffer);
        _imu.dmpGetQuaternion(&q, fifoBuffer);
        _imu.dmpGetGravity(&gravity, &q);
        _imu.dmpGetYawPitchRoll(ypr, &q, &gravity);
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

    float getTemperature() { return imu_success ? imu_temperature : -1; }

    float getAngleX() { return imu_success ? ypr[2] : 0; }

    float getAngleY() { return imu_success ? ypr[1] : 0; }

    float getAngleZ() { return imu_success ? ypr[0] : 0; }

    void readIMU(JsonObject& root) {
        if (!imu_success) return;
        root["x"] = round2(getAngleX());
        root["y"] = round2(getAngleY());
        root["z"] = round2(getAngleZ());
    }

    bool active() { return imu_success; }

  private:
#if FT_ENABLED(USE_MPU6050)
    MPU6050 _imu;
    uint8_t devStatus {false};
    Quaternion q;
    uint8_t fifoBuffer[64];
    VectorFloat gravity;
#endif
#if FT_ENABLED(USE_BNO055)
    Adafruit_BNO055 _imu;
#endif
    bool imu_success {false};
    float ypr[3];
    float imu_temperature {-1};
};