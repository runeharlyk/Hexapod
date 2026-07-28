#pragma once

#include <esp_log.h>
#include <vector>

#include <features.h>
#include <message_types.h>
#include <peripherals/i2c_bus.h>

#if FT_ENABLED(USE_MPU6050)
#include <peripherals/drivers/mpu6050.h>
#endif

#ifndef SDA_PIN
#define SDA_PIN 47
#endif
#ifndef SCL_PIN
#define SCL_PIN 21
#endif

class Peripherals {
  public:
    void begin() {
        if (!I2CBus::instance().isInitialized()) {
            I2CBus::instance().begin(static_cast<gpio_num_t>(SDA_PIN), static_cast<gpio_num_t>(SCL_PIN), 400000);
        }
#if FT_ENABLED(USE_MPU6050)
        _imuReady = _imu.begin();
        if (!_imuReady) ESP_LOGW(TAG, "MPU6050 init failed");
        else ESP_LOGI(TAG, "MPU6050 ready");
#endif
    }

    std::vector<uint8_t> scanI2C() { return I2CBus::instance().scan(1, 127); }

    bool readIMU() {
#if FT_ENABLED(USE_MPU6050)
        if (!_imuReady) return false;
        return _imu.update();
#else
        return false;
#endif
    }

    float angleX() {
#if FT_ENABLED(USE_MPU6050)
        return _imu.getRoll();
#else
        return 0.0f;
#endif
    }

    float angleY() {
#if FT_ENABLED(USE_MPU6050)
        return _imu.getPitch();
#else
        return 0.0f;
#endif
    }

    IMUAnglesMsg getIMUAngles() {
        IMUAnglesMsg msg;
#if FT_ENABLED(USE_MPU6050)
        msg.rpy[0] = _imu.getRoll();
        msg.rpy[1] = _imu.getPitch();
        msg.rpy[2] = _imu.getYaw();
        msg.temperature = _imu.getTemperature();
        msg.success = _imuReady;
#endif
        return msg;
    }

  private:
    static constexpr const char *TAG = "Peripherals";
#if FT_ENABLED(USE_MPU6050)
    MPU6050Driver _imu;
    bool _imuReady{false};
#endif
};
