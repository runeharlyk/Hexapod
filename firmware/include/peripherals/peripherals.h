#pragma once

#include <esp_log.h>
#include <vector>

#include <feature_flags.h>
#include <message_types.h>
#include <peripherals/i2c_bus.h>
#include <settings/peripherals_settings.h>

#if FT_ENABLED(USE_MPU6050)
#include <peripherals/drivers/mpu6050.h>
#endif

#if FT_ENABLED(USE_MAG)
#include <peripherals/drivers/hmc5883.h>
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
            PeripheralSettings cfg = PeripheralSettings_defaults();
            PeripheralSettingsBus::peek(cfg);  // stored config if the service published first
            ESP_LOGI(TAG, "I2C bus sda=%d scl=%d %d Hz", (int)cfg.sda, (int)cfg.scl, (int)cfg.frequency);
            I2CBus::instance().begin(static_cast<gpio_num_t>(cfg.sda), static_cast<gpio_num_t>(cfg.scl),
                                     (uint32_t)cfg.frequency);
        }
#if FT_ENABLED(USE_MPU6050)
        _imuReady = _imu.begin();
        if (!_imuReady) ESP_LOGW(TAG, "MPU6050 init failed");
        else ESP_LOGI(TAG, "MPU6050 ready");
#endif
#if FT_ENABLED(USE_MAG)
        _magReady = _mag.begin();
        if (!_magReady) ESP_LOGW(TAG, "HMC5883 init failed");
        else ESP_LOGI(TAG, "HMC5883 ready");
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

    float angleZ() {
#if FT_ENABLED(USE_MPU6050)
        return _imu.getYaw();
#else
        return 0.0f;
#endif
    }

    // Gravity unit vector in body frame, DMP convention (world-UP). The learned policy wants
    // world-DOWN, so its runner negates.
    void gravityBody(float out[3]) {
#if FT_ENABLED(USE_MPU6050)
        _imu.getGravity(out);
#else
        out[0] = out[1] = 0.0f;
        out[2] = 1.0f;
#endif
    }

    void gyroRad(float out[3]) {
#if FT_ENABLED(USE_MPU6050)
        _imu.getGyro(out);
#else
        out[0] = out[1] = out[2] = 0.0f;
#endif
    }

    float angleY() {
#if FT_ENABLED(USE_MPU6050)
        return _imu.getPitch();
#else
        return 0.0f;
#endif
    }

    bool readMagnetometer() {
#if FT_ENABLED(USE_MAG)
        if (!_magReady) return false;
        return _mag.update();
#else
        return false;
#endif
    }

    bool magActive() const {
#if FT_ENABLED(USE_MAG)
        return _magReady;
#else
        return false;
#endif
    }

    // Compass heading in degrees, 0 = magnetic north + declination. 0 when absent.
    float heading() const {
#if FT_ENABLED(USE_MAG)
        return _magReady ? _mag.getHeading() : 0.0f;
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
#if FT_ENABLED(USE_MAG)
        msg.heading = heading();
#endif
        return msg;
    }

  private:
    static constexpr const char *TAG = "Peripherals";
#if FT_ENABLED(USE_MPU6050)
    MPU6050Driver _imu;
    bool _imuReady{false};
#endif
#if FT_ENABLED(USE_MAG)
    HMC5883Driver _mag;
    bool _magReady{false};
#endif
};
