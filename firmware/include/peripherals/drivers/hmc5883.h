#pragma once

#include <peripherals/i2c_bus.h>
#include <utils/math_utils.h>

/*
 * HMC5883L 3-axis magnetometer over the shared I2C bus, replacing the Arduino
 * Adafruit_HMC5883_U driver. Datasheet register map: config A/B, mode, then six
 * data bytes in X, Z, Y order (not X, Y, Z) — the classic trap with this part.
 *
 * The QMC5883L clones sold on the same breakout use a different register map and
 * do not answer the H/4/3 identification, so begin() rejects them rather than
 * reporting plausible-looking garbage.
 */
class HMC5883Driver {
  public:
    static constexpr uint8_t DEFAULT_ADDR = 0x1E;

    // Magnetic declination for the robot's location, radians. Positive = east.
    explicit HMC5883Driver(uint8_t addr = DEFAULT_ADDR, float declination = 0.22f)
        : _addr(addr), _declination(declination) {}

    bool begin() {
        if (!I2CBus::instance().probe(_addr)) return false;

        uint8_t id[3] = {0, 0, 0};
        if (I2CBus::instance().readReg(_addr, REG_ID_A, id, 3) != ESP_OK) return false;
        if (id[0] != 'H' || id[1] != '4' || id[2] != '3') return false;

        // 8-sample averaging, 15 Hz output, normal measurement bias.
        if (!writeReg(REG_CONFIG_A, 0x70)) return false;
        if (!writeReg(REG_CONFIG_B, GAIN_1_3_GA)) return false;
        if (!writeReg(REG_MODE, MODE_CONTINUOUS)) return false;

        vTaskDelay(pdMS_TO_TICKS(10));
        _initialized = true;
        return true;
    }

    bool update() {
        if (!_initialized) return false;

        uint8_t buf[6];
        if (I2CBus::instance().readReg(_addr, REG_DATA_X_MSB, buf, 6) != ESP_OK) return false;

        const int16_t x = (int16_t)((buf[0] << 8) | buf[1]);
        const int16_t z = (int16_t)((buf[2] << 8) | buf[3]);
        const int16_t y = (int16_t)((buf[4] << 8) | buf[5]);

        // -4096 is the datasheet's saturation/overflow marker on any axis.
        if (x == -4096 || y == -4096 || z == -4096) return false;

        _uT[0] = x * LSB_TO_UT;
        _uT[1] = y * LSB_TO_UT;
        _uT[2] = z * LSB_TO_UT;

        float heading = std::atan2(_uT[1], _uT[0]) + _declination;
        if (heading < 0) heading += 2 * PI_F;
        if (heading > 2 * PI_F) heading -= 2 * PI_F;
        _heading = RAD_TO_DEG_F(heading);
        return true;
    }

    float getX() const { return _uT[0]; }
    float getY() const { return _uT[1]; }
    float getZ() const { return _uT[2]; }
    float getHeading() const { return _heading; }
    bool isInitialized() const { return _initialized; }

  private:
    static constexpr uint8_t REG_CONFIG_A = 0x00;
    static constexpr uint8_t REG_CONFIG_B = 0x01;
    static constexpr uint8_t REG_MODE = 0x02;
    static constexpr uint8_t REG_DATA_X_MSB = 0x03;
    static constexpr uint8_t REG_ID_A = 0x0A;

    static constexpr uint8_t MODE_CONTINUOUS = 0x00;
    static constexpr uint8_t GAIN_1_3_GA = 0x20; // +/-1.3 Ga, 1090 LSB/Gauss
    // 1 Gauss = 100 uT, so 1 LSB = 100/1090 uT at the default gain.
    static constexpr float LSB_TO_UT = 100.0f / 1090.0f;

    bool writeReg(uint8_t reg, uint8_t value) {
        return I2CBus::instance().writeReg(_addr, reg, &value, 1) == ESP_OK;
    }

    uint8_t _addr;
    float _declination;
    bool _initialized{false};
    float _uT[3]{0, 0, 0};
    float _heading{0};
};
