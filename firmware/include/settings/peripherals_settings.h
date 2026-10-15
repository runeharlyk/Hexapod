#pragma once

#include <algorithm>
#include <platform_shared/api.pb.h>
#include <peripherals/compass.h>
#include <template/state_result.h>

#ifndef SDA_PIN
#define SDA_PIN 47
#endif
#ifndef SCL_PIN
#define SCL_PIN 21
#endif

using PeripheralSettings = api_PeripheralSettings;

// The bus is clocked for its slowest device: the MPU6050 and HMC5883L are rated for 400 kHz.
constexpr int32_t I2C_MAX_FREQUENCY = 400000;

inline PeripheralSettings PeripheralSettings_defaults() {
    PeripheralSettings settings = api_PeripheralSettings_init_zero;
    settings.sda = SDA_PIN;
    settings.scl = SCL_PIN;
    settings.frequency = 400000;
    return settings;
}

inline void PeripheralSettings_read(const PeripheralSettings &settings, PeripheralSettings &proto) {
    proto = settings;
}

inline StateUpdateResult PeripheralSettings_update(const PeripheralSettings &proto, PeripheralSettings &settings) {
    // Any GPIO on the S3, and the I2C standard/fast/fast-plus range. A clock above I2C_MAX_FREQUENCY,
    // which earlier firmware accepted and robots may have stored, is lowered rather than refused: a
    // refused file is replaced by the defaults, which would also lose the stored pins.
    if (proto.sda < 0 || proto.sda > 48 || proto.scl < 0 || proto.scl > 48) return StateUpdateResult::ERROR;
    if (proto.sda == proto.scl) return StateUpdateResult::ERROR;
    if (proto.frequency < 100000 || proto.frequency > 1000000) return StateUpdateResult::ERROR;
    if (!compass::valid({proto.mag_offset_x, proto.mag_offset_y, proto.mag_offset_z})) return StateUpdateResult::ERROR;
    const int32_t frequency = std::min(proto.frequency, I2C_MAX_FREQUENCY);

    if (proto.sda == settings.sda && proto.scl == settings.scl && frequency == settings.frequency &&
        proto.mag_offset_x == settings.mag_offset_x && proto.mag_offset_y == settings.mag_offset_y &&
        proto.mag_offset_z == settings.mag_offset_z) {
        return StateUpdateResult::UNCHANGED;
    }

    settings = proto;
    settings.frequency = frequency;
    return StateUpdateResult::CHANGED;
}
