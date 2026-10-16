#pragma once

#include <platform_shared/api.pb.h>

using ServoSettings = api_ServoSettings;

// Factory calibration reproducing the ServoController defaults.
constexpr pb_size_t FACTORY_SERVO_COUNT = 18;
constexpr float FACTORY_CENTER_PWM = 306;
constexpr float FACTORY_CONVERSION = 2;
constexpr float FACTORY_CENTER_ANGLE = 0;
constexpr float FACTORY_DIRECTION[FACTORY_SERVO_COUNT] = {1, 1, -1, 1, 1, -1, 1, 1, -1, 1, -1, 1, 1, -1, 1, 1, -1, 1};

inline ServoSettings ServoSettings_defaults() {
    ServoSettings settings = api_ServoSettings_init_zero;
    settings.servos_count = FACTORY_SERVO_COUNT;
    for (pb_size_t i = 0; i < FACTORY_SERVO_COUNT; i++) {
        settings.servos[i].center_pwm = FACTORY_CENTER_PWM;
        settings.servos[i].conversion = FACTORY_CONVERSION;
        settings.servos[i].direction = FACTORY_DIRECTION[i];
        settings.servos[i].center_angle = FACTORY_CENTER_ANGLE;
    }
    return settings;
}

// Nothing records that a calibration was made, but one moves some servo off the factory values; a
// servo's name is a label, not calibration. Compared field by field rather than against a
// ServoSettings_defaults() copy, which would put another ~720 B on the caller's stack.
inline bool ServoSettings_calibrated(const ServoSettings &settings) {
    pb_size_t count = settings.servos_count < FACTORY_SERVO_COUNT ? settings.servos_count : FACTORY_SERVO_COUNT;
    for (pb_size_t i = 0; i < count; i++) {
        const api_Servo &servo = settings.servos[i];
        if (servo.center_pwm != FACTORY_CENTER_PWM || servo.conversion != FACTORY_CONVERSION ||
            servo.direction != FACTORY_DIRECTION[i] || servo.center_angle != FACTORY_CENTER_ANGLE)
            return true;
    }
    return false;
}
