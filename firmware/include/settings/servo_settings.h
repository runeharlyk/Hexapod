#pragma once

#include <platform_shared/api.pb.h>
#include <settings/servo_factory.h>
#include <template/state_result.h>
#include <event_bus.h>
#include <esp_log.h>
#include <cstring>

// Shallow shared bus: calibration is a rare full snapshot, not a stream (default depth would waste
// ~7 KB static on the ~720 B payload). One definition so the service and controller share it.
using ServoSettingsBus = EventBus<ServoSettings, 2, 2, 1>;

inline void ServoSettings_read(const ServoSettings &settings, ServoSettings &proto) { proto = settings; }

// Written as inclusive range tests so a NaN arriving over the wire fails instead of propagating into
// the PWM math and driving a servo into its endstop.
inline bool Servo_isValid(const api_Servo &servo) {
    if (!(servo.center_pwm >= 0.0f && servo.center_pwm <= 4095.0f)) return false;
    if (!(servo.conversion > 0.0f && servo.conversion <= 100.0f)) return false;
    if (servo.direction != 1.0f && servo.direction != -1.0f) return false;
    if (!(servo.center_angle >= -180.0f && servo.center_angle <= 180.0f)) return false;
    return true;
}

inline bool Servo_equals(const api_Servo &a, const api_Servo &b) {
    return a.center_pwm == b.center_pwm && a.conversion == b.conversion && a.direction == b.direction &&
           a.center_angle == b.center_angle && strncmp(a.name, b.name, sizeof(a.name)) == 0;
}

inline StateUpdateResult ServoSettings_update(const ServoSettings &proto, ServoSettings &settings) {
    // Calibration is an all-or-nothing set: a partially applied one leaves the robot in a pose it was
    // never commanded into, so a single bad entry rejects the whole payload.
    for (pb_size_t i = 0; i < proto.servos_count; i++) {
        if (!Servo_isValid(proto.servos[i])) {
            ESP_LOGE("ServoSettings", "Servo %u calibration out of range", (unsigned)i);
            return StateUpdateResult::ERROR;
        }
    }

    if (proto.servos_count == settings.servos_count) {
        bool identical = true;
        for (pb_size_t i = 0; i < proto.servos_count && identical; i++) {
            identical = Servo_equals(proto.servos[i], settings.servos[i]);
        }
        if (identical) return StateUpdateResult::UNCHANGED;
    }

    settings = proto;
    return StateUpdateResult::CHANGED;
}
