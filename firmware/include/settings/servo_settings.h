#pragma once

#include <platform_shared/api.pb.h>
#include <template/state_result.h>
#include <event_bus.h>

using ServoSettings = api_ServoSettings;

// Shallow shared bus: calibration is a rare full snapshot, not a stream (default depth would waste
// ~7 KB static on the ~720 B payload). One definition so the service and controller share it.
using ServoSettingsBus = EventBus<ServoSettings, 2, 2, 1>;

inline ServoSettings ServoSettings_defaults() {
    ServoSettings settings = api_ServoSettings_init_zero;
    settings.servos_count = 18;
    // Factory calibration reproducing the ServoController defaults; direction is the old l_dir/r_dir.
    static const float direction[18] = {-1, -1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, -1, 1, -1};
    for (int i = 0; i < 18; i++) {
        settings.servos[i].center_pwm = 306;
        settings.servos[i].conversion = 2;
        settings.servos[i].direction = direction[i];
        settings.servos[i].center_angle = 0;
    }
    return settings;
}

inline void ServoSettings_read(const ServoSettings &settings, ServoSettings &proto) { proto = settings; }

inline StateUpdateResult ServoSettings_update(const ServoSettings &proto, ServoSettings &settings) {
    settings = proto;
    return StateUpdateResult::CHANGED;
}
