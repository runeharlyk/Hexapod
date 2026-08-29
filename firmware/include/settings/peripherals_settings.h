#pragma once

#include <platform_shared/api.pb.h>
#include <template/state_result.h>
#include <event_bus.h>

#ifndef SDA_PIN
#define SDA_PIN 47
#endif
#ifndef SCL_PIN
#define SCL_PIN 21
#endif

using PeripheralSettings = api_PeripheralSettings;

// One shallow slot: the I2C pin/frequency config is a rare snapshot, not a stream.
using PeripheralSettingsBus = EventBus<PeripheralSettings, 2, 2, 1>;

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
    // Bounds match the app's form: any GPIO on the S3, and the I2C standard/fast/fast-plus range.
    if (proto.sda < 0 || proto.sda > 48 || proto.scl < 0 || proto.scl > 48) return StateUpdateResult::ERROR;
    if (proto.sda == proto.scl) return StateUpdateResult::ERROR;
    if (proto.frequency < 100000 || proto.frequency > 1000000) return StateUpdateResult::ERROR;

    if (proto.sda == settings.sda && proto.scl == settings.scl && proto.frequency == settings.frequency) {
        return StateUpdateResult::UNCHANGED;
    }

    settings = proto;
    return StateUpdateResult::CHANGED;
}
