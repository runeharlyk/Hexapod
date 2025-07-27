#pragma once

#include <ArduinoJson.h>
#include <utils/string_utils.h>
#include <template/state_result.h>

#ifndef FACTORY_BT_DEVICE_NAME
#define FACTORY_BT_DEVICE_NAME "Hexapod"
#endif

typedef struct {
    String deviceName;

    void serialize(JsonObject &json) const { json["device_name"] = deviceName; }

    bool deserialize(const JsonObject &json) {
        deviceName = json["device_name"].as<String>();
        if (deviceName.length() < 1 || deviceName.length() > 31) {
            ESP_LOGE("BluetoothSettings", "Device name length is invalid: %s", deviceName.c_str());
            return false;
        }
        return true;
    }

} bluetooth_settings_t;

inline bluetooth_settings_t createDefaultBluetoothSettings() {
    return bluetooth_settings_t {.deviceName = FACTORY_BT_DEVICE_NAME};
}

class BluetoothSettings {
  public:
    bluetooth_settings_t settings;

    static void read(BluetoothSettings &settingsObj, JsonObject &root) {
        JsonObject btSettings = root["bluetooth"].to<JsonObject>();
        settingsObj.settings.serialize(btSettings);
        ESP_LOGV("BluetoothSettings", "Bluetooth Settings read");
    }

    static StateUpdateResult update(JsonObject &root, BluetoothSettings &settingsObj) {
        if (root["bluetooth"].is<JsonObject>()) {
            JsonObject btSettings = root["bluetooth"];
            if (settingsObj.settings.deserialize(btSettings)) {
                ESP_LOGV("BluetoothSettings", "Bluetooth Settings updated");
                return StateUpdateResult::CHANGED;
            } else {
                ESP_LOGE("BluetoothSettings", "Failed to deserialize Bluetooth settings");
                return StateUpdateResult::ERROR;
            }
        }
        ESP_LOGV("BluetoothSettings", "No Bluetooth settings found in update");
        return StateUpdateResult::UNCHANGED;
    }
};