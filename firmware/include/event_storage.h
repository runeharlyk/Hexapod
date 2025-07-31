#pragma once
#include <ArduinoJson.h>

#include <event_bus.h>
#include <message_types.h>
#include <filesystem.h>
#include <settings/servo_settings.h>

#define SERVO_SETTINGS_JSON_FILE "/config/ServoSettingsMsg.json"

class EventStorage {
  public:
    EventStorage() {}
    ~EventStorage() {}

    void begin() { registerEvent<ServoSettingsMsg>(SERVO_SETTINGS_JSON_FILE); }

  private:
    template <typename T>
    void registerEvent(const char* filename) {
        loadSettings<T>(filename);
        EventBus::subscribe<T>([this, filename](T const& t) { save(t, filename); });
    }

    template <typename T>
    void loadSettings(const char* filename) {
        File file = ESPFS.open(filename, "r");
        if (!file) {
            ESP_LOGE("EventStorage", "Failed to open file %s", filename);
            save(T(), filename);
            return;
        }

        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, file);
        file.close();

        if (error) {
            ESP_LOGE("EventStorage", "Failed to parse JSON from %s: %s", filename, error.c_str());
            save(T(), filename);
            return;
        }

        T settings;
        JsonObject root = doc.as<JsonObject>();
        settings.fromJson(root);
        EventBus::publish<T>(settings);
        ESP_LOGI("EventStorage", "Loaded settings from %s", filename);
    }

    template <typename T>
    void save(const T& eventData, const char* filename) {
        JsonDocument doc;
        JsonObject obj = doc.to<JsonObject>();
        toJson(obj, eventData);

        File file = ESPFS.open(filename, "w");
        if (!file) {
            ESP_LOGE("EventStorage", "Failed to open file %s for writing", filename);
            return;
        }

        serializeJson(doc, file);
        file.close();
        ESP_LOGI("EventStorage", "Saved settings to %s", filename);
    }
};