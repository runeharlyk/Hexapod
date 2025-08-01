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
        static typename EventBus<T>::Handle h;
        loadSettings<T>(filename);
        if (!h.valid()) h = EventBus<T>::subscribe([this, filename](const T& t) { save(t, filename); });
    }

    template <typename T>
    esp_err_t loadSettings(const char* filename) {
        File file = ESPFS.open(filename, "r");
        if (!file) {
            ESP_LOGE("EventStorage", "Failed to open file %s", filename);
            save(T(), filename);
            return ESP_FAIL;
        }

        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, file);
        file.close();

        if (error) {
            ESP_LOGE("EventStorage", "Failed to parse JSON from %s: %s", filename, error.c_str());
            save(T(), filename);
            return ESP_FAIL;
        }

        T settings;
        settings.fromJson(doc);
        ESP_LOGI("EventStorage", "Loaded settings from %s", filename);
        EventBus<T>::store(settings);
        return ESP_OK;
    }

    template <typename T>
    esp_err_t save(const T& eventData, const char* filename) {
        ESP_LOGI("EventStorage", "About to save");
        JsonDocument doc;
        toJson(doc, eventData);

        File file = ESPFS.open(filename, "w");
        if (!file) {
            ESP_LOGE("EventStorage", "Failed to open file %s for writing", filename);
            return ESP_FAIL;
        }

        serializeJson(doc, file);
        file.close();
        ESP_LOGI("EventStorage", "Saved settings to %s", filename);
        return ESP_OK;
    }
};