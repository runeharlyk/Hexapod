#pragma once
#include <ArduinoJson.h>

#include <event_bus.h>
#include <message_types.h>
#include <filesystem.h>
#include <settings/servo_settings.h>

#define SERVO_SETTINGS_JSON_FILE "/config/ServoSettingsMsg.json"
#define WIFI_SETTINGS_JSON_FILE "/config/WiFiSettingsMsg.json"

class EventStorage {
  public:
    EventStorage() {}
    ~EventStorage() {}

    void begin() {
        registerEvent<ServoSettingsMsg>(SERVO_SETTINGS_JSON_FILE);
        registerEvent<WiFiSettingsMsg>(WIFI_SETTINGS_JSON_FILE);
    }

  private:
    template <typename T>
    void registerEvent(const char* filename) {
        loadSettings<T>(filename);
        EventBus<T>::consume([this, filename](const T& t) { save(t, filename); });
    }

    template <typename T>
    esp_err_t loadSettings(const char* filename) {
        File file = ESP_FS.open(filename, "r");
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
        save(settings, filename);
        return ESP_OK;
    }

    template <typename T>
    esp_err_t save(const T& eventData, const char* filename) {
        JsonDocument doc;
        toJson(doc, eventData);

        File file = ESP_FS.open(filename, "w");
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