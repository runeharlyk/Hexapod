#include <features.h>

namespace feature_service {

// New function to print all feature flags to log
void printFeatureConfiguration() {
    ESP_LOGI("Features", "====================== FEATURE FLAGS ======================");
    ESP_LOGI("Features", "Firmware version: %s, name: %s", APP_VERSION, APP_NAME);

    // Sensors
    ESP_LOGI("Features", "USE_MPU6050: %s", USE_MPU6050 ? "enabled" : "disabled");
    ESP_LOGI("Features", "USE_MAG: %s", USE_MAG ? "enabled" : "disabled");

    // Peripherals
    ESP_LOGI("Features", "USE_SERVO: %s", USE_SERVO ? "enabled" : "disabled");

    // Web services
    ESP_LOGI("Features", "USE_MDNS: %s", USE_MDNS ? "enabled" : "disabled");
    ESP_LOGI("Features", "==========================================================");
}

void features(JsonObject &root) {
    root["imu"] = USE_MPU6050;
    root["mag"] = USE_MAG;
    root["servo"] = USE_SERVO;
    root["firmware_version"] = APP_VERSION;
    root["firmware_name"] = APP_NAME;
}

esp_err_t getFeatures(PsychicRequest *request) {
    PsychicJsonResponse response = PsychicJsonResponse(request, false);
    JsonObject root = response.getRoot();
    features(root);
    return response.send();
}

} // namespace feature_service