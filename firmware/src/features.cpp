#include <feature_flags.h>

#include <esp_log.h>

#ifndef APP_NAME
#define APP_NAME "Hexapod"
#endif
#ifndef APP_VERSION
#define APP_VERSION "0.0.1"
#endif

namespace feature_service {

void printFeatureConfiguration() {
    ESP_LOGI("Features", "====================== FEATURE FLAGS ======================");
    ESP_LOGI("Features", "Firmware version: %s, name: %s", APP_VERSION, APP_NAME);
    ESP_LOGI("Features", "USE_CAMERA:  %s", USE_CAMERA ? "enabled" : "disabled");
    ESP_LOGI("Features", "USE_MPU6050: %s", USE_MPU6050 ? "enabled" : "disabled");
    ESP_LOGI("Features", "USE_MAG:     %s", USE_MAG ? "enabled" : "disabled");
    ESP_LOGI("Features", "USE_MDNS:    %s", USE_MDNS ? "enabled" : "disabled");
    ESP_LOGI("Features", "USE_ESPNOW:  %s", USE_ESPNOW ? "enabled" : "disabled");
    ESP_LOGI("Features", "EMBED_WEBAPP:%s", EMBED_WEBAPP ? "enabled" : "disabled");
    ESP_LOGI("Features", "==========================================================");
}

} // namespace feature_service
