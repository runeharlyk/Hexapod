#pragma once

#include <string>

// LittleFS mounted at MOUNT_POINT on the "spiffs" partition.
#define MOUNT_POINT "/littlefs"
#define FS_PARTITION_LABEL "spiffs"

#define FS_CONFIG_DIRECTORY MOUNT_POINT "/config"
#define AP_SETTINGS_FILE FS_CONFIG_DIRECTORY "/apSettings.json"
#define WIFI_SETTINGS_FILE FS_CONFIG_DIRECTORY "/wifiSettings.json"
#define SERVO_SETTINGS_FILE FS_CONFIG_DIRECTORY "/servoSettings.json"
#define MDNS_SETTINGS_FILE FS_CONFIG_DIRECTORY "/mdnsSettings.json"
#define PERIPHERAL_SETTINGS_FILE FS_CONFIG_DIRECTORY "/peripheralSettings.json"

namespace FileSystem {

bool init();
bool readFile(const char *path, std::string &out);
bool writeFile(const char *path, const char *content);
bool mkdirRecursive(const char *path);

} // namespace FileSystem
