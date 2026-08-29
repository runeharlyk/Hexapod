#pragma once

#include <string>

// LittleFS mounted at MOUNT_POINT on the "spiffs" partition.
#define MOUNT_POINT "/littlefs"
#define FS_PARTITION_LABEL "spiffs"

#define FS_CONFIG_DIRECTORY MOUNT_POINT "/config"
#define AP_SETTINGS_FILE FS_CONFIG_DIRECTORY "/apSettings.json"
#define CAMERA_SETTINGS_FILE FS_CONFIG_DIRECTORY "/cameraSettings.json"
#define DEVICE_CONFIG_FILE FS_CONFIG_DIRECTORY "/peripheral.json"
#define WIFI_SETTINGS_FILE FS_CONFIG_DIRECTORY "/wifiSettings.json"
#define SERVO_SETTINGS_FILE FS_CONFIG_DIRECTORY "/servoSettings.json"
#define MDNS_SETTINGS_FILE FS_CONFIG_DIRECTORY "/mdnsSettings.json"
#define PERIPHERAL_SETTINGS_FILE FS_CONFIG_DIRECTORY "/peripheralSettings.json"
#define BLUETOOTH_SETTINGS_FILE FS_CONFIG_DIRECTORY "/bluetoothSettings.json"

namespace FileSystem {

bool init();
bool exists(const char *path);
bool readFile(const char *path, std::string &out);
bool writeFile(const char *path, const char *content);
bool mkdirRecursive(const char *path);

} // namespace FileSystem
