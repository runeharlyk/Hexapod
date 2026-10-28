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
#define IMU_CALIBRATION_FILE FS_CONFIG_DIRECTORY "/imuCalibration.pb"
#define CAMERA_SETTINGS_FILE FS_CONFIG_DIRECTORY "/cameraSettings.pb"

namespace FileSystem {

bool init();
bool readFile(const char *path, std::string &out);
bool writeFile(const char *path, const char *content);
bool mkdirRecursive(const char *path);
// Moves `from` over `to` in one step, so a reset leaves one of the two whole. Removing `to` first and retrying would
// lose it whenever the move fails for a reason that persists, such as a full file system.
bool replaceFile(const char *from, const char *to);

} // namespace FileSystem
