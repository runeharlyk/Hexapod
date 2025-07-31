#ifndef Features_h
#define Features_h

#include <WiFi.h>
#include <ArduinoJson.h>
#include <PsychicHttp.h>

#define FT_ENABLED(feature) feature

// ESP32 IMU on by default
#ifndef USE_MPU6050
#define USE_MPU6050 1
#endif

// ESP32 CAMERA off by default
#ifndef USE_CAMERA
#define USE_CAMERA 0
#endif

// ESP32 IMU on by default
#ifndef USE_BNO055
#define USE_BNO055 0
#endif

// ESP32 magnetometer on by default
#ifndef USE_MAG
#define USE_MAG 0
#endif

// ESP32 MDNS on by default
#ifndef USE_MDNS
#define USE_MDNS 1
#endif

namespace feature_service {

void printFeatureConfiguration();

void features(JsonObject &root);

esp_err_t getFeatures(PsychicRequest *request);

} // namespace feature_service

#endif
