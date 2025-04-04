#pragma once

#include <ArduinoJson.h>
#include <template/state_result.h>

typedef struct {
    float centerPwm;
    float direction;
    float centerAngle;
    float conversion;
    const char *name;
    // bool enable;
} servo_settings_t;

class ServoSettings {
  public:
    servo_settings_t servos[18] = {{306, 1, 0, 2, "Servo1"},  {306, 1, 0, 2, "Servo2"},  {306, 1, 0, 2, "Servo3"},
                                   {306, 1, 0, 2, "Servo4"},  {306, 1, 0, 2, "Servo5"},  {306, 1, 0, 2, "Servo6"},
                                   {306, 1, 0, 2, "Servo7"},  {306, 1, 0, 2, "Servo8"},  {306, 1, 0, 2, "Servo9"},
                                   {306, 1, 0, 2, "Servo10"}, {306, 1, 0, 2, "Servo11"}, {306, 1, 0, 2, "Servo12"},
                                   {306, 1, 0, 2, "Servo13"}, {306, 1, 0, 2, "Servo14"}, {306, 1, 0, 2, "Servo15"},
                                   {306, 1, 0, 2, "Servo16"}, {306, 1, 0, 2, "Servo17"}, {306, 1, 0, 2, "Servo18"}};
    static void read(ServoSettings &settings, JsonObject &root) {
        JsonArray servos = root["servos"].to<JsonArray>();
        for (auto &servo : settings.servos) {
            JsonObject newServo = servos.add<JsonObject>();
            newServo["center_pwm"] = servo.centerPwm;
            newServo["direction"] = servo.direction;
            newServo["center_angle"] = servo.centerAngle;
            newServo["conversion"] = servo.conversion;
        }
    }
    static StateUpdateResult update(JsonObject &root, ServoSettings &settings) {
        if (root["servos"].is<JsonArray>()) {
            JsonArray servosJson = root["servos"];
            int i = 0;
            for (auto servo : servosJson) {
                JsonObject servoObject = servo.as<JsonObject>();
                uint8_t servoId = i; // servoObject["id"].as<uint8_t>();
                settings.servos[servoId].centerPwm = servoObject["center_pwm"].as<float>();
                settings.servos[servoId].centerAngle = servoObject["center_angle"].as<float>();
                settings.servos[servoId].direction = servoObject["direction"].as<float>();
                settings.servos[servoId].conversion = servoObject["conversion"].as<float>();
                i++;
            }
        }
        ESP_LOGI("ServoController", "Updating servo data");
        return StateUpdateResult::CHANGED;
    };
};