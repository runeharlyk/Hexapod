#pragma once

#include <ArduinoJson.h>
#include <template/state_result.h>

typedef struct {
    float centerPwm;
    float direction;
    float conversion;
    uint8_t pin;
    String name;
    // bool enable;
} servo_settings_t;

class ServoSettings {
  public:
    servo_settings_t servos[18] = {
        {306, 1, 8, 2, "left_1_1"},  {306, 1, 9, 2, "left_1_2"},  {306, 1, 10, 2, "left_1_3"},
        {306, 1, 2, 2, "left_2_1"},  {306, 1, 3, 2, "left_2_2"},  {306, 1, 4, 2, "left_2_3"},
        {306, 1, 5, 2, "left_3_1"},  {306, 1, 6, 2, "left_3_2"},  {306, 1, 7, 2, "left_3_3"},
        {306, 1, 8, 2, "right_1_1"}, {306, 1, 9, 2, "right_1_2"}, {306, 1, 10, 2, "right_1_3"},
        {306, 1, 2, 2, "right_2_1"}, {306, 1, 3, 2, "right_2_2"}, {306, 1, 4, 2, "right_2_3"},
        {306, 1, 5, 2, "right_3_1"}, {306, 1, 6, 2, "right_3_2"}, {306, 1, 7, 2, "right_3_3"}};
    static void read(ServoSettings &settings, JsonObject &root) {
        JsonArray servos = root["servos"].to<JsonArray>();
        for (auto &servo : settings.servos) {
            JsonObject newServo = servos.add<JsonObject>();
            newServo["name"] = servo.name;
            newServo["center_pwm"] = servo.centerPwm;
            newServo["direction"] = servo.direction;
            newServo["pin"] = servo.pin;
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
                settings.servos[servoId].name = servoObject[""].as<String>();
                settings.servos[servoId].centerPwm = servoObject["center_pwm"].as<float>();
                settings.servos[servoId].pin = servoObject["pin"].as<uint8_t>();
                settings.servos[servoId].direction = servoObject["direction"].as<float>();
                settings.servos[servoId].conversion = servoObject["conversion"].as<float>();
                i++;
            }
        }
        ESP_LOGI("ServoController", "Updating servo data");
        return StateUpdateResult::CHANGED;
    };
};