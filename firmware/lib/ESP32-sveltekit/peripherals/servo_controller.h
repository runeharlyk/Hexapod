#ifndef ServoController_h
#define ServoController_h

#include <Adafruit_PWMServoDriver.h>
#include <event_socket.h>
#include <template/stateful_persistence.h>
#include <template/stateful_service.h>
#include <template/stateful_endpoint.h>
#include <utils/math_utils.h>
#include <settings/servo_settings.h>

/*
 * Servo Settings
 */
#ifndef FACTORY_SERVO_PWM_FREQUENCY
#define FACTORY_SERVO_PWM_FREQUENCY 50
#endif

#ifndef FACTORY_SERVO_OSCILLATOR_FREQUENCY
#define FACTORY_SERVO_OSCILLATOR_FREQUENCY 27000000
#endif

#define EVENT_SERVO_PWM "servoPWM"
#define EVENT_SERVO_ANGLE "servoAngle"
#define EVENT_SERVO_STATE "servoState"
#define EVENT_SERVO_SETTINGS "servoConfiguration"

enum class SERVO_CONTROL_STATE { DEACTIVATED, PWM, ANGLE };

// int hexapod2pwm(int legIndex, int partIndex) {
//     switch (legIndex) {
//         case 0: return 5 + partIndex;
//         case 1: return 2 + partIndex;
//         case 2: return 8 + partIndex;
//         case 3: return 16 + 8 + partIndex;
//         case 4: return 16 + 2 + partIndex;
//         case 5: return 16 + 5 + partIndex;
//         default: return 0;
//     }
// }

class ServoController : public StatefulService<ServoSettings> {
  public:
    ServoController()
        : endpoint(ServoSettings::read, ServoSettings::update, this),
          _eventEndpoint(ServoSettings::read, ServoSettings::update, this, EVENT_SERVO_SETTINGS),
          _persistence(ServoSettings::read, ServoSettings::update, this, SERVO_SETTINGS_FILE),
          _pcaLeft(0x41),
          _pcaRight(0x40) {}

    void begin() {
        socket.onEvent(EVENT_SERVO_PWM, [&](JsonObject &root, int originId) { servoEvent(root, originId); });
        socket.onEvent(EVENT_SERVO_ANGLE, [&](JsonObject &root, int originId) { servoAngleEvent(root, originId); });
        socket.onEvent(EVENT_SERVO_STATE, [&](JsonObject &root, int originId) { stateUpdate(root, originId); });
        _persistence.readFromFS();
        _eventEndpoint.begin();

        initializePCA();
        socket.onEvent(EVENT_SERVO_STATE, [&](JsonObject &root, int originId) {
            is_active = root["active"] | false;
            is_active ? activate() : deactivate();
        });
    }

    void pcaWrite(int index, int value) {
        if (value < 0 || value > 4096) {
            ESP_LOGE("Peripherals", "Invalid PWM value %d for %d :: Valid range 0-4096", value, index);
            return;
        }
        index < 9 ? _pcaLeft.setPWM(index + 2, 0, value) : _pcaRight.setPWM(index - 7, 0, value);
    }

    void pcaWriteAngle(int index, int value) {
        auto &servo = state().servos[index];
        float angle = servo.direction * value + servo.centerAngle;
        uint16_t pwm = angle * servo.conversion + servo.centerPwm;
        if (pwm < 120 || pwm > 600) {
            ESP_LOGE("ServoController", "Servo %d, Invalid PWM value %d", index, pwm);
            return;
        }
        pcaWrite(index, pwm);
    }

    void activate() {
        if (is_active) return;
        control_state = SERVO_CONTROL_STATE::ANGLE;
        is_active = true;
        _pcaLeft.wakeup();
        _pcaRight.wakeup();
    }

    void deactivate() {
        if (!is_active) return;
        is_active = false;
        control_state = SERVO_CONTROL_STATE::DEACTIVATED;
        _pcaLeft.sleep();
        _pcaRight.sleep();
    }

    void stateUpdate(JsonObject &root, int originId) {
        bool active = root["active"].as<bool>();
        ESP_LOGI("SERVOCONTROLLER", "Setting state %d", active);
        active ? activate() : deactivate();
    }

    void servoEvent(JsonObject &root, int originId) {
        control_state = SERVO_CONTROL_STATE::PWM;
        uint8_t servo_id = root["servo_id"];
        int pwm = root["pwm"].as<int>();
        pcaWrite(servo_id, pwm);
        ESP_LOGI("SERVO_CONTROLLER", "Setting servo %d to %d", servo_id, pwm);
    }

    void servoAngleEvent(JsonObject &root, int originId) {
        control_state = SERVO_CONTROL_STATE::PWM;
        uint8_t servo_id = root["servo_id"];
        int angle = root["angle"].as<int>();
        pcaWriteAngle(servo_id, angle);
        ESP_LOGI("SERVO_CONTROLLER", "Setting servo %d to %d degrees", servo_id, angle);
    }

    void syncAngles(const String &originId) {
        char output[100];
        snprintf(output, sizeof(output),
                 "[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]",
                 angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], angles[6], angles[7], angles[8],
                 angles[9], angles[10], angles[11], angles[12], angles[13], angles[14], angles[15], angles[16],
                 angles[17]);
        socket.emit("angles", output, String(originId).c_str());
    }

    void updateActiveState() { is_active ? activate() : deactivate(); }

    void setAngles(float new_angles[12]) {
        control_state = SERVO_CONTROL_STATE::ANGLE;
        for (int i = 0; i < 18; i++) {
            target_angles[i] = new_angles[i];
        }
    }

    void calculatePWM() {
        for (int i = 0; i < 18; i++) {
            angles[i] = lerp(angles[i], target_angles[i], 0.2);
            auto &servo = state().servos[i];
            float angle = servo.direction * angles[i] + servo.centerAngle;
            uint16_t pwm = angle * servo.conversion + servo.centerPwm;
            if (pwm < 120 || pwm > 600) {
                ESP_LOGE("ServoController", "Servo %d, Invalid PWM value %d", i, pwm);
                continue;
            }
            pcaWrite(i, pwm);
        }
    }

    void updateServoState() {
        if (control_state == SERVO_CONTROL_STATE::ANGLE) calculatePWM();
    }

    StatefulHttpEndpoint<ServoSettings> endpoint;

  private:
    void initializePCA() {
        _pcaLeft.begin();
        _pcaLeft.setPWMFreq(FACTORY_SERVO_PWM_FREQUENCY);
        _pcaLeft.setOscillatorFrequency(FACTORY_SERVO_OSCILLATOR_FREQUENCY);
        _pcaLeft.sleep();

        _pcaRight.begin();
        _pcaRight.setPWMFreq(FACTORY_SERVO_PWM_FREQUENCY);
        _pcaRight.setOscillatorFrequency(FACTORY_SERVO_OSCILLATOR_FREQUENCY);
        _pcaRight.sleep();
    }
    FSPersistence<ServoSettings> _persistence;
    EventEndpoint<ServoSettings> _eventEndpoint;

    Adafruit_PWMServoDriver _pcaLeft;
    Adafruit_PWMServoDriver _pcaRight;

    SERVO_CONTROL_STATE control_state = SERVO_CONTROL_STATE::DEACTIVATED;

    bool is_active {false};
    float angles[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    float target_angles[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
};

#endif