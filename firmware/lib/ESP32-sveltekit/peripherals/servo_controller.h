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

#define EVENT_SERVO_CONFIGURATION_SETTINGS "servoPWM"
#define EVENT_SERVO_STATE "servoState"

enum class SERVO_CONTROL_STATE { DEACTIVATED, PWM, ANGLE };

class ServoController : public StatefulService<ServoSettings> {
  public:
    ServoController()
        : endpoint(ServoSettings::read, ServoSettings::update, this),
          _persistence(ServoSettings::read, ServoSettings::update, this, SERVO_SETTINGS_FILE),
          _left_pca {0x40},
          _right_pca {0x41} {}

    void begin() {
        socket.onEvent(EVENT_SERVO_CONFIGURATION_SETTINGS,
                       [&](JsonObject &root, int originId) { servoEvent(root, originId); });
        socket.onEvent(EVENT_SERVO_STATE, [&](JsonObject &root, int originId) { stateUpdate(root, originId); });
        _persistence.readFromFS();

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
        int pin = state().servos[index].pin;
        if (index < 9) {
            _left_pca.setPWM(pin, 0, value);
        } else {
            _right_pca.setPWM(pin, 0, value);
        }
    }

    void activate() {
        if (is_active) return;
        control_state = SERVO_CONTROL_STATE::ANGLE;
        is_active = true;
        _left_pca.wakeup();
        _right_pca.wakeup();
    }

    void deactivate() {
        if (!is_active) return;
        is_active = false;
        control_state = SERVO_CONTROL_STATE::DEACTIVATED;
        _left_pca.sleep();
        _right_pca.sleep();
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

    void syncAngles(const String &originId) {
        // char output[100];
        // snprintf(output, sizeof(output), "[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]", angles[0],
        //          angles[1], angles[2], angles[3], angles[4], angles[5], angles[6], angles[7], angles[8], angles[9],
        //          angles[10], angles[11]);
        // socket.emit("angles", output, String(originId).c_str());
    }

    void updateActiveState() { is_active ? activate() : deactivate(); }

    void setAngles(float new_angles[12]) {
        control_state = SERVO_CONTROL_STATE::ANGLE;
        for (int i = 0; i < 12; i++) {
            target_angles[i] = new_angles[i];
        }
    }

    void calculatePWM() {
        for (int i = 0; i < 9; i++) {
            auto servoLeft = state().servos[i];
            auto servoRight = state().servos[i + 9];
            left_current_pwm[i] = lerp(left_current_pwm[i], left_target_pwms[i], 0.05);
            right_current_pwm[i] = lerp(right_current_pwm[i], right_target_pwms[i], 0.05);

            left_pwm[servoLeft.pin] = left_current_pwm[i];
            right_pwm[servoRight.pin] = right_current_pwm[i];

            if (servoLeft.direction == -1) {
                left_pwm[servoLeft.pin] = servoLeft.centerPwm - (left_pwm[servoLeft.pin] - servoLeft.centerPwm);
            }
            if (servoRight.direction == -1) {
                right_pwm[servoRight.pin] = servoRight.centerPwm - (right_pwm[servoRight.pin] - servoRight.centerPwm);
            }

            if (servoLeft.pin > max_pin_used) {
                max_pin_used = servoLeft.pin;
            }
            if (servoRight.pin > max_pin_used) {
                max_pin_used = servoRight.pin;
            }
        }
        _left_pca.setMultiplePWM(left_pwm, max_pin_used + 1);
        _right_pca.setMultiplePWM(right_pwm, max_pin_used + 1);
    }

    void getCalibrationPWM(uint16_t left[9], uint16_t right[9]) {
        for (int i = 0; i < 9; i++) {
            auto servoLeft = state().servos[i];
            auto servoRight = state().servos[i + 9];
            left[i] = servoLeft.centerPwm;
            right[i] = servoRight.centerPwm;
        }
    }

    void updateServoState() {
        if (control_state == SERVO_CONTROL_STATE::ANGLE) calculatePWM();
    }

    void updateLeftPwm(uint16_t pwm[9]) { memcpy(left_target_pwms, pwm, sizeof(uint16_t) * 9); }

    void updateRightPwm(uint16_t pwm[9]) { memcpy(right_target_pwms, pwm, sizeof(uint16_t) * 9); }

    StatefulHttpEndpoint<ServoSettings> endpoint;

  private:
    void initializePCA() {
        _left_pca.begin();
        _left_pca.setPWMFreq(FACTORY_SERVO_PWM_FREQUENCY);
        // _left_pca.setOscillatorFrequency(FACTORY_SERVO_OSCILLATOR_FREQUENCY);
        // _left_pca.sleep();
        _right_pca.begin();
        _right_pca.setPWMFreq(FACTORY_SERVO_PWM_FREQUENCY);
        // _right_pca.setOscillatorFrequency(FACTORY_SERVO_OSCILLATOR_FREQUENCY);
        // _right_pca.sleep();
    }
    FSPersistence<ServoSettings> _persistence;

    Adafruit_PWMServoDriver _left_pca;
    Adafruit_PWMServoDriver _right_pca;

    uint16_t left_target_pwms[9];
    uint16_t right_target_pwms[9];
    uint16_t left_current_pwm[9];
    uint16_t right_current_pwm[9];

    uint16_t left_pwm[16];
    uint16_t right_pwm[16];
    uint8_t max_pin_used = 0;

    SERVO_CONTROL_STATE control_state = SERVO_CONTROL_STATE::DEACTIVATED;

    bool is_active {false};
    // float angles[12] = {0, 90, -145, 0, 90, -145, 0, 90, -145, 0, 90, -145};
    float target_angles[12] = {0, 90, -145, 0, 90, -145, 0, 90, -145, 0, 90, -145};
};

#endif