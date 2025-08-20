#ifndef ServoController_h
#define ServoController_h

#include <Adafruit_PWMServoDriver.h>

#include <event_bus.h>
#include <utils/math_utils.h>
#include <message_types.h>

/*
 * Servo Settings
 */
#ifndef FACTORY_SERVO_PWM_FREQUENCY
#define FACTORY_SERVO_PWM_FREQUENCY 50
#endif

#ifndef FACTORY_SERVO_OSCILLATOR_FREQUENCY
#define FACTORY_SERVO_OSCILLATOR_FREQUENCY 27000000
#endif

#ifndef FACTORY_SERVO_CENTER_ANGLE
#define FACTORY_SERVO_CENTER_ANGLE 90
#endif

enum class SERVO_CONTROL_STATE { DEACTIVATED, PWM, ANGLE };

class ServoController {
  public:
    ServoController() : _left_pca {0x40}, _right_pca {0x41} {}

    void begin() {
        EventBus<ServoSignalMsg>::consume([&](ServoSignalMsg const &msg) { servoEvent(msg); });
        EventBus<ServoSettingsMsg>::consume([](const ServoSettingsMsg &s) { cfg = s; });
        EventBus<ServoSettingsMsg>::peek(cfg);
        initializePCA();
    }

    void pcaWrite(int index, int value) {
        if (value < 0 || value > 4096) {
            ESP_LOGE("Peripherals", "Invalid PWM value %d for %d :: Valid range 0-4096", value, index);
            return;
        }
        int pin = cfg.servos[index].pin;
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
        updateActiveState();
    }

    void servoEvent(ServoSignalMsg const &msg) {
        control_state = SERVO_CONTROL_STATE::PWM;
        pcaWrite(msg.id, msg.pwm);
        ESP_LOGI("SERVO_CONTROLLER", "Setting servo %d to %d", msg.id, msg.pwm);
    }

    void updateActiveState() { is_active ? activate() : deactivate(); }

    void setAngles(float new_angles[18]) {
        control_state = SERVO_CONTROL_STATE::ANGLE;
        for (int i = 0; i < 18; i++) {
            target_angles[i] = new_angles[i] + (i % 3 == 2 ? 90.0f : 0.0f);
        }
    }

    void calculatePWM() {
        int8_t l_dir[3] = {-1, -1, 1};
        int8_t r_dir[3] = {-1, 1, -1};

        for (int i = 0; i < 9; i++) {
            auto servoLeft = cfg.servos[i];
            auto servoRight = cfg.servos[i + 9];

            left_target_pwms[i] = (target_angles[i] * l_dir[i % 3]) * servoLeft.conversion + servoLeft.centerPwm;
            right_target_pwms[i] = (target_angles[i + 9] * r_dir[i % 3]) * servoRight.conversion + servoRight.centerPwm;

            left_current_pwm[i] = left_target_pwms[i];
            right_current_pwm[i] = right_target_pwms[i];

            left_pwm[servoLeft.pin] = left_current_pwm[i];
            right_pwm[servoRight.pin] = right_current_pwm[i];

            if (servoLeft.pin > max_pin_used) max_pin_used = servoLeft.pin;
            if (servoRight.pin > max_pin_used) max_pin_used = servoRight.pin;
        }
        _left_pca.setMultiplePWM(left_pwm, max_pin_used + 1);
        _right_pca.setMultiplePWM(right_pwm, max_pin_used + 1);
    }

    void setCenterPwm() {
        for (int i = 0; i < 9; i++) {
            auto servoLeft = cfg.servos[i];
            auto servoRight = cfg.servos[i + 9];
            left_pwm[servoLeft.pin] = servoLeft.centerPwm;
            left_pwm[servoRight.pin] = servoRight.centerPwm;
        }
        _left_pca.setMultiplePWM(left_pwm, max_pin_used + 1);
        _right_pca.setMultiplePWM(right_pwm, max_pin_used + 1);
    }

    void updateServoState() {
        if (control_state == SERVO_CONTROL_STATE::ANGLE) calculatePWM();
    }

  private:
    void initializePCA() {
        _left_pca.begin();
        _left_pca.setPWMFreq(FACTORY_SERVO_PWM_FREQUENCY);
        _right_pca.begin();
        _right_pca.setPWMFreq(FACTORY_SERVO_PWM_FREQUENCY);
    }

    inline static ServoSettingsMsg cfg;

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
    float target_angles[18] = {0};
};

#endif