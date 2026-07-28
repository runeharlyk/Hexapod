#ifndef ServoController_h
#define ServoController_h

#include <esp_log.h>
#include <cstdint>
#include <event_bus.h>
#include <message_types.h>
#include <platform_shared/api.pb.h>
#include <settings/servo_settings.h>
#include <peripherals/i2c_bus.h>
#include <peripherals/drivers/pca9685.h>

#ifndef NUM_SERVO
#define NUM_SERVO 18
#endif

#ifndef FACTORY_SERVO_PWM_FREQUENCY
#define FACTORY_SERVO_PWM_FREQUENCY 50
#endif

#ifndef SDA_PIN
#define SDA_PIN 47
#endif
#ifndef SCL_PIN
#define SCL_PIN 21
#endif

struct ServoCfg {
    uint8_t pin;
    float centerPwm;
    float conversion;
    float direction;
    float centerAngle;
};

enum class SERVO_CONTROL_STATE { DEACTIVATED, PWM, ANGLE };

class ServoController {
  public:
    ServoController() : _left_pca(0x40), _right_pca(0x41) {}

    void begin() {
        if (!I2CBus::instance().isInitialized()) {
            I2CBus::instance().begin(static_cast<gpio_num_t>(SDA_PIN), static_cast<gpio_num_t>(SCL_PIN), 400000);
        }
        _left_pca.begin();
        _left_pca.setPWMFreq(FACTORY_SERVO_PWM_FREQUENCY);
        _right_pca.begin();
        _right_pca.setPWMFreq(FACTORY_SERVO_PWM_FREQUENCY);

        _signalSub = EventBus<ServoSignalMsg>::subscribe([this](ServoSignalMsg const &m) { servoEvent(m); });
        _stateSub = EventBus<ServoStateMsg>::subscribe([this](ServoStateMsg const &m) {
            m.active ? activate() : deactivate();
        });
        _calibrationSub = ServoSettingsBus::subscribe([this](api_ServoSettings const &s) { applyCalibration(s); });
        api_ServoSettings pending;
        if (ServoSettingsBus::peek(pending)) applyCalibration(pending);  // if the service published first
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

    void servoEvent(ServoSignalMsg const &msg) {
        control_state = SERVO_CONTROL_STATE::PWM;
        pcaWrite(msg.id, msg.pwm);
    }

    void setAngles(float new_angles[NUM_SERVO]) {
        control_state = SERVO_CONTROL_STATE::ANGLE;
        for (int i = 0; i < NUM_SERVO; i++) {
            target_angles[i] = new_angles[i] + (i % 3 == 2 ? 90.0f : 0.0f);
        }
    }

    void calculatePWM() {
        for (int i = 0; i < 9; i++) {
            const ServoCfg &sl = cfg[i];
            const ServoCfg &sr = cfg[i + 9];

            left_pwm[sl.pin] = (uint16_t)((target_angles[i] + sl.centerAngle) * sl.direction * sl.conversion + sl.centerPwm);
            right_pwm[sr.pin] =
                (uint16_t)((target_angles[i + 9] + sr.centerAngle) * sr.direction * sr.conversion + sr.centerPwm);

            if (sl.pin > max_pin_used) max_pin_used = sl.pin;
            if (sr.pin > max_pin_used) max_pin_used = sr.pin;
        }
        _left_pca.setMultiplePWM(left_pwm, max_pin_used + 1);
        _right_pca.setMultiplePWM(right_pwm, max_pin_used + 1);
    }

    // Pins are hardware; only the angle->PWM fields come from the proto.
    void applyCalibration(const api_ServoSettings &s) {
        size_t n = s.servos_count < NUM_SERVO ? s.servos_count : NUM_SERVO;
        for (size_t i = 0; i < n; i++) {
            cfg[i].centerPwm = s.servos[i].center_pwm;
            cfg[i].conversion = s.servos[i].conversion;
            cfg[i].direction = s.servos[i].direction;
            cfg[i].centerAngle = s.servos[i].center_angle;
        }
    }

    void setCenterPwm() {
        for (int i = 0; i < 9; i++) {
            left_pwm[cfg[i].pin] = cfg[i].centerPwm;
            right_pwm[cfg[i + 9].pin] = cfg[i + 9].centerPwm;
        }
        _left_pca.setMultiplePWM(left_pwm, max_pin_used + 1);
        _right_pca.setMultiplePWM(right_pwm, max_pin_used + 1);
    }

    void updateServoState() {
        if (control_state == SERVO_CONTROL_STATE::ANGLE) calculatePWM();
    }

  private:
    void pcaWrite(int index, int value) {
        if (value < 0 || value > 4096) {
            ESP_LOGE(TAG, "Invalid PWM %d for servo %d (0-4096)", value, index);
            return;
        }
        int pin = cfg[index].pin;
        index < 9 ? _left_pca.setPWM(pin, 0, value) : _right_pca.setPWM(pin, 0, value);
    }

    static constexpr const char *TAG = "ServoController";

    // direction defaults reproduce the former l_dir{-1,-1,1}/r_dir{-1,1,-1} so motion is unchanged.
    ServoCfg cfg[NUM_SERVO] = {{8, 306, 2, -1, 0},  {9, 306, 2, -1, 0}, {10, 306, 2, 1, 0},
                               {2, 306, 2, -1, 0},  {3, 306, 2, -1, 0}, {4, 306, 2, 1, 0},
                               {5, 306, 2, -1, 0},  {6, 306, 2, -1, 0}, {7, 306, 2, 1, 0},
                               {8, 306, 2, -1, 0},  {9, 306, 2, 1, 0},  {10, 306, 2, -1, 0},
                               {2, 306, 2, -1, 0},  {3, 306, 2, 1, 0},  {4, 306, 2, -1, 0},
                               {5, 306, 2, -1, 0},  {6, 306, 2, 1, 0},  {7, 306, 2, -1, 0}};

    PCA9685Driver _left_pca;
    PCA9685Driver _right_pca;

    uint16_t left_pwm[16] = {0};
    uint16_t right_pwm[16] = {0};
    uint8_t max_pin_used = 0;

    SERVO_CONTROL_STATE control_state = SERVO_CONTROL_STATE::DEACTIVATED;
    bool is_active{false};
    float target_angles[NUM_SERVO] = {0};

    EventBus<ServoSignalMsg>::Handle _signalSub;
    EventBus<ServoStateMsg>::Handle _stateSub;
    ServoSettingsBus::Handle _calibrationSub;
};

#endif
