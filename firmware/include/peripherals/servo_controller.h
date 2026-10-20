#ifndef ServoController_h
#define ServoController_h

#include <esp_log.h>
#include <esp_timer.h>
#include <cmath>
#include <cstdint>
#include <event_bus.h>
#include <message_types.h>
#include <platform_shared/api.pb.h>
#include <platform_shared/message.pb.h>
#include <settings/servo_settings.h>
#include <peripherals/i2c_bus.h>
#include <peripherals/servo_board.h>
#include <utils/math_utils.h>
#include <kinematics.h>

#ifndef NUM_SERVO
#define NUM_SERVO 18
#endif

#ifndef FACTORY_SERVO_OSCILLATOR_FREQUENCY
#define FACTORY_SERVO_OSCILLATOR_FREQUENCY 25000000
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
    // The PCA9685 channel span is a property of the wiring, not of whether calculatePWM() has run yet.
    ServoController()
        : _left(0x40, FACTORY_SERVO_OSCILLATOR_FREQUENCY, FACTORY_SERVO_PWM_FREQUENCY),
          _right(0x41, FACTORY_SERVO_OSCILLATOR_FREQUENCY, FACTORY_SERVO_PWM_FREQUENCY) {
        for (const ServoCfg &servo : cfg) {
            if (servo.pin + 1 > channel_count) channel_count = servo.pin + 1;
        }
    }

    void begin() {
        if (!I2CBus::instance().isInitialized()) {
            I2CBus::instance().begin(static_cast<gpio_num_t>(SDA_PIN), static_cast<gpio_num_t>(SCL_PIN), 400000);
        }
        const int64_t now = esp_timer_get_time();
        if (!_left.begin(now)) ESP_LOGW(TAG, "servo board 0x%02X does not answer", _left.address());
        if (!_right.begin(now)) ESP_LOGW(TAG, "servo board 0x%02X does not answer", _right.address());

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
        _left.wakeup();
        _right.wakeup();
    }

    void deactivate() {
        if (!is_active) return;
        is_active = false;
        control_state = SERVO_CONTROL_STATE::DEACTIVATED;
        _left.sleep();
        _right.sleep();
    }

    void servoEvent(ServoSignalMsg const &msg) {
        control_state = SERVO_CONTROL_STATE::PWM;
        pcaWrite(msg.id, msg.pwm);
    }

    void setAngles(float new_angles[NUM_SERVO]) {
        control_state = SERVO_CONTROL_STATE::ANGLE;
        for (int i = 0; i < NUM_SERVO; i++) {
            // Last line of defence: a NaN or infinity from any mode keeps the servo where it was.
            if (!std::isfinite(new_angles[i])) {
                _nonFiniteSinceLog++;
                continue;
            }
            const int joint = i % 3;
            const float requested = std::fabs(new_angles[i]);
            // Peak is tracked from the PRE-clamp command on every tick, not only when clamping:
            // a run with zero clamps still needs to say how much travel was left.
            if (requested > _peakAbs[joint]) {
                _peakAbs[joint] = requested;
                _peakLeg[joint] = (uint8_t)(i / 3);
            }
            const float angle = withinTravel(new_angles[i], joint);
            if (angle != new_angles[i]) _clampedSinceLog[joint]++;
            target_angles[i] = angle + (joint == 2 ? 90.0f : 0.0f);
        }
        reportJointTravel();
    }

    // Commands beyond travel mean the gait is asking for motion the robot cannot make. Report per
    // joint type -- which joint saturates identifies the cause, and an aggregate cannot -- but not
    // at 200 Hz. Counts are per joint command, so the ceiling is 6 legs x window, not 1 per tick.
    void reportJointTravel() {
        const int64_t now = esp_timer_get_time();
        if (now - _lastClampLog < 2000000) return;
        _lastClampLog = now;
        const uint32_t total = _clampedSinceLog[0] + _clampedSinceLog[1] + _clampedSinceLog[2];
        if (_nonFiniteSinceLog) {
            ESP_LOGE(TAG, "%lu non-finite joint commands ignored", (unsigned long)_nonFiniteSinceLog);
            _nonFiniteSinceLog = 0;
        }
        if (total) {
            ESP_LOGW(TAG,
                     "joint travel exceeded: coxa %lu femur %lu tibia %lu commands clamped; "
                     "peak |cmd| coxa %.1f (leg %u) femur %.1f (leg %u) tibia %.1f (leg %u) deg",
                     (unsigned long)_clampedSinceLog[0], (unsigned long)_clampedSinceLog[1],
                     (unsigned long)_clampedSinceLog[2], _peakAbs[0], (unsigned)_peakLeg[0],
                     _peakAbs[1], (unsigned)_peakLeg[1], _peakAbs[2], (unsigned)_peakLeg[2]);
        } else {
            ESP_LOGD(TAG, "joint travel ok; peak |cmd| coxa %.1f/%.1f femur %.1f/%.1f tibia %.1f/%.1f deg",
                     _peakAbs[0], JOINT_LIMIT_DEG[0], _peakAbs[1], JOINT_LIMIT_DEG[1], _peakAbs[2],
                     JOINT_LIMIT_DEG[2]);
        }
        for (int j = 0; j < 3; j++) {
            _clampedSinceLog[j] = 0;
            _peakAbs[j] = 0.0f;
        }
    }

    // Float -> uint16_t is undefined for negatives, and the manual pcaWrite() path already
    // validates this range; the gait path did not.
    static uint16_t toPwm(float angle, const ServoCfg &cfg) {
        const float pwm = (angle + cfg.centerAngle) * cfg.direction * cfg.conversion + cfg.centerPwm;
        return (uint16_t)CLIP(pwm, 0.0f, 4095.0f);
    }

    void calculatePWM() {
        for (int i = 0; i < 9; i++) {
            const ServoCfg &sr = cfg[i];
            const ServoCfg &sl = cfg[i + 9];

            right_pwm[sr.pin] = toPwm(target_angles[i], sr);
            left_pwm[sl.pin] = toPwm(target_angles[i + 9], sl);
        }
        writeBoards();
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

    // Control task, IDLE: one joint dragged in the app, through the same travel, calibration and direction as the
    // motion's angles, written to its own channel alone, so every other servo holds its pulse. It is its board's write
    // of the tick, backed off and reported as the motion's are.
    void setJointAngle(int servo, float angleDeg) {
        if (servo < 0 || servo >= NUM_SERVO || !std::isfinite(angleDeg)) return;
        const int joint = servo % 3;
        target_angles[servo] = withinTravel(angleDeg, joint) + (joint == 2 ? 90.0f : 0.0f);
        const ServoCfg &c = cfg[servo];
        const bool right = servo < 9;
        uint16_t *pwm = right ? right_pwm : left_pwm;
        pwm[c.pin] = toPwm(target_angles[servo], c);
        ServoBoard &board = right ? _right : _left;
        const int64_t start = esp_timer_get_time();
        const bool wasAnswering = board.answering();
        const ServoBoard::Write write = board.writeChannel(pwm, channel_count, c.pin, start);
        _writeUs += static_cast<uint32_t>(esp_timer_get_time() - start);
        (right ? _rightWrite : _leftWrite) = report(board, wasAnswering, write);
    }

    // The joint angle whose pulse is the servo's calibrated centre (toPwm's inverse at centerPwm): the calibration's
    // centre angle negated, less the tibia's square knee.
    float centreAngle(int servo) const { return -cfg[servo].centerAngle - (servo % 3 == 2 ? 90.0f : 0.0f); }

    // Control task, IDLE: every servo to its calibrated centre pulse, the pose the calibration page lines joints up in.
    void setCenterPwm() {
        for (int i = 0; i < 9; i++) {
            right_pwm[cfg[i].pin] = cfg[i].centerPwm;
            left_pwm[cfg[i + 9].pin] = cfg[i + 9].centerPwm;
        }
        writeBoards();
    }

    void updateServoState() {
        if (control_state == SERVO_CONTROL_STATE::ANGLE) calculatePWM();
    }

    // Control task, before the tick's writes: a tick that writes nothing reports no write.
    void startTick() {
        _rightWrite = _leftWrite = socket_message_ServoWrite_SERVO_WRITE_NONE;
        _writeUs = 0;
    }

    // Control task, after the tick's writes: what they did, and the pulses they wrote or meant to write in servo order.
    void fillTelemetry(socket_message_TickSample &tick) const {
        tick.servo_write_right = _rightWrite;
        tick.servo_write_left = _leftWrite;
        tick.servo_write_us = _writeUs;
        for (int i = 0; i < NUM_SERVO; i++) {
            const bool right = i < 9;
            if ((right ? _rightWrite : _leftWrite) == socket_message_ServoWrite_SERVO_WRITE_NONE) continue;
            tick.pwm[i] = (right ? right_pwm : left_pwm)[cfg[i].pin];
        }
    }

  private:
    void writeBoards() {
        const int64_t start = esp_timer_get_time();
        const bool leftWasAnswering = _left.answering();
        const bool rightWasAnswering = _right.answering();
        const ServoBoard::Write left = _left.write(left_pwm, channel_count, start);
        const ServoBoard::Write right = _right.write(right_pwm, channel_count, start);
        // The log lines of a transition come after the timestamp: console time is not bus time.
        _writeUs += static_cast<uint32_t>(esp_timer_get_time() - start);
        _leftWrite = report(_left, leftWasAnswering, left);
        _rightWrite = report(_right, rightWasAnswering, right);
    }

    socket_message_ServoWrite report(const ServoBoard &board, bool wasAnswering, ServoBoard::Write write) {
        if (board.answering() != wasAnswering) {
            if (wasAnswering) {
                ESP_LOGW(TAG, "servo board 0x%02X stopped answering; retrying every %lld ms", board.address(),
                         (long long)(WriteBackoff::RETRY_INTERVAL_US / 1000));
            } else {
                ESP_LOGI(TAG, "servo board 0x%02X answers again", board.address());
            }
        }
        switch (write) {
            case ServoBoard::Write::OK: return socket_message_ServoWrite_SERVO_WRITE_OK;
            case ServoBoard::Write::FAILED: return socket_message_ServoWrite_SERVO_WRITE_FAILED;
            case ServoBoard::Write::SKIPPED: return socket_message_ServoWrite_SERVO_WRITE_SKIPPED;
            case ServoBoard::Write::RESTARTED: return socket_message_ServoWrite_SERVO_WRITE_RESTARTED;
        }
        return socket_message_ServoWrite_SERVO_WRITE_NONE;
    }

    void pcaWrite(int index, int value) {
        if (index < 0 || index >= NUM_SERVO) {
            ESP_LOGE(TAG, "Invalid servo id %d (0-%d)", index, NUM_SERVO - 1);
            return;
        }
        if (value < 0 || value > 4096) {
            ESP_LOGE(TAG, "Invalid PWM %d for servo %d (0-4096)", value, index);
            return;
        }
        int pin = cfg[index].pin;
        index < 9 ? _right.setPWM(pin, value) : _left.setPWM(pin, value);
    }

    static constexpr const char *TAG = "ServoController";

    // Servos 0-8 are legs 0-2, the right side, on board 0x41; servos 9-17 are the left legs on 0x40
    // (docs/conventions.md). The femur and tibia servos are mounted mirrored across the body, so their
    // directions differ by side; the coxa servos are not, and a positive coxa angle turns every leg
    // counter-clockwise seen from above.
    // One leg per row: coxa, femur, tibia.
    // clang-format off
    ServoCfg cfg[NUM_SERVO] = {{8, 306, 2, 1, 0}, {9, 306, 2, 1, 0},  {10, 306, 2, -1, 0},
                               {2, 306, 2, 1, 0}, {3, 306, 2, 1, 0},  {4, 306, 2, -1, 0},
                               {5, 306, 2, 1, 0}, {6, 306, 2, 1, 0},  {7, 306, 2, -1, 0},
                               {8, 306, 2, 1, 0}, {9, 306, 2, -1, 0}, {10, 306, 2, 1, 0},
                               {2, 306, 2, 1, 0}, {3, 306, 2, -1, 0}, {4, 306, 2, 1, 0},
                               {5, 306, 2, 1, 0}, {6, 306, 2, -1, 0}, {7, 306, 2, 1, 0}};
    // clang-format on

    ServoBoard _left;
    ServoBoard _right;

    uint16_t left_pwm[16] = {0};
    uint16_t right_pwm[16] = {0};
    uint8_t channel_count = 0;

    SERVO_CONTROL_STATE control_state = SERVO_CONTROL_STATE::DEACTIVATED;
    bool is_active{false};
    int64_t _lastClampLog{0};
    // Per joint type: coxa, femur, tibia. Reset each log window.
    uint32_t _clampedSinceLog[3] = {0, 0, 0};
    uint32_t _nonFiniteSinceLog = 0;
    float _peakAbs[3] = {0.0f, 0.0f, 0.0f};
    uint8_t _peakLeg[3] = {0, 0, 0};
    float target_angles[NUM_SERVO] = {0};
    // This tick's write, kept for its telemetry.
    socket_message_ServoWrite _rightWrite = socket_message_ServoWrite_SERVO_WRITE_NONE;
    socket_message_ServoWrite _leftWrite = socket_message_ServoWrite_SERVO_WRITE_NONE;
    uint32_t _writeUs = 0;

    EventBus<ServoSignalMsg>::Handle _signalSub;
    EventBus<ServoStateMsg>::Handle _stateSub;
    ServoSettingsBus::Handle _calibrationSub;
};

#endif
