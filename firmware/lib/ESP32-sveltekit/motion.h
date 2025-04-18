#ifndef MotionService_h
#define MotionService_h

#include <event_socket.h>
#include <kinematics.h>
#include <peripherals/servo_controller.h>
#include <utils/timing.h>
#include <utils/math_utils.h>

#include <gait/state.h>
#include <gait/crawl_state.h>
#include <gait/bezier_state.h>

#include "config.h"

#define DEFAULT_STATE false
#define ANGLES_EVENT "angles"
#define INPUT_EVENT "input"
#define POSITION_EVENT "position"
#define MODE_EVENT "mode"

enum class MOTION_STATE { DEACTIVATED, IDLE, CALIBRATION, REST, STAND, CRAWL, WALK };

class MotionService {
  public:
    MotionService(ServoController *servoController) : _servoController(servoController) {}

    void begin() {
        socket.onEvent(INPUT_EVENT, [&](JsonObject &root, int originId) { handleInput(root, originId); });

        socket.onEvent(MODE_EVENT, [&](JsonObject &root, int originId) { handleMode(root, originId); });

        socket.onEvent(ANGLES_EVENT, [&](JsonObject &root, int originId) { anglesEvent(root, originId); });

        socket.onEvent(POSITION_EVENT, [&](JsonObject &root, int originId) { positionEvent(root, originId); });

        socket.onSubscribe(ANGLES_EVENT,
                           std::bind(&MotionService::syncAngles, this, std::placeholders::_1, std::placeholders::_2));

        kinematics.genPosture(DEG_TO_RAD_F(60), DEG_TO_RAD_F(75), body_state.feet);
    }

    void anglesEvent(JsonObject &root, int originId) {
        JsonArray array = root["data"].as<JsonArray>();
        for (int i = 0; i < 12; i++) {
            angles[i] = array[i];
        }
        syncAngles(String(originId));
    }

    void positionEvent(JsonObject &root, int originId) {
        JsonArray array = root["data"].as<JsonArray>();
        body_state.omega = array[0];
        body_state.phi = array[1];
        body_state.psi = array[2];
        body_state.xm = array[3];
        body_state.ym = array[4];
        body_state.zm = array[5];
    }

    void handleInput(JsonObject &root, int originId) {
        JsonArray array = root["data"].as<JsonArray>();
        command.lx = array[1];
        command.ly = array[2];
        command.rx = array[3];
        command.ry = array[4];
        command.h = array[5];
        command.s = array[6];
        command.s1 = array[7];

        body_state.ym = (command.h + 127.f) * 0.35f / 100;

        switch (motionState) {
            case MOTION_STATE::STAND: {
                body_state.phi = command.rx / 8;
                body_state.psi = command.ry / 8;
                body_state.xm = command.ly / 2 / 100;
                body_state.zm = command.lx / 2 / 100;
                kinematics.genPosture(DEG_TO_RAD_F(60), DEG_TO_RAD_F(75), body_state.feet);
                break;
            }
        }
    }

    void handleMode(JsonObject &root, int originId) {
        motionState = (MOTION_STATE)root["data"].as<int>();
        ESP_LOGI("MotionService", "Mode %d", motionState);
        char output[2];
        itoa((int)motionState, output, 10);
        motionState == MOTION_STATE::DEACTIVATED ? _servoController->deactivate() : _servoController->activate();
        socket.emit(MODE_EVENT, output, String(originId).c_str());
    }

    void emitAngles(const String &originId = "", bool sync = false) {
        char output[100];
        snprintf(output, sizeof(output), "[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]", angles[0],
                 angles[1], angles[2], angles[3], angles[4], angles[5], angles[6], angles[7], angles[8], angles[9],
                 angles[10], angles[11]);
        socket.emit(ANGLES_EVENT, output, originId.c_str());
    }

    void syncAngles(const String &originId = "", bool sync = false) {
        emitAngles(originId, sync);
        _servoController->setAngles(angles);
    }

    bool updateMotion() {
        switch (motionState) {
            case MOTION_STATE::DEACTIVATED: return false;
            case MOTION_STATE::IDLE: return false;
            case MOTION_STATE::CALIBRATION:
                _servoController->getCalibrationPWM(left_pwm_values, right_pwm_values);
                // update_angles(calibration_angles, new_angles, false);
                break;
            case MOTION_STATE::REST:
                // update_angles(rest_angles, new_angles, false);
                break;
            case MOTION_STATE::STAND:
                // kinematics.calculate_inverse_kinematics(body_state, new_angles);
                break;
            case MOTION_STATE::CRAWL: {
                body_state.xm = command.lx / 2.4;
                body_state.ym = command.ly / 2.4;
                body_state.zm = command.h / 2.4;
                body_state.psi = command.rx / 500;
                body_state.omega = command.ry / 500;

                kinematics.genPosture(DEG_TO_RAD_F(60), DEG_TO_RAD_F(75), body_state.feet);
                kinematics.inverseKinematics(body_state, new_angles);

                int8_t l_dir[3] = {-1, -1, 1};
                int8_t r_dir[3] = {-1, 1, -1};
                int16_t center_pwm = 307;

                for (int leg_idx = 0; leg_idx < 3; leg_idx++) {
                    for (int joint_idx = 0; joint_idx < 3; joint_idx++) {
                        int pin = leg_idx * 3 + joint_idx;
                        left_pwm_values[pin] =
                            (new_angles[leg_idx * 3 + joint_idx] * l_dir[joint_idx]) * 2 + center_pwm;
                        right_pwm_values[pin] =
                            (new_angles[leg_idx * 3 + joint_idx + 9] * r_dir[joint_idx]) * 2 + center_pwm;
                    }
                }
                // crawlGait->step(body_state, command);
                break;
            }
            case MOTION_STATE::WALK:
                // walkGait->step(body_state, command);
                break;
        }
        return true;
    }

    float *getAngles() { return angles; }

    uint16_t *getLeftPwm() { return left_pwm_values; }
    uint16_t *getRightPwm() { return right_pwm_values; }

  private:
    ServoController *_servoController;
    int step_count = 0;
    Kinematics kinematics;
    ControllerCommand command = {0, 0, 0, 0, 0, 0, 0, 0};

    friend class GaitState;

    uint16_t right_pwm_values[9] = {SERVOMID};
    uint16_t left_pwm_values[9] = {SERVOMID};

    // std::unique_ptr<GaitState> walkGait = std::make_unique<BezierState>();

    MOTION_STATE motionState = MOTION_STATE::DEACTIVATED;
    unsigned long _lastUpdate;

    body_state_t body_state = {0, 0, 0, 0, 0, 15};
    float new_angles[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    float angles[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
};

#endif
