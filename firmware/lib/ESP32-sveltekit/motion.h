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

#include "motion_table.h"
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

        body_state.updateFeet(default_feet_positions);
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
                body_state.updateFeet(default_feet_positions);
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
            case MOTION_STATE::CRAWL:
                // crawlGait->step(body_state, command);
                // kinematics.calculate_inverse_kinematics(body_state, new_angles);
                break;
            case MOTION_STATE::WALK:
                walk(command);
                // walkGait->step(body_state, command);
                // kinematics.calculate_inverse_kinematics(body_state, new_angles);
                break;
        }
        return true; // update_angles(new_angles, angles);
    }

    void walk(ControllerCommand command) {
        float abs_lx = abs(command.lx);
        float abs_ly = abs(command.ly);

        if (abs_ly > abs_lx && abs_ly > 30) {
            if (command.ly > 0) {
                if (abs(command.lx) < 30) {
                    exec_motion(lut_fast_forward_length, lut_fast_forward);
                } else if (command.lx > 30) {
                    exec_motion(lut_walk_r45_length, lut_walk_r45);
                } else if (command.lx < -30) {
                    exec_motion(lut_walk_l45_length, lut_walk_l45);
                }
            } else {
                if (abs(command.lx) < 30) {
                    exec_motion(lut_fast_backward_length, lut_fast_backward);
                } else if (command.lx > 30) {
                    exec_motion(lut_walk_r135_length, lut_walk_r135);
                } else if (command.lx < -30) {
                    exec_motion(lut_walk_l135_length, lut_walk_l135);
                }
            }
        } else if (abs_lx > abs_ly && abs_lx > 30) {
            if (command.lx > 0) {
                exec_motion(lut_walk_r90_length, lut_walk_r90);
            } else {
                exec_motion(lut_walk_l90_length, lut_walk_l90);
            }
        } else if (abs_lx <= 30 && abs_ly <= 30) {
            if (command.rx > 30) {
                exec_motion(lut_turn_right_length, lut_turn_right);
            } else if (command.rx < -30) {
                exec_motion(lut_turn_left_length, lut_turn_left);
            } else if (command.ry > 30) {
                exec_motion(lut_rotate_x_length, lut_rotate_x);
            } else if (command.ry < -30) {
                exec_motion(lut_rotate_y_length, lut_rotate_y);
            }
        } else if (command.s > 100) {
            exec_motion(lut_twist_length, lut_twist);
        } else if (command.s1 > 100) {
            exec_motion(lut_climb_forward_length, lut_climb_forward);
        }
    }

    void exec_motion(int lut_size, int lut[][6][3]) {
        for (int leg_idx = 0; leg_idx < 3; leg_idx++) {
            for (int joint_idx = 0; joint_idx < 3; joint_idx++) {
                int pin = leg_idx * 3 + joint_idx;
                right_pwm_values[pin] = lut[step_count / 8][leg_idx][joint_idx];
                left_pwm_values[pin] = lut[step_count / 8][leg_idx + 3][joint_idx];
            }
        }
        step_count = (step_count + 1) % (lut_size * 8);
    }

    bool update_angles(float new_angles[12], float angles[12], bool useLerp = true) {
        bool updated = false;
        for (int i = 0; i < 12; i++) {
            float new_angle = useLerp ? lerp(angles[i], new_angles[i] * dir[i], 0.3) : new_angles[i] * dir[i];
            if (!isEqual(new_angle, angles[i], 0.1)) {
                angles[i] = new_angle;
                updated = true;
            }
        }
        return updated;
    }

    float *getAngles() { return angles; }

    uint16_t *getLeftPwm() { return left_pwm_values; }
    uint16_t *getRightPwm() { return right_pwm_values; }

  private:
    ServoController *_servoController;
    int step_count = 0;
    // Kinematics kinematics;
    ControllerCommand command = {0, 0, 0, 0, 0, 0, 0, 0};

    friend class GaitState;

    uint16_t right_pwm_values[9] = {SERVOMID};
    uint16_t left_pwm_values[9] = {SERVOMID};

    // std::unique_ptr<GaitState> crawlGait = std::make_unique<EightPhaseWalkState>();
    // std::unique_ptr<GaitState> walkGait = std::make_unique<BezierState>();

    MOTION_STATE motionState = MOTION_STATE::DEACTIVATED;
    unsigned long _lastUpdate;

    body_state_t body_state = {0, 0, 0, 0, 0, 0};
    float new_angles[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    float angles[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    float dir[12] = {1, -1, -1, -1, -1, -1, 1, -1, -1, -1, -1, -1};
    float default_feet_positions[4][4] = {
        {1.0375, -1, 0.7, 1}, {1.0375, -1, -0.7, 1}, {-1.0375, -1, 0.7, 1}, {-1.0375, -1, -0.7, 1}};

    float rest_angles[12] = {0, 90, -145, 0, 90, -145, 0, 90, -145, 0, 90, -145};
    float calibration_angles[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
};

#endif
