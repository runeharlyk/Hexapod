#ifndef MotionService_h
#define MotionService_h

#include <event_socket.h>
#include <kinematics.h>
#include <peripherals/servo_controller.h>
#include <utils/timing.h>
#include <utils/math_utils.h>
#include <gait.h>

#include "config.h"

#define DEFAULT_STATE false
#define ANGLES_EVENT "angles"
#define INPUT_EVENT "input"
#define POSITION_EVENT "position"
#define MODE_EVENT "mode"
#define GAIT_EVENT "gait"

enum class MOTION_STATE { DEACTIVATED, IDLE, POSE, STAND, WALK };

class MotionService {
  public:
    MotionService(ServoController *servoController) : _servoController(servoController) {}

    void begin() {
        socket.onEvent(INPUT_EVENT, [&](JsonObject &root, int originId) { handleInput(root, originId); });

        socket.onEvent(MODE_EVENT, [&](JsonObject &root, int originId) { handleMode(root, originId); });

        socket.onEvent(ANGLES_EVENT, [&](JsonObject &root, int originId) { anglesEvent(root, originId); });

        socket.onEvent(POSITION_EVENT, [&](JsonObject &root, int originId) { positionEvent(root, originId); });

        socket.onEvent(GAIT_EVENT, [&](JsonObject &root, int originId) { gaitEvent(root, originId); });

        socket.onSubscribe(ANGLES_EVENT,
                           std::bind(&MotionService::syncAngles, this, std::placeholders::_1, std::placeholders::_2));

        body_state.updateFeet(default_feet_pos);
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

        body_state.zm = command.h / 2.4;

        switch (motionState) {
            case MOTION_STATE::STAND: {
                body_state.xm = command.lx / 2.4;
                body_state.ym = command.ly / 2.4;
                body_state.zm = command.h / 2.4;
                body_state.phi = command.rx / 500;
                body_state.omega = command.ry / 500;
                body_state.updateFeet(default_feet_pos);
                break;
            }
            case MOTION_STATE::WALK: {
                gait_state.step_x = command.lx;
                gait_state.step_z = command.ly;
                gait_state.step_angle = command.rx / 150;
                gait_state.step_speed = command.s / 128 + 1;
                gait_state.step_height = command.s1 / 15 + 15;
                gait_state.step_depth = 0.002;

                body_state.omega = command.ry / 500;
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
        if (motionState == MOTION_STATE::STAND) body_state.updateFeet(default_feet_pos);
        socket.emit(MODE_EVENT, output, String(originId).c_str());
    }

    void gaitEvent(JsonObject &root, int originId) {
        gait_state.gait_type = (GaitType)root["data"].as<int>();
        ESP_LOGI("MotionService", "Gait %d", gait_state.gait_type);
        gait.setGait(gait_state);
        char output[2];
        itoa((int)gait_state.gait_type, output, 10);
        socket.emit(GAIT_EVENT, output, String(originId).c_str());
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
            case MOTION_STATE::POSE: _servoController->setCenterPwm(); return false;
            case MOTION_STATE::STAND: kinematics.inverseKinematics(body_state, angles); break;
            case MOTION_STATE::WALK: {
                gait.step(gait_state, body_state, 5.f / 1000.f);
                kinematics.inverseKinematics(body_state, angles);
                break;
            }
        }
        return true;
    }

    float *getAngles() { return angles; }

  private:
    ServoController *_servoController;
    int step_count = 0;
    Kinematics kinematics;
    GaitController gait;

    ControllerCommand command = {0, 0, 0, 0, 0, 0, 0, 0};
    body_state_t body_state = {0, 0, 0, 0, 0, 15};
    gait_state_t gait_state = {15, 0, 0, 0, 1, 0.002, default_stand_frac, GaitType::TRI_GATE, {0, 0.5, 0, 0.5, 0, 0.5}};

    float default_feet_pos[6][4] = {{122, 152, -66, 1},  {171, 0, -66, 1},  {122, -152, -66, 1},
                                    {-122, 152, -66, 1}, {-171, 0, -66, 1}, {-122, -152, -66, 1}};

    MOTION_STATE motionState = MOTION_STATE::DEACTIVATED;
    unsigned long _lastUpdate;

    float angles[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
};

#endif
