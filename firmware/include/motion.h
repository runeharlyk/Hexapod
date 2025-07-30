#ifndef MotionService_h
#define MotionService_h

#include <kinematics.h>
#include <peripherals/servo_controller.h>
#include <utils/timing.h>
#include <utils/math_utils.h>
#include <gait.h>
#include <event_bus.h>

#include "config.h"

#define DEFAULT_STATE false
#define ANGLES_EVENT "angles"
#define INPUT_EVENT "input"
#define POSITION_EVENT "position"
#define MODE_EVENT "mode"
#define GAIT_EVENT "gait"

class MotionService {
    void *_cmdSubHandle;
    void *_modeSubHandle;
    void *_gaitSubHandle;
    void *_anglesSubHandle;
    void *_positionSubHandle;

  public:
    MotionService(ServoController *servoController, Peripherals *peripherals)
        : _servoController(servoController), _peripherals(peripherals) {}

    void begin() {
        _cmdSubHandle = EventBus::subscribe<CommandMsg>([&](CommandMsg const &c) { handleCommand(c); });
        _modeSubHandle = EventBus::subscribe<ModeMsg>([&](ModeMsg const &c) { handleInputMode(c); });
        _gaitSubHandle = EventBus::subscribe<GaitMsg>([&](GaitMsg const &c) { handleInputGait(c); });
        _anglesSubHandle = EventBus::subscribe<ServoAnglesMsg>([&](ServoAnglesMsg const &s) { handleAnglesEvent(s); });
        // TODO: Add body state
        // _positionSubHandle = EventBus::subscribe<Gait>([&](Gait const &c) { handleInputGait(c); });

        // TODO: Send joint angles on subscribe
        body_state.updateFeet(default_feet_pos);
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

    void handleAnglesEvent(ServoAnglesMsg const &s) {
        for (int i = 0; i < 12; i++) {
            angles[i] = s.angles[i];
        }
    }

    void handleInputGait(GaitMsg const &g) {
        ESP_LOGI("MotionService", "Gait %d", g.gait);
        gait_state.gait_type = g.gait;
    }

    void handleInputMode(ModeMsg const &m) {
        ESP_LOGI("MotionService", "Mode %d", m.mode);
        motionState = m.mode;
        motionState == MOTION_STATE::DEACTIVATED ? _servoController->deactivate() : _servoController->activate();
        if (motionState == MOTION_STATE::STAND) body_state.updateFeet(default_feet_pos);
    }

    void handleCommand(CommandMsg const &c) {
        Serial.println("handleCommand");
        body_state.zm = c.h * 50;
        ESP_LOGI("imu", "%.2f \t %.2f \t %.2f \t %.2f", _peripherals->angleZ(), _peripherals->angleY(),
                 _peripherals->angleX(), c.rx * 0.254f);
        switch (motionState) {
            case MOTION_STATE::STAND: {
                target_body_state.xm = c.lx * 50.f;
                target_body_state.ym = -c.ly * 50.f;
                target_body_state.phi = c.rx * 0.254f;
                target_body_state.omega = c.ry * 0.254f;
                body_state.updateFeet(default_feet_pos);
                break;
            }
            case MOTION_STATE::WALK: {
                gait_state.step_x = -c.lx * 100;
                gait_state.step_z = c.ly * 100;
                gait_state.step_angle = c.rx * 0.8;
                gait_state.step_speed = c.s + 1.f;
                gait_state.step_height = (c.s1 + 1.f) * 10.f;
                gait_state.step_depth = 0.002f;

                body_state.omega = c.ry * 0.254f;
                break;
            }
        }
    }

    bool updateMotion() {
        switch (motionState) {
            case MOTION_STATE::DEACTIVATED: return false;
            case MOTION_STATE::IDLE: return false;
            case MOTION_STATE::POSE: _servoController->setCenterPwm(); return false;
            case MOTION_STATE::STAND: {
                body_state.xm = lerp(body_state.xm, target_body_state.xm, 0.05);
                body_state.ym = lerp(body_state.ym, target_body_state.ym, 0.05);
                body_state.phi = lerp(body_state.phi, target_body_state.phi + _peripherals->angleY(), 0.05);
                body_state.omega = lerp(body_state.omega, target_body_state.omega + _peripherals->angleX(), 0.05);
                kinematics.inverseKinematics(body_state, angles);
                break;
            }
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
    Peripherals *_peripherals;
    int step_count = 0;
    Kinematics kinematics;
    GaitController gait;

    CommandMsg command = {0, 0, 0, 0, 0, 0, 0};
    body_state_t body_state = {0, 0, 0, 0, 0, 15};
    body_state_t target_body_state = {0, 0, 0, 0, 0, 15};
    gait_state_t gait_state = {15, 0, 0, 0, 1, 0.002, default_stand_frac, GaitType::TRI_GATE, {0, 0.5, 0, 0.5, 0, 0.5}};

    float default_feet_pos[6][4] = {{122, 152, -66, 1},  {171, 0, -66, 1},  {122, -152, -66, 1},
                                    {-122, 152, -66, 1}, {-171, 0, -66, 1}, {-122, -152, -66, 1}};

    MOTION_STATE motionState = MOTION_STATE::DEACTIVATED;
    unsigned long _lastUpdate;

    float angles[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
};

#endif
