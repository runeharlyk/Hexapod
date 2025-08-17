#ifndef MotionService_h
#define MotionService_h

#include <kinematics.h>
#include <peripherals/servo_controller.h>
#include <utils/timing.h>
#include <utils/math_utils.h>
#include <gait.h>
#include <event_bus.h>
#include <message_types.h>

class MotionService {
  public:
    MotionService(ServoController *servoController, Peripherals *peripherals)
        : _servoController(servoController), _peripherals(peripherals) {}

    void begin() {
        _cmdSubHandle = EventBus<CommandMsg>::subscribe([&](CommandMsg const &c) { handleCommand(c); });
        _modeSubHandle = EventBus<ModeMsg>::subscribe([&](ModeMsg const &c) { handleInputMode(c); });
        _gaitSubHandle = EventBus<GaitMsg>::subscribe([&](GaitMsg const &c) { handleInputGait(c); });
        _angleSubHandle = EventBus<ServoAnglesMsg>::subscribe([&](ServoAnglesMsg const &s) { handleAnglesEvent(s); });
        // TODO: Add body state
        // _positionSubHandle = EventBus::subscribe<Gait>([&](Gait const &c) { handleInputGait(c); });

        // TODO: Send joint angles on subscribe
        body_state.updateFeet(default_feet_pos);
    }

    void handleAnglesEvent(ServoAnglesMsg const &s) {
        for (int i = 0; i < 12; i++) {
            msgAngles.angles[i] = s.angles[i];
        }
    }

    void handleInputGait(GaitMsg const &g) {
        ESP_LOGI("MotionService", "Gait %d", g.gait);
        gait_state.gait_type = g.gait;
        gait.setGait(gait_state);
    }

    void handleInputMode(ModeMsg const &m) {
        ESP_LOGI("MotionService", "Mode %d", m.mode);
        motionState = m.mode;
        motionState == MOTION_STATE::DEACTIVATED ? _servoController->deactivate() : _servoController->activate();
        if (motionState == MOTION_STATE::STAND) body_state.updateFeet(default_feet_pos);
    }

    void handleCommand(CommandMsg const &c) {
        target_body_state.zm = c.h * 50;
        target_body_state.omega = c.ry * 0.254f;
        switch (motionState) {
            case MOTION_STATE::STAND: {
                target_body_state.xm = c.lx * 50.f;
                target_body_state.ym = -c.ly * 50.f;
                target_body_state.phi = c.rx * 0.254f;
                body_state.updateFeet(default_feet_pos);
                break;
            }
            case MOTION_STATE::WALK: {
                gait_state.step_x = -c.lx * 100;
                gait_state.step_z = c.ly * 100;
                gait_state.step_angle = c.rx * 0.8;
                gait_state.step_speed = c.s + 1.f;
                gait_state.step_height = (c.s1 + 1.f) * 20.f;
                gait_state.step_depth = 0.002f;
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
                body_state.xm = lerp(body_state.xm, target_body_state.xm, smoothing_factor);
                body_state.ym = lerp(body_state.ym, target_body_state.ym, smoothing_factor);
                body_state.zm = lerp(body_state.zm, target_body_state.zm, smoothing_factor);
                body_state.phi = lerp(body_state.phi, target_body_state.phi + _peripherals->angleY(), smoothing_factor);
                body_state.omega =
                    lerp(body_state.omega, target_body_state.omega + _peripherals->angleX(), smoothing_factor);
                kinematics.inverseKinematics(body_state, msgAngles.angles);
                break;
            }
            case MOTION_STATE::WALK: {
                body_state.phi = lerp(body_state.phi, target_body_state.phi + _peripherals->angleY(), smoothing_factor);
                body_state.omega =
                    lerp(body_state.omega, target_body_state.omega + _peripherals->angleX(), smoothing_factor);
                gait.step(gait_state, body_state, 5.f / 1000.f);
                kinematics.inverseKinematics(body_state, msgAngles.angles);
                break;
            }
        }
        return true;
    }

    float *getAngles() { return msgAngles.angles; }

    void publishState() { EventBus<ServoAnglesMsg>::publish(msgAngles); }

  private:
    ServoController *_servoController;
    Peripherals *_peripherals;
    EventBus<CommandMsg>::Handle _cmdSubHandle;
    EventBus<ModeMsg>::Handle _modeSubHandle;
    EventBus<GaitMsg>::Handle _gaitSubHandle;
    EventBus<ServoAnglesMsg>::Handle _angleSubHandle;
    Kinematics kinematics;
    GaitController gait;

    CommandMsg command = {0, 0, 0, 0, 0, 0, 0};
    body_state_t body_state = {0, 0, 0, 0, 0, 0};
    body_state_t target_body_state = {0, 0, 0, 0, 0, 0};
    gait_state_t gait_state = {15, 0, 0, 0, 1, 0.002, default_stand_frac, GaitType::TRI_GATE, {0, 0.5, 0, 0.5, 0, 0.5}};

    const float smoothing_factor = 0.06f;

    float default_feet_pos[6][4] = {{122, 152, -66, 1},  {171, 0, -66, 1},  {122, -152, -66, 1},
                                    {-122, 152, -66, 1}, {-171, 0, -66, 1}, {-122, -152, -66, 1}};

    MOTION_STATE motionState = MOTION_STATE::DEACTIVATED;

    ServoAnglesMsg msgAngles = {.angles = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
};

#endif
