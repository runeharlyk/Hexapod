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
        ESP_LOGI("MotionService", "Subscribing to event buses...");
        _cmdSubHandle = EventBus<CommandMsg>::subscribe([&](CommandMsg const &c) {
            ESP_LOGI("MotionService", "COMMAND callback called");
            handleCommand(c);
        });
        _modeSubHandle = EventBus<ModeMsg>::subscribe([&](ModeMsg const &c) {
            ESP_LOGI("MotionService", "MODE callback called with mode %d", (int)c.mode);
            handleInputMode(c);
        });
        _gaitSubHandle = EventBus<GaitMsg>::subscribe([&](GaitMsg const &c) {
            ESP_LOGI("MotionService", "GAIT callback called with gait %d", (int)c.gait);
            handleInputGait(c);
        });
        _angleSubHandle = EventBus<ServoAnglesMsg>::subscribe([&](ServoAnglesMsg const &s) {
            ESP_LOGI("MotionService", "ANGLES callback called");
            handleAnglesEvent(s);
        });
        ESP_LOGI("MotionService", "Event bus subscriptions completed");
        // TODO: Add body state
        // _positionSubHandle = EventBus::subscribe<Gait>([&](Gait const &c) { handleInputGait(c); });

        // TODO: Send joint angles on subscribe
        body_state.updateFeet(default_feet_pos);
        EventBus<ModeMsg>::publish({motionState});
        EventBus<GaitMsg>::publish({gait_state.gait_type});
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
    }

    void handleCommand(CommandMsg const &c) {
        lastCommandMillis = millis();
        commandTimedOut = false;
        updateFeetDistanceTarget(c.fd);
        target_body_state.zm = c.h * 50;
        target_body_state.omega = c.ry * 0.254f;
        switch (motionState) {
            case MOTION_STATE::STAND: {
                target_body_state.xm = c.lx * 50.f;
                target_body_state.ym = -c.ly * 50.f;
                target_body_state.phi = c.rx * 0.254f;
                gait_state.step_x = 0;
                gait_state.step_z = 0;
                gait_state.step_angle = 0;
                break;
            }
            case MOTION_STATE::WALK: {
                gait_state.step_x = -c.lx * 100;
                gait_state.step_z = c.ly * 100;
                gait_state.step_angle = c.rx * 0.8;
                gait_state.step_speed = CLIP((c.s + 1.f) * 0.5f, 0.0f, 1.0f);
                gait_state.step_height = (c.s1 + 1.f) * 20.f;
                gait_state.step_depth = 0.002f;
                break;
            }
        }
    }

    bool updateMotion() {
        resetCommandIfTimedOut();
        const float dt = getMotionDeltaSeconds();
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
                gait.step(gait_state, body_state, dt);
                kinematics.inverseKinematics(body_state, msgAngles.angles);
                break;
            }
            case MOTION_STATE::WALK: {
                body_state.xm = lerp(body_state.xm, target_body_state.xm, smoothing_factor);
                body_state.ym = lerp(body_state.ym, target_body_state.ym, smoothing_factor);
                body_state.zm = lerp(body_state.zm, target_body_state.zm, smoothing_factor);
                body_state.phi = lerp(body_state.phi, target_body_state.phi + _peripherals->angleY(), smoothing_factor);
                body_state.omega =
                    lerp(body_state.omega, target_body_state.omega + _peripherals->angleX(), smoothing_factor);
                gait.step(gait_state, body_state, dt);
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

    CommandMsg command = {0, 0, 0, 0, 0, 0, 0, 0};
    BodyStateMsg body_state = {0, 0, 0, 0, 0, 0};
    BodyStateMsg target_body_state = {0, 0, 0, 0, 0, 0};
    gait_state_t gait_state = {15, 0, 0, 0, 1, 0.002, default_stand_frac, GaitType::TRI_GATE, {0, 0.5, 0, 0.5, 0, 0.5}};

    const float smoothing_factor = 0.06f;
    static constexpr unsigned long COMMAND_TIMEOUT_MS = 2000;
    static constexpr float FEET_DISTANCE_SCALE_MIN = 0.75f;
    static constexpr float FEET_DISTANCE_SCALE_MAX = 1.25f;
    static constexpr float FEET_DISTANCE_SCALE_RANGE = FEET_DISTANCE_SCALE_MAX - FEET_DISTANCE_SCALE_MIN;

    float base_feet_pos[6][4] = {{122, 152, -66, 1},  {171, 0, -66, 1},  {122, -152, -66, 1},
                                 {-122, 152, -66, 1}, {-171, 0, -66, 1}, {-122, -152, -66, 1}};
    float default_feet_pos[6][4] = {{122, 152, -66, 1},  {171, 0, -66, 1},  {122, -152, -66, 1},
                                    {-122, 152, -66, 1}, {-171, 0, -66, 1}, {-122, -152, -66, 1}};

    MOTION_STATE motionState = MOTION_STATE::DEACTIVATED;
    unsigned long lastCommandMillis = 0;
    unsigned long lastMotionMicros = 0;
    bool commandTimedOut = false;

    ServoAnglesMsg msgAngles = {.angles = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

    void applyZeroCommand() {
        target_body_state.xm = 0;
        target_body_state.ym = 0;
        target_body_state.zm = 0;
        target_body_state.phi = 0;
        target_body_state.omega = 0;
        gait_state.step_x = 0;
        gait_state.step_z = 0;
        gait_state.step_angle = 0;
        gait_state.step_speed = 0.5f;
        gait_state.step_height = 15.f;
        gait_state.step_depth = 0.002f;
    }

    void rebuildDefaultFeet(float scale, float output[6][4]) {
        for (int i = 0; i < 6; ++i) {
            output[i][0] = base_feet_pos[i][0] * scale;
            output[i][1] = base_feet_pos[i][1] * scale;
            output[i][2] = base_feet_pos[i][2];
            output[i][3] = base_feet_pos[i][3];
        }
    }

    void updateFeetDistanceTarget(float normalizedFeetDistance) {
        const float clamped = CLIP(normalizedFeetDistance, -1.0f, 1.0f);
        const float scale = FEET_DISTANCE_SCALE_MIN + ((clamped + 1.0f) * 0.5f) * FEET_DISTANCE_SCALE_RANGE;
        rebuildDefaultFeet(scale, default_feet_pos);
        gait.setDefaultFootTarget(default_feet_pos);
    }

    float getMotionDeltaSeconds() {
        const unsigned long now = micros();
        if (lastMotionMicros == 0) {
            lastMotionMicros = now;
            return 0.005f;
        }

        const unsigned long elapsedMicros = now - lastMotionMicros;
        lastMotionMicros = now;
        const float dt = elapsedMicros / 1000000.0f;
        return CLIP(dt, 0.001f, 0.05f);
    }

    void resetCommandIfTimedOut() {
        if (lastCommandMillis == 0) return;
        if (commandTimedOut) return;
        if (millis() - lastCommandMillis < COMMAND_TIMEOUT_MS) return;

        ESP_LOGW("MotionService", "No motion command for %lu ms, applying zero command", COMMAND_TIMEOUT_MS);
        applyZeroCommand();
        commandTimedOut = true;
    }
};

#endif
