#ifndef MotionService_h
#define MotionService_h

#include <esp_log.h>
#include <kinematics.h>
#include <peripherals/servo_controller.h>
#include <peripherals/peripherals.h>
#include <utils/timing.h>
#include <utils/math_utils.h>
#include <gait.h>
#include <event_bus.h>
#include <message_types.h>
#include <policy_runner.h>
#include <animation/animation_runner.h>

class MotionService {
  public:
    MotionService(ServoController *servoController, Peripherals *peripherals)
        : _servoController(servoController), _peripherals(peripherals) {}

    void begin() {
        _animation.begin(kinematics, default_feet_pos);
        ESP_LOGI("MotionService", "Subscribing to event buses...");
        _cmdSubHandle = EventBus<CommandMsg>::subscribe([&](CommandMsg const &c) {
            ESP_LOGD("MotionService", "COMMAND callback called");
            handleCommand(c);
        });
        _modeSubHandle = EventBus<ModeMsg>::subscribe([&](ModeMsg const &c) {
            ESP_LOGD("MotionService", "MODE callback called with mode %d", (int)c.mode);
            handleInputMode(c);
        });
        _gaitSubHandle = EventBus<GaitMsg>::subscribe([&](GaitMsg const &c) {
            ESP_LOGD("MotionService", "GAIT callback called with gait %d", (int)c.gait);
            handleInputGait(c);
        });
        _angleSubHandle = EventBus<ServoAnglesMsg>::subscribe([&](ServoAnglesMsg const &s) {
            ESP_LOGD("MotionService", "ANGLES callback called");
            handleAnglesEvent(s);
        });
        ESP_LOGI("MotionService", "Event bus subscriptions completed");
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
        _requestedGait = g.gait;
        // Swapping a schedule mid-stride is the hazard updateAutoGait() documents: offset[] reassigns
        // every leg's phase at once, so a leg at the top of its swing becomes a stance leg and its
        // foot is commanded straight down. Defer to the next non-stepping tick instead, which
        // applyRequestedGait() handles from the walking loop.
        if (gaitIsCommanded(gait_state)) return;
        applyRequestedGait();
    }

    // Loads _requestedGait into the live schedule. Only safe while the gait is not stepping.
    void applyRequestedGait() {
        gait_state.gait_type = _requestedGait == GaitType::AUTO
                                   ? selectAutoGait(commandSpeed01(), gait_state.gait_type)
                                   : _requestedGait;
        gait.setGait(gait_state);
        // The schedule switches outright; only the command eases. Keep the target's identity fields
        // in step so a later ramp never reads a stale gait.
        target_gait_state.gait_type = gait_state.gait_type;
        target_gait_state.stand_frac = gait_state.stand_frac;
        for (int i = 0; i < 6; i++) target_gait_state.offset[i] = gait_state.offset[i];
    }

    void handleInputMode(ModeMsg const &m) {
        ESP_LOGI("MotionService", "Mode %d%s", m.mode, m.borrow ? " (borrow)" : "");
        // A borrow is decided here, in mode order, so a DEACTIVATED processed while the clip loaded
        // wins over it. Any explicit mode ends a borrow: an explicit ANIMATE is sticky and an explicit
        // STAND mid-play is final.
        if (m.borrow) {
            if (!isActuatedMode(motionState)) {
                _animation.cancelPendingPlay();
                // Every mode observer has just seen the refused ANIMATE; restate the mode that stands.
                EventBus<ModeMsg>::publish({motionState});
                return;
            }
            if (motionState != MOTION_STATE::ANIMATE) {
                _previousMode = motionState;
                _borrowedMode = true;
            }
        } else {
            _borrowedMode = false;
            if (m.mode != MOTION_STATE::ANIMATE) _animation.cancelPendingPlay();
        }
        motionState = m.mode;
#if FT_ENABLED(USE_POLICY)
        if (m.mode == MOTION_STATE::WALK_NN) _policy.reset(gait, default_feet_pos);
#endif
        // ANIMATE drives no gait, so a half-stride left in gait_state would resume after it.
        if (!isWalkingMode(m.mode)) stopLocomotionCommand(!isActuatedMode(m.mode) || m.mode == MOTION_STATE::ANIMATE);
        motionState == MOTION_STATE::DEACTIVATED ? _servoController->deactivate() : _servoController->activate();
    }

    // Adapter task. A play from any active mode borrows ANIMATE and hands the mode back when the
    // player finishes; a play while already in ANIMATE (borrowed or sticky) chains. The clip is loaded
    // before the mode changes, so a refused play leaves the robot in its mode and pose; whether the
    // borrow is granted is decided by handleInputMode.
    void handleAnimationCommand(AnimationCommandMsg const &c) {
        if (!isActuatedMode(motionState)) return;
        if (!_animation.requestPlay(c)) return;
        if (motionState == MOTION_STATE::ANIMATE) return;
        EventBus<ModeMsg>::publish({MOTION_STATE::ANIMATE, true});
    }

    // Adapter task. A stop only makes sense in ANIMATE.
    void handleAnimationStop() {
        if (motionState == MOTION_STATE::ANIMATE) _animation.requestStop();
    }

    // Adapter task. Puppeteer poses apply only in ANIMATE.
    void handleAnimationPose(PoseMsg const &p) {
        if (motionState == MOTION_STATE::ANIMATE) _animation.setPuppet(p);
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
                target_gait_state.step_x = 0;
                target_gait_state.step_y = 0;
                target_gait_state.step_angle = 0;
                break;
            }
#if FT_ENABLED(USE_POLICY)
            case MOTION_STATE::WALK_NN: {
                _policy.setCommand(c);
                target_body_state.xm = target_body_state.ym = 0;
                target_body_state.phi = 0;
                break;
            }
#endif
            case MOTION_STATE::WALK: {
                target_gait_state.step_x = -c.lx * 100;
                target_gait_state.step_y = c.ly * 100;
                target_gait_state.step_angle = c.rx * 0.8;
                target_gait_state.step_speed = c.s + 1.f;
                if (gait_state.gait_type == GaitType::TUNED) {
                    // The searched gait is a VELOCITY -> gait map, not a direct stride map: stride
                    // and cadence were optimized together, so driving stride from the stick while
                    // leaving cadence to the legacy law reproduces neither. The stick therefore
                    // becomes a velocity command over the range the search covered.
                    const float cmd[3] = {c.ly * TUNED_CMD_VX_MAX, -c.lx * TUNED_CMD_VY_MAX,
                                          c.rx * TUNED_CMD_YAW_MAX};
                    float a[6];
                    tuned_gait::velocity_to_gait(cmd, a);
                    target_gait_state.step_x = a[0] * tuned_gait::STEP_XY_MM;
                    target_gait_state.step_y = a[1] * tuned_gait::STEP_XY_MM;
                    target_gait_state.step_angle = a[2] * tuned_gait::STEP_ANGLE_RAD;
                    // s1 trims foot lift around the searched value rather than setting it from
                    // zero; the old (s1+1)*20 mapping tops out at 40 mm and cannot express 68 mm.
                    target_gait_state.step_height =
                        CLIP(tuned_gait::STEP_HEIGHT_MM + c.s1 * TUNED_LIFT_TRIM_MM,
                             tuned_gait::STEP_HEIGHT_MIN_MM, tuned_gait::STEP_HEIGHT_MAX_MM);
                    target_gait_state.step_depth = tuned_gait::STEP_DEPTH_MM;
                    target_gait_state.phase_rate =
                        tuned_gait::PHASE_RATE_MIN +
                        (a[5] + 1.f) * 0.5f * (tuned_gait::PHASE_RATE_MAX - tuned_gait::PHASE_RATE_MIN);
                    target_body_state.zm = -tuned_gait::RIDE_MM;
                    logTunedCommand(cmd, target_gait_state);
                } else {
                    target_gait_state.step_height = (c.s1 + 1.f) * 20.f;
                    target_gait_state.step_depth = 0.002f;
                    target_gait_state.phase_rate = 0.f;  // legacy stride-derived cadence
                }
                break;
            }
            default: break;
        }
    }

    // The stick passes through three clip1() stages inside velocity_to_gait plus a CLIP on lift, and
    // stride saturates at half stick, so a bench operator cannot otherwise tell a mapping that
    // saturated from a command that was ignored. Once a second is enough to read while driving.
    void logTunedCommand(const float cmd[3], const gait_state_t &g) {
        const unsigned long now = millis();
        if (now - _lastTunedLog < 1000) return;
        _lastTunedLog = now;
        ESP_LOGI("MotionService",
                 "TUNED cmd v=[%.3f %.3f %.3f] -> step x/y %.1f/%.1f angle %.3f lift %.1f rate %.2f zm %.1f",
                 cmd[0], cmd[1], cmd[2], g.step_x, g.step_y, g.step_angle, g.step_height, g.phase_rate,
                 target_body_state.zm);
    }

    // Normalized stride magnitude, the speed signal AUTO switches on.
    float commandSpeed01() const {
        const float stride = sqrtf(target_gait_state.step_x * target_gait_state.step_x +
                                   target_gait_state.step_y * target_gait_state.step_y);
        return CLIP(stride / 100.0f, 0.0f, 1.0f);
    }

    // Schedules cannot be swapped while stepping: changing offset[] reassigns every leg's phase at
    // once, and a leg at the top of its swing in one gait is a stance leg in the next, so its foot is
    // commanded straight down -- measured at 78 mm in a single tick. Retiming legs individually to
    // avoid that is a real transition problem and is deliberately not attempted here.
    //
    // Instead switch only while the gait is not stepping. selectAutoGait reads the TARGET command,
    // so the ramp's window at each start and stop is enough to choose the gait for the walk about to
    // happen. The cost is that changing gait at speed needs a brief stop.
    void updateAutoGait() {
        if (gaitIsCommanded(gait_state)) return;

        // A manual selection that arrived mid-stride was deferred; this is the window to honour it.
        if (_requestedGait != GaitType::AUTO) {
            if (_requestedGait == gait_state.gait_type) return;
            applyRequestedGait();
            ESP_LOGI("MotionService", "Deferred gait -> %d", (int)gait_state.gait_type);
            return;
        }

        const GaitType next = selectAutoGait(commandSpeed01(), gait_state.gait_type);
        if (next == gait_state.gait_type) return;
        gait_state.gait_type = next;
        gait.setGait(gait_state);
        target_gait_state.gait_type = next;
        target_gait_state.stand_frac = gait_state.stand_frac;
        for (int i = 0; i < 6; i++) target_gait_state.offset[i] = gait_state.offset[i];
        ESP_LOGI("MotionService", "Auto gait -> %d", (int)next);
    }

    bool updateMotion() {
        resetCommandIfTimedOut();
        const float dt = getMotionDeltaSeconds();
        const MOTION_STATE state = motionState;
        // Entering and leaving ANIMATE are handled here rather than in handleInputMode so the runner
        // is only ever touched by the control task.
        if ((state == MOTION_STATE::ANIMATE) != _animating) {
            _animating = state == MOTION_STATE::ANIMATE;
            if (_animating)
                _animation.enter(body_state);
            else
                _animation.reset();
        }
        switch (state) {
            case MOTION_STATE::DEACTIVATED: return false;
            case MOTION_STATE::IDLE: return false;
            case MOTION_STATE::POSE: _servoController->setCenterPwm(); return false;
            case MOTION_STATE::STAND: {
                body_state.xm = lerpf(body_state.xm, target_body_state.xm, smoothing_factor);
                body_state.ym = lerpf(body_state.ym, target_body_state.ym, smoothing_factor);
                body_state.zm = lerpf(body_state.zm, target_body_state.zm, smoothing_factor);
                body_state.phi = lerpf(body_state.phi, target_body_state.phi + _peripherals->angleY(), smoothing_factor);
                body_state.omega =
                    lerpf(body_state.omega, target_body_state.omega + _peripherals->angleX(), smoothing_factor);
                // Only ANIMATE sets yaw; ease back what a mode change mid-animation left behind.
                body_state.psi = lerpf(body_state.psi, 0.f, smoothing_factor);
                // STAND is not stepping, so this is where a gait selection deferred by
                // handleInputGait gets honoured -- otherwise leaving WALK would strand it.
                updateAutoGait();
                approachGaitCommand(gait_state, target_gait_state, dt, GAIT_COMMAND_TAU_S);
                gait.step(gait_state, body_state, dt);
                kinematics.inverseKinematics(body_state, msgAngles.angles);
                break;
            }
#if FT_ENABLED(USE_POLICY)
            case MOTION_STATE::WALK_NN: {
                // The policy owns stride, lift, cadence and ride height; it was trained with the
                // firmware's self-levelling active, so that term stays on here too.
                body_state.phi = lerpf(body_state.phi, _peripherals->angleY(), smoothing_factor);
                body_state.omega = lerpf(body_state.omega, _peripherals->angleX(), smoothing_factor);
                body_state.psi = lerpf(body_state.psi, 0.f, smoothing_factor);
                if (!_policy.update(_peripherals, gait, body_state, msgAngles.angles)) return false;
                kinematics.inverseKinematics(body_state, msgAngles.angles);
                break;
            }
#endif
            case MOTION_STATE::ANIMATE: {
                // tick writes the angles itself; body_state carries the absolute pose for telemetry.
                const bool finished = _animation.tick(dt, body_state, msgAngles.angles);
                if (finished && _borrowedMode && EventBus<ModeMsg>::publish({_previousMode})) _borrowedMode = false;
                break;
            }
            case MOTION_STATE::WALK: {
                body_state.xm = lerpf(body_state.xm, target_body_state.xm, smoothing_factor);
                body_state.ym = lerpf(body_state.ym, target_body_state.ym, smoothing_factor);
                body_state.zm = lerpf(body_state.zm, target_body_state.zm, smoothing_factor);
                body_state.phi = lerpf(body_state.phi, target_body_state.phi + _peripherals->angleY(), smoothing_factor);
                body_state.omega =
                    lerpf(body_state.omega, target_body_state.omega + _peripherals->angleX(), smoothing_factor);
                // Only ANIMATE sets yaw; ease back what a mode change mid-animation left behind.
                body_state.psi = lerpf(body_state.psi, 0.f, smoothing_factor);
                // Before the ramp: it clears the 2 mm deadband in one tick, closing the window.
                updateAutoGait();
                approachGaitCommand(gait_state, target_gait_state, dt, GAIT_COMMAND_TAU_S);
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
#if FT_ENABLED(USE_POLICY)
    PolicyRunner _policy;
#endif
    AnimationRunner _animation;
    MOTION_STATE _previousMode = MOTION_STATE::STAND;
    bool _borrowedMode = false;
    bool _animating = false; // control task only: whether the last tick ran ANIMATE

    CommandMsg command = {0, 0, 0, 0, 0, 0, 0, 0};
    BodyStateMsg body_state = {0, 0, 0, 0, 0, 0};
    BodyStateMsg target_body_state = {0, 0, 0, 0, 0, 0};
    static constexpr float DEFAULT_STEP_HEIGHT_MM = 15.f;
    static constexpr float DEFAULT_STEP_DEPTH = 0.002f;
    gait_state_t gait_state = {
        DEFAULT_STEP_HEIGHT_MM,  0, 0, 0, 1, DEFAULT_STEP_DEPTH, default_stand_frac, GaitType::TRI_GATE,
        {0, 0.5, 0, 0.5, 0, 0.5}};
    // Commands land here; gait_state eases toward it every tick (issue #7).
    gait_state_t target_gait_state = gait_state;

    const float smoothing_factor = 0.06f;
    // Ramp time constant for the locomotion command: ~0.45 s to settle. Slow enough that a slammed
    // stick does not jerk the stride, fast enough that the robot still feels directly driven.
    static constexpr float GAIT_COMMAND_TAU_S = 0.15f;
    static constexpr unsigned long COMMAND_TIMEOUT_MS = 2000;
    static constexpr float FEET_DISTANCE_SCALE_MIN = 0.75f;
    static constexpr float FEET_DISTANCE_SCALE_MAX = 1.25f;
    static constexpr float FEET_DISTANCE_SCALE_RANGE = FEET_DISTANCE_SCALE_MAX - FEET_DISTANCE_SCALE_MIN;

    float base_feet_pos[6][4] = {{122, 152, -66, 1},  {171, 0, -66, 1},  {122, -152, -66, 1},
                                 {-122, 152, -66, 1}, {-171, 0, -66, 1}, {-122, -152, -66, 1}};
    float default_feet_pos[6][4] = {{122, 152, -66, 1},  {171, 0, -66, 1},  {122, -152, -66, 1},
                                    {-122, 152, -66, 1}, {-171, 0, -66, 1}, {-122, -152, -66, 1}};

    GaitType _requestedGait = GaitType::TRI_GATE;
    MOTION_STATE motionState = MOTION_STATE::DEACTIVATED;
    unsigned long lastCommandMillis = 0;
    unsigned long lastMotionMicros = 0;
    unsigned long _lastTunedLog = 0;
    bool commandTimedOut = false;

    ServoAnglesMsg msgAngles = {.angles = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

    static bool isWalkingMode(MOTION_STATE mode) { return mode == MOTION_STATE::WALK || mode == MOTION_STATE::WALK_NN; }

    static bool isActuatedMode(MOTION_STATE mode) {
        return mode == MOTION_STATE::STAND || isWalkingMode(mode) || mode == MOTION_STATE::ANIMATE;
    }

    // Zeroes the locomotion command. `snap` collapses the live gait state onto it as well, for modes
    // that drive nothing: there is no output to jerk, and a half-stride left frozen in gait_state
    // would otherwise resume the moment the robot is put back into STAND. STAND itself only gets the
    // target, so the existing ramp walks the legs down instead of dropping them in one tick.
    void stopLocomotionCommand(bool snap) {
        target_gait_state.step_x = 0;
        target_gait_state.step_y = 0;
        target_gait_state.step_angle = 0;
        target_gait_state.step_speed = 1.f;
        target_gait_state.step_height = DEFAULT_STEP_HEIGHT_MM;
        target_gait_state.step_depth = DEFAULT_STEP_DEPTH;
        target_gait_state.phase_rate = 0.f;
        if (!snap) return;
        approachGaitCommand(gait_state, target_gait_state, 0.f, 0.f);
        gait.setPhase(0.f);
    }

    void applyZeroCommand() {
        target_body_state.xm = 0;
        target_body_state.ym = 0;
        target_body_state.zm = 0;
        target_body_state.phi = 0;
        target_body_state.omega = 0;
        stopLocomotionCommand(false);
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
