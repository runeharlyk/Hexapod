#ifndef MotionService_h
#define MotionService_h

#include <algorithm>
#include <atomic>
#include <iterator>
#include <esp_log.h>
#include <kinematics.h>
#include <peripherals/servo_controller.h>
#include <peripherals/peripherals.h>
#include <utils/timing.h>
#include <utils/math_utils.h>
#include <gait.h>
#include <gait_speed.h>
#include <event_bus.h>
#include <message_types.h>
#include <policy_runner.h>
#include <self_level.h>
#include <animation/animation_runner.h>
#include <animation/mode_arbiter.h>

class MotionService {
  public:
    MotionService(ServoController *servoController, Peripherals *peripherals)
        : _servoController(servoController), _peripherals(peripherals) {
        scaleStance(feetScale(_feetDistance), default_feet_pos);
    }

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
        // foot is commanded straight down. Defer to the next tick that may swap instead, which
        // updateAutoGait() handles from the stepping loops.
        if (!gaitSwappable()) return;
        applyRequestedGait();
    }

    // Loads _requestedGait into the live schedule. Only safe while the gait is not stepping.
    void applyRequestedGait() {
        loadGait(_requestedGait == GaitType::AUTO ? selectAutoGait(commandSpeed01(), gait_state.gait_type)
                                                  : _requestedGait);
    }

    // Mode worker. The rules live in decideMode (animation/mode_arbiter.h); this executes them. A
    // BORROW or HANDBACK is internal, so the mode it applied or kept is published as APPLIED for the
    // observers, which see only REQUEST and APPLIED.
    void handleInputMode(ModeMsg const &m) {
        if (m.kind == ModeMsgKind::APPLIED) return;
        ESP_LOGI("MotionService", "Mode %d%s", (int)m.mode,
                 m.kind == ModeMsgKind::BORROW     ? " (borrow)"
                 : m.kind == ModeMsgKind::HANDBACK ? " (hand-back)"
                                                   : "");
        _handbackSent = false;
        const ModeArbiterDecision d =
            decideMode({motionState, _borrowedMode, _previousMode, m.mode, m.kind, _animation.busy()});
        _borrowedMode = d.borrowed;
        _previousMode = d.previous;
        if (d.cancelPending) _animation.cancelPendingPlay();
        if (d.requestLeave) _animation.requestLeave();
        const bool internal = m.kind != ModeMsgKind::REQUEST;
        switch (d.action) {
            case ModeArbiterDecision::APPLY: break;
            case ModeArbiterDecision::RESTATE: EventBus<ModeMsg>::publish({motionState, ModeMsgKind::APPLIED}); return;
            case ModeArbiterDecision::IGNORE:
            case ModeArbiterDecision::GRACEFUL_LEAVE: return;
        }
        const bool enteringWalk = d.mode == MOTION_STATE::WALK && motionState != MOTION_STATE::WALK;
        motionState = d.mode;
#if FT_ENABLED(USE_POLICY)
        if (d.mode == MOTION_STATE::WALK_NN) _policy.reset(gait, default_feet_pos);
#endif
        // ANIMATE drives no gait, so a half-stride left in gait_state would resume after it.
        if (!isWalkingMode(d.mode)) stopLocomotionCommand(!actuated(d.mode) || d.mode == MOTION_STATE::ANIMATE);
        // WALK_NN keeps the command, and its policy moves the ride height and the feet; a stride or a posture left
        // from before it would be walked through whatever it left. The stick comes back with the next heartbeat.
        if (enteringWalk) stopLocomotionCommand(false);
        motionState == MOTION_STATE::DEACTIVATED ? _servoController->deactivate() : _servoController->activate();
        if (internal) EventBus<ModeMsg>::publish({motionState, ModeMsgKind::APPLIED});
    }

    // Adapter task. A play from any active mode borrows ANIMATE and hands the mode back when the
    // player finishes; a play while already in ANIMATE (borrowed or sticky) chains. The clip is loaded
    // before the mode changes, so a refused play leaves the robot in its mode and pose; whether the
    // borrow is granted is decided by handleInputMode.
    void handleAnimationCommand(AnimationCommandMsg const &c) {
        if (!actuated(motionState)) return;
        if (!_animation.requestPlay(c)) return;
        if (motionState == MOTION_STATE::ANIMATE) return;
        EventBus<ModeMsg>::publish({MOTION_STATE::ANIMATE, ModeMsgKind::BORROW});
    }

    // Adapter task. A stop only makes sense in ANIMATE.
    void handleAnimationStop() {
        if (motionState == MOTION_STATE::ANIMATE) _animation.requestStop();
    }

    // Adapter task, when the last client on every transport has gone. Outside ANIMATE a play may be
    // loaded with its borrow still in flight; dropping it keeps an unattended loop from starting.
    void animationControlLost() {
        if (motionState == MOTION_STATE::ANIMATE) _animation.controlLost();
        else
            _animation.cancelPendingPlay();
    }

    // Adapter task. Puppeteer poses apply only in ANIMATE.
    void handleAnimationPose(PoseMsg const &p) {
        if (motionState == MOTION_STATE::ANIMATE) _animation.setPuppet(p);
    }

    void validateAnimation(const char *name, socket_message_AnimationReport &r) { _animation.validate(name, r); }

    void handleCommand(CommandMsg const &c) {
        lastCommandMillis = millis();
        lastCommand = c;
        commandTimedOut = false;
        const MOTION_STATE mode = motionState;
        steering = isWalkingMode(mode) && (c.lx != 0.f || c.ly != 0.f || c.rx != 0.f);
        CommandMsg clamped = c;
        clampPosture(clamped.s1, clamped.h, clamped.fd);
        // The searched gait rides at its own height, outside the grid the envelope was measured on.
        const bool tuned = mode == MOTION_STATE::WALK && gait_state.gait_type == GaitType::TUNED;
        // A classic walk takes its posture between steps only (settleWalkPosture); every other mode takes it now.
        if (mode != MOTION_STATE::WALK || tuned) {
            float h = c.h, fd = c.fd;
            if (mode == MOTION_STATE::WALK_NN) {
                h = clamped.h;
                fd = clamped.fd;
            } else if (!tuned) {
                clampStandPosture(kinematics, h, fd);
            }
            _walkPostureValid = false;
            if (mode == MOTION_STATE::STAND) {
                // A walk left for STAND may still be ramping down or have feet in the air, and a stance change may
                // still be stepping; a posture taken under any of them is stepped through. takeStandPosture waits
                // until the feet are still.
                _standPosture = {0.f, h, fd};
                _standPosturePending = true;
            } else {
                _standPosturePending = false;
                updateFeetDistanceTarget(fd);
                target_body_state.zm = bodyHeightMm(h);
            }
        }
        target_body_state.omega = c.ry * BODY_TILT_LIMIT_RAD;
        switch (mode) {
            case MOTION_STATE::STAND: {
                target_body_state.xm = c.lx * 50.f;
                target_body_state.ym = -c.ly * 50.f;
                target_body_state.phi = c.rx * BODY_TILT_LIMIT_RAD;
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
                // Kept whatever the gait, so a classic gait loaded later walks this stick and no older one.
                lastWalkCommand_ = clamped;
                if (tuned) {
                    // It rides at its own height and lift, so a classic walk after it cannot start in the stand's.
                    _atRest = false;
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
                    target_gait_state.step_speed = c.s + 1.f;
                    target_gait_state.phase_rate =
                        tuned_gait::PHASE_RATE_MIN +
                        (a[5] + 1.f) * 0.5f * (tuned_gait::PHASE_RATE_MAX - tuned_gait::PHASE_RATE_MIN);
                    target_body_state.zm = -tuned_gait::RIDE_MM;
                    logTunedCommand(cmd, target_gait_state);
                } else {
                    target_gait_state.step_depth = 0.002f;
                    // The envelope sets the cadence; step_speed is left to pace a stance change.
                    target_gait_state.step_speed = 1.f;
                    takeRestPosture(clamped);
                    applyWalkTarget();
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

    // The commanded share of the top speed, the speed signal AUTO switches on.
    float commandSpeed01() const { return _walkSpeed01; }

    // Schedules cannot be swapped while stepping: changing offset[] reassigns every leg's phase at
    // once, and a leg at the top of its swing in one gait is a stance leg in the next, so its foot is
    // commanded straight down -- measured at 78 mm in a single tick. Retiming legs individually to
    // avoid that is a real transition problem and is deliberately not attempted here.
    //
    // Instead switch only while the gait is not stepping. selectAutoGait reads the TARGET command,
    // so the ramp's window at each start and stop is enough to choose the gait for the walk about to
    // happen. The cost is that changing gait at speed needs a brief stop.
    void updateAutoGait() {
        if (!gaitSwappable()) return;

        // A manual selection that arrived mid-stride was deferred; this is the window to honour it.
        if (_requestedGait != GaitType::AUTO) {
            if (_requestedGait == gait_state.gait_type) return;
            applyRequestedGait();
            ESP_LOGI("MotionService", "Deferred gait -> %d", (int)gait_state.gait_type);
            return;
        }

        const GaitType next = selectAutoGait(commandSpeed01(), gait_state.gait_type);
        if (next == gait_state.gait_type) return;
        loadGait(next);
        ESP_LOGI("MotionService", "Auto gait -> %d", (int)next);
    }

    // A stance change steps the feet with a zero stroke, so a gait that is not commanded can still have a foot in the
    // air; a posture being settled into steps them next. Either way a swap would land under a swinging foot.
    bool gaitSwappable() const {
        return !gaitIsCommanded(gait_state) && !gait.hasPendingStanceChange() && !_postureSettling;
    }

    bool updateMotion() {
        resetCommandIfTimedOut();
        const float dt = getMotionDeltaSeconds();
        const MOTION_STATE state = motionState;
        if (state != MOTION_STATE::STAND && state != MOTION_STATE::WALK) _atRest = false;
        // Entering and leaving ANIMATE are handled here rather than in handleInputMode so the runner
        // is only ever touched by the control task.
        if ((state == MOTION_STATE::ANIMATE) != _animating) {
            _animating = state == MOTION_STATE::ANIMATE;
            if (_animating)
                _animation.enter(body_state, target_body_state.zm);
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
                levelAttitude(body_state, target_body_state.phi, target_body_state.omega, _peripherals->angleX(),
                              _peripherals->angleY());
                takeStandPosture();
                // STAND is not stepping, so this is where a gait selection deferred by
                // handleInputGait gets honoured -- otherwise leaving WALK would strand it.
                updateAutoGait();
                approachGaitCommand(gait_state, target_gait_state, dt, GAIT_COMMAND_TAU_S);
                gait.step(gait_state, body_state, dt);
                kinematics.inverseKinematics(body_state, msgAngles.angles);
                noteRestPosture();
                break;
            }
#if FT_ENABLED(USE_POLICY)
            case MOTION_STATE::WALK_NN: {
                // The policy owns stride, lift, cadence and ride height; it was trained with the
                // firmware's self-levelling active, so that term stays on here too.
                levelAttitude(body_state, 0.f, 0.f, _peripherals->angleX(), _peripherals->angleY());
                if (!_policy.update(_peripherals, gait, body_state, msgAngles.angles)) return false;
                kinematics.inverseKinematics(body_state, msgAngles.angles);
                break;
            }
#endif
            case MOTION_STATE::ANIMATE: {
                // tick writes the angles itself; body_state carries the absolute pose for telemetry.
                _animation.tick(dt, target_body_state.zm, body_state, msgAngles.angles);
                // Only asks: the worker decides, so a mode it handled first (a stop, a sticky ANIMATE)
                // wins. The flag is raised before the publish so the worker's clear cannot be lost.
                if (_borrowedMode && !_handbackSent && !_animation.busy()) {
                    _handbackSent = true;
                    if (!EventBus<ModeMsg>::publish({_previousMode, ModeMsgKind::HANDBACK})) _handbackSent = false;
                }
                break;
            }
            case MOTION_STATE::WALK: {
                body_state.xm = lerpf(body_state.xm, target_body_state.xm, smoothing_factor);
                body_state.ym = lerpf(body_state.ym, target_body_state.ym, smoothing_factor);
                body_state.zm = lerpf(body_state.zm, target_body_state.zm, smoothing_factor);
                levelAttitude(body_state, target_body_state.phi, target_body_state.omega, _peripherals->angleX(),
                              _peripherals->angleY());
                // A walk that steps before its first classic command, TUNED's, leaves the stand's posture.
                if (gaitIsCommanded(gait_state)) _atRest = false;
                settleWalkPosture();
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

    // The locomotion command the gait eases toward, and the body pose the motion eases toward, for the virtual robot's
    // tests.
    const gait_state_t &gaitCommand() const { return target_gait_state; }
    const BodyStateMsg &bodyCommand() const { return target_body_state; }

    // The motion's half of a telemetry tick: what it produced and from which command and mode.
    void fillTelemetry(socket_message_TickSample &tick) const {
        std::copy(std::begin(msgAngles.angles), std::end(msgAngles.angles), tick.angles);
        const float command[8] = {lastCommand.lx, lastCommand.ly, lastCommand.rx, lastCommand.ry,
                                  lastCommand.h,  lastCommand.s,  lastCommand.s1, lastCommand.fd};
        std::copy(std::begin(command), std::end(command), tick.command);
        tick.command_age_ms = lastCommandMillis == 0 ? 0 : millis() - lastCommandMillis;
        tick.mode = static_cast<uint32_t>(motionState);
        tick.gait = static_cast<uint32_t>(gait_state.gait_type);
    }

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
    std::atomic<bool> _handbackSent {false}; // a hand-back is queued; cleared by the worker on any mode
    bool _animating = false;                 // control task only: whether the last tick ran ANIMATE

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

    static constexpr float smoothing_factor = BODY_SMOOTHING;
    // Ramp time constant for the locomotion command: ~0.45 s to settle. Slow enough that a slammed
    // stick does not jerk the stride, fast enough that the robot still feels directly driven.
    static constexpr float GAIT_COMMAND_TAU_S = 0.15f;
    static constexpr unsigned long COMMAND_TIMEOUT_MS = 2000;
    // A walking robot covers ~0.9 m in COMMAND_TIMEOUT_MS. While a stick drives locomotion the
    // senders repeat it (app heartbeat 200 ms, ESP-NOW controller 50 Hz), so silence for three
    // heartbeats means the link is gone and the robot must stop walking.
    static constexpr unsigned long STEERING_TIMEOUT_MS = 600;

    float default_feet_pos[6][4];

    GaitType _requestedGait = GaitType::TRI_GATE;
    // The last WALK stick, posture clamped, and the share of a classic gait's top speed it asked for;
    // both are cleared when the locomotion command stops.
    CommandMsg lastWalkCommand_ {};
    float _walkSpeed01 = 0.f;
    struct Posture {
        float s1, h, fd;
    };
    // The posture the classic walk stands in, valid once it was taken between steps (settleWalkPosture) or from the
    // stand the walk was entered from (takeRestPosture).
    Posture _walkPosture {};
    bool _walkPostureValid = false;
    // The posture of a stand at rest, its lift as an s1, for a walk entered next: the targets the stand holds, which
    // the control task rewrites unchanged every STAND tick while _atRest holds, so the command worker's read cannot
    // tear. Anything that moves the robot in WALK before the first classic command takes it consumes it (clears
    // _atRest).
    Posture _restPosture {};
    std::atomic<bool> _atRest {false};
    // A posture is waiting for the gait to stop, then for the robot to settle; the walk is held at zero meanwhile.
    std::atomic<bool> _posturePending {false};
    std::atomic<bool> _postureSettling {false};
    // The stand posture last commanded, waiting until the feet are still (takeStandPosture).
    Posture _standPosture {};
    std::atomic<bool> _standPosturePending {false};
    enum class SettleStage : uint8_t { LOWER_LIFT, MOVE, RAISE_LIFT };
    SettleStage _settleStage = SettleStage::LOWER_LIFT; // control task, while settling
    bool _heightAfterStance = false;                    // control task: the body height waits for the stance
    float _feetDistance = 0.f;                          // the stance last commanded, as its fd
    // A posture change below this, per slider, is not worth a stop and is not taken while walking.
    static constexpr float POSTURE_DEADBAND = 1.f / 32.f;
    static constexpr float POSTURE_SETTLED_MM = 0.5f;
    // Past the stance change's own end, which is anywhere within 0.5 mm of its target (hasPendingStanceChange).
    static constexpr float FEET_AT_REST_MM = 1.0f;
    MOTION_STATE motionState = MOTION_STATE::DEACTIVATED;
    unsigned long lastCommandMillis = 0;
    unsigned long lastMotionMicros = 0;
    unsigned long _lastTunedLog = 0;
    bool commandTimedOut = false;
    bool steering = false;
    // Written on the command worker, read by the control task for telemetry only; a torn read costs
    // one sample's command, never the motion.
    CommandMsg lastCommand{};

    ServoAnglesMsg msgAngles = {.angles = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

  public:
    // Lying on its belly with the legs limp or folded: still enough to calibrate the IMU against.
    bool resting() const { return motionState == MOTION_STATE::DEACTIVATED || motionState == MOTION_STATE::IDLE; }

  private:
    static bool isWalkingMode(MOTION_STATE mode) { return mode == MOTION_STATE::WALK || mode == MOTION_STATE::WALK_NN; }

    // Only safe while the gait is not stepping.
    void loadGait(GaitType next) {
        // The searched gait rides at its own height and lift, so the classic walk's posture is unknown either side.
        if ((next == GaitType::TUNED) != (gait_state.gait_type == GaitType::TUNED)) {
            forgetWalkPosture();
            _atRest = false;
        }
        gait_state.gait_type = next;
        gait.setGait(gait_state);
        // The schedule switches outright; only the command eases. Keep the target's identity fields
        // in step so a later ramp never reads a stale gait.
        target_gait_state.gait_type = gait_state.gait_type;
        target_gait_state.stand_frac = gait_state.stand_frac;
        for (int i = 0; i < 6; i++) target_gait_state.offset[i] = gait_state.offset[i];
        // Top speeds differ several times between gaits, so a held stick's stroke and cadence are the
        // new gait's from this tick on, not from the next command.
        applyWalkTarget();
    }

    // The classic gaits' stroke and cadence for the last stick, from the speed envelope at the posture in effect. A
    // posture further than POSTURE_DEADBAND from that one is never walked into: the parts of a posture arrive at their
    // own rates (the height in a tenth of a second, the stance only swing by swing), and the mixed postures between
    // can leave the joints' travel. So the walk stops, settleWalkPosture takes the new posture once the gait is still,
    // and the walk resumes once the robot has settled into it.
    void applyWalkTarget() {
        if (motionState != MOTION_STATE::WALK || gait_state.gait_type == GaitType::TUNED) return;
        const CommandMsg &c = lastWalkCommand_;
        if (!_walkPostureValid || postureOffset(c) > POSTURE_DEADBAND) _posturePending = true;
        const bool held = _posturePending || _postureSettling;
        const Posture p = held ? Posture {c.s1, c.h, c.fd} : _walkPosture;
        const WalkTarget w = classicWalkTarget(gait_state.gait_type, c.lx, c.ly, c.rx, c.s, p.s1, p.h, p.fd);
        // Held, it still tells AUTO how fast the walk will be, so the gait is chosen while the walk waits.
        _walkSpeed01 = w.speed01;
        target_gait_state.step_x = held ? 0.f : w.stepX;
        target_gait_state.step_y = held ? 0.f : w.stepY;
        target_gait_state.step_angle = held ? 0.f : w.stepAngle;
        target_gait_state.phase_rate = held ? 0.f : w.phaseRate;
    }

    float postureOffset(const CommandMsg &c) const {
        return std::max(
            {std::fabs(c.s1 - _walkPosture.s1), std::fabs(c.h - _walkPosture.h), std::fabs(c.fd - _walkPosture.fd)});
    }

    // Control task, classic walk. Takes a pending posture once the gait has stopped stepping, as a stand takes one,
    // and releases the walk once the robot has settled into it. Each stage waits for the one before to arrive:
    // - the lift comes down to the stand's, since the walk's own, up to 40 mm, leaves the femur's travel when the
    //   stance is stepped into;
    // - the stance and the body height move, the stance stepped at the lower of the two heights, since a foot lifted
    //   under a raised body leaves it too: a body going up waits for the stance, one going down leads it, unless the
    //   legs could not stand through that order;
    // - the lift goes up to the walk's.
    void settleWalkPosture() {
        if (gait_state.gait_type == GaitType::TUNED) return;
        if (_posturePending && !gaitIsCommanded(gait_state)) {
            // Settling is raised first, so the command worker holds the walk at zero throughout.
            _postureSettling = true;
            _posturePending = false;
            const CommandMsg c = lastWalkCommand_;
            _walkPosture = {c.s1, c.h, c.fd};
            _walkPostureValid = true;
            _settleStage = SettleStage::LOWER_LIFT;
            target_gait_state.step_height = DEFAULT_STEP_HEIGHT_MM;
        }
        if (!_postureSettling) return;
        switch (_settleStage) {
            case SettleStage::LOWER_LIFT: {
                // A stance retargeted mid-swing moves the swinging foot's anchor in one tick.
                if (!liftArrived() || !feetAtRest()) return;
                const float heldHeight = target_body_state.zm / bodyHeightMm(1.f);
                _heightAfterStance = _walkPosture.h > heldHeight;
                if (!(_heightAfterStance ? standFeasible(kinematics, heldHeight, _walkPosture.fd)
                                         : standFeasible(kinematics, _walkPosture.h, _feetDistance)))
                    _heightAfterStance = !_heightAfterStance;
                if (!_heightAfterStance) target_body_state.zm = bodyHeightMm(_walkPosture.h);
                updateFeetDistanceTarget(_walkPosture.fd);
                _settleStage = SettleStage::MOVE;
                return;
            }
            case SettleStage::MOVE: {
                if (std::fabs(body_state.zm - target_body_state.zm) >= POSTURE_SETTLED_MM || !feetAtRest()) return;
                if (_heightAfterStance) {
                    _heightAfterStance = false;
                    target_body_state.zm = bodyHeightMm(_walkPosture.h);
                    return;
                }
                target_gait_state.step_height = (_walkPosture.s1 + 1.f) * 20.f;
                _settleStage = SettleStage::RAISE_LIFT;
                return;
            }
            case SettleStage::RAISE_LIFT: {
                if (!liftArrived()) return;
                _postureSettling = false;
                applyWalkTarget();
                return;
            }
        }
    }

    // Control task, at the end of a STAND tick. A stand that has taken its posture and holds still is one a walk can
    // start from. A stand posture still pending has not moved anything, and entering WALK drops it.
    void noteRestPosture() {
        const bool atRest = !gaitIsCommanded(gait_state) && feetAtRest() &&
                            std::fabs(body_state.zm - target_body_state.zm) < POSTURE_SETTLED_MM && liftArrived();
        if (atRest)
            _restPosture = {target_gait_state.step_height / 20.f - 1.f, target_body_state.zm / bodyHeightMm(1.f),
                            _feetDistance};
        _atRest = atRest;
    }

    // Command worker, the first classic walk command after a stand at rest. A command that keeps the stand's height
    // and stance walks at once in it: only the lift differs, and a lift that rises eases in alongside the stroke,
    // which keeps every state between inside the envelope (checked over its walkable grid). One that falls does not:
    // the stand's lift over the posture's stroke can leave travel, so that walk, like any other, settles first.
    void takeRestPosture(const CommandMsg &c) {
        if (_walkPostureValid || _postureSettling || !_atRest.exchange(false)) return;
        const Posture rest = _restPosture;
        if (std::fabs(c.h - rest.h) > POSTURE_DEADBAND || std::fabs(c.fd - rest.fd) > POSTURE_DEADBAND ||
            c.s1 < rest.s1 - POSTURE_DEADBAND || !postureWalkable(c.s1, rest.h, rest.fd))
            return;
        _walkPosture = {c.s1, rest.h, rest.fd};
        _walkPostureValid = true;
        _posturePending = false;
        target_gait_state.step_height = (c.s1 + 1.f) * 20.f;
    }

    // Control task, STAND.
    void takeStandPosture() {
        if (!_standPosturePending || gaitIsCommanded(gait_state) || !feetAtRest()) return;
        _standPosturePending = false;
        const Posture p = _standPosture;
        updateFeetDistanceTarget(p.fd);
        target_body_state.zm = bodyHeightMm(p.h);
    }

    // Every foot stands where the stance puts it: no swing, no stance change and no ease home is under way. A stopped
    // walk leaves feet in the air, and BI, at a third stance, leaves most of them there.
    bool feetAtRest() const {
        if (gait.hasPendingStanceChange()) return false;
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 3; ++j)
                if (std::fabs(body_state.feet[i][j] - default_feet_pos[i][j]) >= FEET_AT_REST_MM) return false;
        return true;
    }

    bool liftArrived() const {
        return std::fabs(gait_state.step_height - target_gait_state.step_height) < POSTURE_SETTLED_MM;
    }

    // Zeroes the locomotion command. `snap` collapses the live gait state onto it as well, for modes
    // that drive nothing: there is no output to jerk, and a half-stride left frozen in gait_state
    // would otherwise resume the moment the robot is put back into STAND. STAND itself only gets the
    // target, so the existing ramp walks the legs down instead of dropping them in one tick.
    // The next classic walk takes its posture again, unless it starts from a stand at rest in it (takeRestPosture).
    void forgetWalkPosture() {
        _walkPostureValid = false;
        _posturePending = false;
        _postureSettling = false;
    }

    void stopLocomotionCommand(bool snap) {
        target_gait_state.step_x = 0;
        target_gait_state.step_y = 0;
        target_gait_state.step_angle = 0;
        target_gait_state.step_speed = 1.f;
        target_gait_state.step_height = DEFAULT_STEP_HEIGHT_MM;
        target_gait_state.step_depth = DEFAULT_STEP_DEPTH;
        target_gait_state.phase_rate = 0.f;
        // Else a gait loaded later, AUTO's included, would walk the stick the dead-man stopped.
        lastWalkCommand_ = {};
        _walkSpeed01 = 0.f;
        forgetWalkPosture();
        // A stand posture waits only within the command stream it came in; the dead-man and a new mode end that.
        _standPosturePending = false;
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

    void updateFeetDistanceTarget(float normalizedFeetDistance) {
        _feetDistance = normalizedFeetDistance;
        scaleStance(feetScale(normalizedFeetDistance), default_feet_pos);
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
        // The app sends no heartbeat in ANIMATE, and the animation base follows the height slider, so
        // zeroing it would drop the body mid-play. The window restarts instead, so the app has a full
        // window after a hand-back to resume its heartbeat.
        if (motionState == MOTION_STATE::ANIMATE) {
            lastCommandMillis = millis();
            return;
        }
        if (commandTimedOut) return;
        const unsigned long timeout = steering ? STEERING_TIMEOUT_MS : COMMAND_TIMEOUT_MS;
        if (millis() - lastCommandMillis < timeout) return;

        ESP_LOGW("MotionService", "No motion command for %lu ms, applying zero command", timeout);
        applyZeroCommand();
        commandTimedOut = true;
        steering = false;
    }
};

#endif
