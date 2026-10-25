#ifndef MotionService_h
#define MotionService_h

#include <algorithm>
#include <iterator>
#include <utility>
#include <esp_log.h>
#include <kinematics.h>
#include <peripherals/servo_controller.h>
#include <peripherals/peripherals.h>
#include <utils/timing.h>
#include <utils/math_utils.h>
#include <gait.h>
#include <gait_speed.h>
#include <imu_frame.h>
#include <odometry.h>
#include <goal_controller.h>
#include <event_bus.h>
#include <message_types.h>
#include <policy_runner.h>
#include <self_level.h>
#include <animation/animation_runner.h>
#include <animation/mode_arbiter.h>
#include <motion_inbox.h>

/** The goal status the motion pushes: a few pushes a tick at most, to the app's bridge. */
using GoalStatusBus = EventBus<socket_message_GoalStatus, 4, 2, 1>;

class MotionService {
  public:
    MotionService(ServoController *servoController, Peripherals *peripherals)
        : _servoController(servoController), _peripherals(peripherals) {
        scaleStance(feetScale(_feetDistance), default_feet_pos);
    }

    void begin() {
        _animation.begin(kinematics, default_feet_pos);
        ESP_LOGI("MotionService", "Subscribing to event buses...");
        _cmdSubHandle = EventBus<CommandMsg>::subscribe([&](CommandMsg const &c) { _inbox.postCommand(c, millis()); });
        _modeSubHandle = EventBus<ModeMsg>::subscribe([&](ModeMsg const &m) { _inbox.postMode(m, _animation.busy()); });
        _gaitSubHandle = EventBus<GaitMsg>::subscribe([&](GaitMsg const &g) { _inbox.postGait(g.gait); });
        _servoSwitchSubHandle =
            EventBus<ServoStateMsg>::subscribe([&](ServoStateMsg const &s) { _inbox.postServoSwitch(s.active); });
        ESP_LOGI("MotionService", "Event bus subscriptions completed");
        body_state.updateFeet(default_feet_pos);
        EventBus<ModeMsg>::publish({motionState});
        EventBus<GaitMsg>::publish({gait_state.gait_type});
    }

  private:
    // Control task, at the start of the tick after the gait arrived; the goal it ends was ended on arrival.
    void handleInputGait(GaitType gait) {
        _requestedGait = gait;
        // Swapping a schedule mid-stride is the hazard updateAutoGait() documents: offset[] reassigns
        // every leg's phase at once, so a leg at the top of its swing becomes a stance leg and its
        // foot is commanded straight down. Defer to the next tick that may swap instead, which
        // updateAutoGait() handles from the stepping loops.
        if (!gaitSwappable()) return;
        applyRequestedGait();
    }

    // Loads _requestedGait into the live schedule. Only safe while the gait is not stepping.
    void applyRequestedGait() {
        const GaitType requested = _requestedGait;
        loadGait(requested == GaitType::AUTO ? _autoGait : requested);
    }

    // Control task, at the start of the tick after the mode arrived, in arrival order. The rules live in decideMode
    // (animation/mode_arbiter.h); this executes them. The goal a mode ends was ended on arrival
    // (MotionInbox::postMode). Every mode applied, and the mode a refused borrow keeps, is published as APPLIED: the
    // robot reports a mode only once it is in it, and the observers that show the mode forward APPLIED alone. The mode
    // is noted for the replies before it is reported, so a client that heard the report finds it in force. The ease out
    // of IDLE a mode of this tick borrowed has not begun yet, and is busy already: a mode after it waits for it.
    void handleInputMode(ModeMsg const &m) {
        _handbackSent = false;
        const ModeArbiterDecision d = decideMode({motionState, _borrowedMode, _previousMode, m.mode, m.kind,
                                                  _animation.busy() || _enterFromHeld, _idleHeld});
        _borrowedMode = d.borrowed;
        _previousMode = d.previous;
        if (d.cancelPending) _animation.cancelPendingPlay();
        if (d.requestLeave) _animation.requestLeave();
        if (d.action == ModeArbiterDecision::APPLY) applyMode(d.mode);
        noteModeState();
        if (d.action == ModeArbiterDecision::APPLY || d.action == ModeArbiterDecision::RESTATE)
            EventBus<ModeMsg>::publish({motionState, ModeMsgKind::APPLIED});
    }

    // Control task. Whether the handheld drives is reported with the mode, so a change of it reports the mode applied
    // again: the mode's observers see the same mode, and the app's bridge reads the flag as it pushes the report.
    void reportHandheldDriving(bool driving) {
        if (driving == _handheldReported) return;
        _handheldReported = driving;
        EventBus<ModeMsg>::publish({motionState, ModeMsgKind::APPLIED});
    }

    // Control task.
    void applyMode(MOTION_STATE mode) {
        const bool enteringWalk = mode == MOTION_STATE::WALK && motionState != MOTION_STATE::WALK;
        const bool leavingPolicy = motionState == MOTION_STATE::WALK_NN;
        const bool deactivating = mode == MOTION_STATE::DEACTIVATED && motionState != MOTION_STATE::DEACTIVATED;
        // ANIMATE entered from an IDLE whose servos were written eases from the angles they hold (decideMode).
        if (mode != MOTION_STATE::ANIMATE)
            _enterFromHeld = false;
        else if (motionState == MOTION_STATE::IDLE && _idleHeld)
            _enterFromHeld = true;
        // ANIMATE's pose, a play's or the ease's half done, is no motion's: IDLE holds it as one it wrote.
        if (mode != MOTION_STATE::IDLE)
            _idleHeld = false;
        else if (motionState == MOTION_STATE::ANIMATE)
            _idleHeld = true;
        motionState = mode;
#if FT_ENABLED(USE_POLICY)
        // The policy was trained on the stance of fd 0 alone, so WALK_NN walks it whatever the feet distance, which
        // the stance takes again, stepped, once WALK_NN is left.
        if (mode == MOTION_STATE::WALK_NN) {
            std::copy(&DEFAULT_STANCE[0][0], &DEFAULT_STANCE[0][0] + 6 * 4, &default_feet_pos[0][0]);
            _policy.reset(gait, DEFAULT_STANCE);
        }
#endif
        if (leavingPolicy && mode != MOTION_STATE::WALK_NN) updateFeetDistanceTarget(_feetDistance);
        // ANIMATE drives no gait, so a half-stride left in gait_state would resume after it.
        if (!isWalkingMode(mode)) stopLocomotionCommand(!actuated(mode) || mode == MOTION_STATE::ANIMATE);
        // WALK_NN keeps the command, and its policy moves the ride height and the feet; a stride or a posture left
        // from before it would be walked through whatever it left. The stick comes back with the next heartbeat.
        // The classic stroke WALK_NN left in gait_state is snapped out too: ramped out from the feet the policy left,
        // it took a joint past its travel at some phases of the walk before.
        if (enteringWalk) stopLocomotionCommand(leavingPolicy);
        mode == MOTION_STATE::DEACTIVATED ? _servoController->deactivate() : _servoController->activate();
        // Every servo goes limp, so the angles a client is answered with are none rather than the pose they held.
        if (deactivating) publishState();
    }

  public:
    // Adapter task. A play from any active mode borrows ANIMATE and hands the mode back when the
    // player finishes; a play while already in ANIMATE (borrowed or sticky) chains. The clip is loaded
    // before the mode changes, so a refused play leaves the robot in its mode and pose; whether the
    // borrow is granted is decided by handleInputMode. The play judges the mode the control task applied
    // last (MotionInbox::appliedMode); a stop, a pose and a lost control judge MotionInbox::animating.
    void handleAnimationCommand(AnimationCommandMsg const &c) {
        const MOTION_STATE mode = _inbox.appliedMode();
        if (!actuated(mode)) return;
        if (!_animation.requestPlay(c)) return;
        if (mode == MOTION_STATE::ANIMATE) return;
        EventBus<ModeMsg>::publish({MOTION_STATE::ANIMATE, ModeMsgKind::BORROW});
    }

    // Adapter task. A stop only makes sense in ANIMATE, which a play's borrow still queued is about to enter.
    void handleAnimationStop() {
        if (_inbox.animating()) _animation.requestStop();
    }

    // Adapter task, when the last client on every transport has gone. Outside ANIMATE a play may be
    // loaded with its borrow still in flight; dropping it keeps an unattended loop from starting.
    void animationControlLost() {
        if (_inbox.animating())
            _animation.controlLost();
        else
            _animation.cancelPendingPlay();
    }

    // Adapter task. Puppeteer poses apply only in ANIMATE, or once a queued borrow of it is applied.
    void handleAnimationPose(PoseMsg const &p) {
        if (_inbox.animating()) _animation.setPuppet(p);
    }

    void validateAnimation(const char *name, socket_message_AnimationReport &r) { _animation.validate(name, r); }

    // Adapter task: a joint dragged on the app's 3D model, in degrees as the robot reports its angles. It moves that
    // servo alone, in IDLE alone (MotionInbox::postJoint), from the control task's next tick. A servo that does not
    // exist or an angle that is no number is dropped here, as input from the network.
    void setJoint(int servo, float degrees) {
        if (servo < 0 || servo >= NUM_SERVO || !std::isfinite(degrees)) return;
        _inbox.postJoint(servo, degrees);
    }

    // Adapter task: the reply to the calibration page's "Centre all servos", 200 when the control task centres every
    // servo at its next tick, 409 when the robot is not idle or a mode is on its way.
    int requestCentreServos() { return _inbox.postCentreServos() ? 200 : 409; }

  private:
    // Control task, at the start of the tick after the command arrived; the dead-man's window runs from its arrival.
    void handleCommand(CommandMsg const &c, unsigned long arrivalMs) {
        lastCommandMillis = arrivalMs;
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
                if (mode != MOTION_STATE::WALK_NN) updateFeetDistanceTarget(fd);
                target_body_state.zm = bodyHeightMm(h);
            }
        }
        const float tiltLimit = mode == MOTION_STATE::STAND ? STAND_TILT_LIMIT_RAD : BODY_TILT_LIMIT_RAD;
        target_body_state.omega = c.ry * tiltLimit;
        switch (mode) {
            case MOTION_STATE::STAND: {
                // The body moves by the inverse of this pose (docs/conventions.md): +lx shifts it right, +ly forward,
                // +rx lowers its right side and +ry its nose.
                target_body_state.xm = -c.lx * 50.f;
                target_body_state.ym = -c.ly * 50.f;
                target_body_state.phi = -c.rx * STAND_TILT_LIMIT_RAD;
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
                    // Forward, rightward and counter-clockwise, as classicWalkTarget signs its stroke.
                    const float cmd[3] = {c.ly * TUNED_CMD_VX_MAX, c.lx * TUNED_CMD_VY_MAX, -c.rx * TUNED_CMD_YAW_MAX};
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
                    std::copy(std::begin(cmd), std::end(cmd), std::begin(_tunedVelocity));
                    _tunedLogDue = true;
                } else {
                    walkClassic(clamped);
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

    // The speed the last stick asks for, as a share of BI's top speed at the posture: AUTO's speed signal, the same
    // whichever gait walks it.
    float commandSpeed01() const { return _walkSpeed01; }

    // Schedules cannot be swapped while stepping: changing offset[] reassigns every leg's phase at
    // once, and a leg at the top of its swing in one gait is a stance leg in the next, so its foot is
    // commanded straight down -- measured at 78 mm in a single tick. Retiming legs individually to
    // avoid that is a real transition problem and is deliberately not attempted here.
    //
    // Instead switch only while the gait is not stepping. AUTO chooses from the TARGET command,
    // so the ramp's window at each start and stop is enough to choose the gait for the walk about to
    // happen. The cost is that changing gait at speed needs a brief stop, and a stick pushed past the
    // top of the gait in effect walks at that top until the next stop (autoWalkTarget).
    void updateAutoGait() {
        if (!gaitSwappable()) return;

        // A manual selection that arrived mid-stride was deferred; this is the window to honour it.
        if (_requestedGait != GaitType::AUTO) {
            if (_requestedGait == gait_state.gait_type) return;
            applyRequestedGait();
            ESP_LOGI("MotionService", "Deferred gait -> %d", (int)gait_state.gait_type);
            return;
        }

        const GaitType next = _autoGait;
        if (next == gait_state.gait_type) return;
        loadGait(next);
        ESP_LOGI("MotionService", "Auto gait -> %d at %.2f of BI's top speed", (int)next, commandSpeed01());
    }

    // A stance change steps the feet with a zero stroke, so a gait that is not commanded can still have a foot in the
    // air; a posture being settled into steps them next. Either way a swap would land under a swinging foot.
    bool gaitSwappable() const {
        return !gaitIsCommanded(gait_state) && !gait.hasPendingStanceChange() && !_postureSettling;
    }

  public:
    // imuFresh: the IMU was read on this tick, so its DMP packet and gyro rate are this tick's.
    bool updateMotion(bool imuFresh) {
        const MotionInbox::Mail mail = _inbox.take(millis());
        if (mail.servoSwitch) switchServos(*mail.servoSwitch);
        // Accepted in IDLE with no mode queued (MotionInbox::postCentreServos): before every mode taken with it.
        if (mail.centreServos) centreServos();
        _modesTaken = mail.modes;
        _gaitTaken = mail.gait;
        for (int i = 0; i < mail.modes.count; ++i) handleInputMode(mail.modes.items[i]);
        reportHandheldDriving(mail.handheldDriving);
        if (mail.gait) handleInputGait(*mail.gait);
        if (mail.command) handleCommand(*mail.command, mail.commandMs);
        takeGoalMail(mail);
        resetCommandIfTimedOut();
        const float dt = getMotionDeltaSeconds();
        const MOTION_STATE state = motionState;
        if (mail.odometryReset) {
            _odometry.reset();
            _inbox.noteOdometry(_odometry.estimate());
        }
        if (state != MOTION_STATE::STAND && state != MOTION_STATE::WALK) {
            _atRest = false;
            // The odometry holds its pose; whatever moves the feet meanwhile is no travel it can measure.
            _odometry.pause();
        }
        // Entering and leaving ANIMATE are handled here, once every mode the tick took is applied. An ANIMATE taken
        // out of IDLE-held angles in a tick that began in ANIMATE left it for that IDLE and enters it again: the IDLE
        // ended what ANIMATE ran, and the ease starts from the angles it holds.
        const bool animating = state == MOTION_STATE::ANIMATE;
        const bool fromHeld = animating && std::exchange(_enterFromHeld, false);
        if (_animating && (!animating || fromHeld)) _animation.reset();
        if (animating && (!_animating || fromHeld)) {
            if (fromHeld)
                _animation.enterFromJoints(msgAngles.angles, _servoController->driven(), target_body_state.zm);
            else
                _animation.enter(body_state, target_body_state.zm);
        }
        _animating = animating;
        const bool enteringIdle = state == MOTION_STATE::IDLE && !_idling;
        _idling = state == MOTION_STATE::IDLE;
        switch (state) {
            case MOTION_STATE::DEACTIVATED: return false;
            case MOTION_STATE::IDLE: driveIdleServos(mail, enteringIdle); return false;
            case MOTION_STATE::STAND: {
                body_state.xm = lerpf(body_state.xm, target_body_state.xm, smoothing_factor);
                body_state.ym = lerpf(body_state.ym, target_body_state.ym, smoothing_factor);
                body_state.zm = lerpf(body_state.zm, target_body_state.zm, smoothing_factor);
                levelAttitude(body_state, target_body_state.phi, target_body_state.omega, _peripherals->angleX(),
                              _peripherals->angleY(), STAND_TILT_LIMIT_RAD);
                // A walk left for the stand ramps out at its own lift: another lift, a falling one above all, can leave
                // travel under the stroke still being stepped. The stand steps a stance change at its own lift.
                if (!gaitIsCommanded(gait_state)) target_gait_state.step_height = DEFAULT_STEP_HEIGHT_MM;
                takeStandPosture();
                // STAND is not stepping, so this is where a gait selection deferred by
                // handleInputGait gets honoured -- otherwise leaving WALK would strand it.
                updateAutoGait();
                _commandRamp.step(gait_state, target_gait_state, dt, GAIT_COMMAND_TAU_S);
                gait.step(gait_state, body_state, dt);
                updateOdometry(dt, imuFresh);
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
                // Only asks: the hand-back queues behind every mode that arrived before it and is decided when
                // taken, so a mode taken first (a stop, a sticky ANIMATE) wins.
                if (_borrowedMode && !_handbackSent && !_animation.busy()) {
                    _handbackSent = true;
                    _inbox.postMode({_previousMode, ModeMsgKind::HANDBACK}, false);
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
                slewStride(dt);
                // Before the ramp: it clears the 2 mm deadband in one tick, closing the window.
                updateAutoGait();
                _commandRamp.step(gait_state, target_gait_state, dt, GAIT_COMMAND_TAU_S);
                gait.step(gait_state, body_state, dt);
                updateOdometry(dt, imuFresh);
                stepGoal(dt);
                kinematics.inverseKinematics(body_state, msgAngles.angles);
                break;
            }
        }
        return true;
    }

    // Control task, after the tick's servo write: the log lines of what the tick took from the inbox, so a slow console
    // never delays the write.
    void logMail() {
        if (_modesTaken.dropped)
            ESP_LOGW("MotionService", "%u modes dropped, the oldest, from a full queue", _modesTaken.dropped);
        for (int i = 0; i < _modesTaken.count; ++i) {
            const ModeMsg &m = _modesTaken.items[i];
            ESP_LOGI("MotionService", "Mode %d%s", (int)m.mode,
                     m.kind == ModeMsgKind::BORROW     ? " (borrow)"
                     : m.kind == ModeMsgKind::HANDBACK ? " (hand-back)"
                                                       : "");
        }
        if (_gaitTaken) ESP_LOGI("MotionService", "Gait %d", (int)*_gaitTaken);
        if (std::exchange(_tunedLogDue, false)) logTunedCommand(_tunedVelocity, target_gait_state);
    }

    float *getAngles() { return msgAngles.angles; }

    // The locomotion command the gait eases toward, the command it walks now, the body pose the motion eases
    // toward, and the body pose and feet the last tick solved the legs for, for the virtual robot's tests.
    const gait_state_t &gaitCommand() const { return target_gait_state; }
    const gait_state_t &gaitState() const { return gait_state; }
    const BodyStateMsg &bodyCommand() const { return target_body_state; }
    const BodyStateMsg &bodyState() const { return body_state; }

    // Any task: the pose of the last tick, whole.
    OdometryEstimate odometry() const { return _inbox.odometry(); }
    // Any task: whether the ESP-NOW handheld drove at the start of the last tick, which the mode's report carries.
    bool handheldDriving() const { return _inbox.handheldDriving(); }
    // Any task. The control task zeroes the pose before its next update; the gyro bias stays. Refused, resetting
    // nothing, while a goal drives: the pose is its origin.
    bool resetOdometry() { return _inbox.requestOdometryReset(); }

    // Adapter task: the reply to a goal, 202 when the control task is to walk it from its next tick. A malformed goal
    // or style is 400, a robot that is neither standing nor walking 409, a style whose speed envelope is empty 422:
    // TUNED, which has none, or a posture none of whose cells walks the goal's axes, and a goal while the handheld
    // drives 423. The style's posture is clamped as the sliders' is and its sticks are the goal's. A goal accepted
    // while another runs replaces it.
    int startGoal(const Goal &goal, const CommandMsg &style, GaitType gait, GoalOwner owner) {
        CommandMsg s = style;
        if (!GoalController::valid(goal) || !s.sanitize() || (int)gait < 0 || (int)gait > (int)GaitType::AUTO)
            return 400;
        // The mode the robot reports: a script waits for the stand it asked for to be reported before it sends a goal.
        const MOTION_STATE mode = _inbox.appliedMode();
        if (mode != MOTION_STATE::STAND && mode != MOTION_STATE::WALK) return 409;
        clampPosture(s.s1, s.h, s.fd);
        if (!goalTops(goal.kind, gait, {s.s1, s.h, s.fd}, s.stride).walkable()) return 422;
        // A full stick walks the gait's top speed, which the goal's speed is a share of.
        s.lx = s.ly = s.rx = s.ry = 0.f;
        s.s = 1.f;
        return _inbox.postGoal(goal, s, gait, owner, millis()) ? 202 : 423;
    }

    // Any task.
    void cancelGoal() { _inbox.cancelGoal(); }

    // Adapter task: a client left. The goal it asked for ends; another client's does not.
    void clientGone(GoalOwner owner) { _inbox.clientGone(owner); }

    // Adapter task: a client sent something. A goal's owner that falls silent for OWNER_SILENCE_MS loses its goal,
    // since a closed USB serial port is not reported while the host still polls the bus.
    void ownerHeard(GoalOwner owner) { _inbox.ownerHeard(owner, millis()); }

    // Any task: the newest goal's status, one accepted but not yet taken as settling, with the pose of the last tick.
    socket_message_GoalStatus goalStatus() const { return _inbox.goalStatus(); }

    // The motion's half of a telemetry tick: what it produced and from which command and mode.
    void fillTelemetry(socket_message_TickSample &tick) const {
        std::copy(std::begin(msgAngles.angles), std::end(msgAngles.angles), tick.angles);
        const CommandMsg &c = lastCommand;
        const float command[9] = {c.lx, c.ly, c.rx, c.ry, c.h, c.s, c.s1, c.fd, c.stride};
        std::copy(std::begin(command), std::end(command), tick.command);
        tick.command_age_ms = lastCommandMillis == 0 ? 0 : millis() - lastCommandMillis;
        tick.link_lost = commandTimedOut;
        tick.mode = static_cast<uint32_t>(motionState);
        tick.gait = static_cast<uint32_t>(gait_state.gait_type);
        const OdometryEstimate odometry = _odometry.estimate();
        tick.odom_x = odometry.x;
        tick.odom_y = odometry.y;
        tick.odom_yaw = odometry.yaw;
        tick.odom_kinematic_yaw = odometry.kinematicYaw;
        tick.odom_distance = odometry.distance;
    }

    // The angles with the servos the controller has driven by no pulse since the boards woke marked limp.
    void publishState() {
        msgAngles.limp = ~_servoController->driven() & ((1u << NUM_SERVO) - 1);
        EventBus<ServoAnglesMsg>::publish(msgAngles);
    }

  private:
    ServoController *_servoController;
    Peripherals *_peripherals;
    EventBus<CommandMsg>::Handle _cmdSubHandle;
    EventBus<ModeMsg>::Handle _modeSubHandle;
    EventBus<GaitMsg>::Handle _gaitSubHandle;
    EventBus<ServoStateMsg>::Handle _servoSwitchSubHandle;
    Kinematics kinematics;
    GaitController gait;
#if FT_ENABLED(USE_POLICY)
    PolicyRunner _policy;
#endif
    AnimationRunner _animation;
    MOTION_STATE _previousMode = MOTION_STATE::STAND;
    bool _borrowedMode = false;
    bool _handbackSent = false;              // a hand-back is queued; cleared by any mode taken
    bool _animating = false;                 // control task only: whether the last tick ran ANIMATE
    bool _idling = false;                    // control task only: whether the last tick ran IDLE
    bool _idleHeld = false; // control task only: IDLE holds servos at a pose no motion set, written there or ANIMATE's
    bool _enterFromHeld = false; // control task only: ANIMATE is entered from such an IDLE
    bool _handheldReported = false;          // control task only: whether the handheld drove, as last reported

    BodyStateMsg body_state = {0, 0, 0, 0, 0, 0};
    BodyStateMsg target_body_state = {0, 0, 0, 0, 0, 0};
    static constexpr float DEFAULT_STEP_HEIGHT_MM = 15.f;
    static constexpr float DEFAULT_STEP_DEPTH = 0.002f;
    gait_state_t gait_state = {
        DEFAULT_STEP_HEIGHT_MM,  0, 0, 0, 1, DEFAULT_STEP_DEPTH, default_stand_frac, GaitType::TRI_GATE,
        {0, 0.5, 0, 0.5, 0, 0.5}};
    // Commands land here; gait_state eases toward it every tick (issue #7).
    gait_state_t target_gait_state = gait_state;
    GaitCommandRamp _commandRamp; // control task
    FusedOdometry _odometry;      // control task

    // What the other tasks post for the control task to take. A goal is asked for on an adapter task and walked by the
    // control task; whoever ends it early, on any task, asks the control task to through the inbox.
    MotionInbox _inbox;
    // Control task: the goal taken, and how far it got.
    MotionInbox::GoalRequest _goalTaken {};
    GoalController _goal;
    GoalState _goalState = GoalState::IDLE;
    bool _goalStyleTaken = false; // its posture and gait were handed to the walk
    bool _goalStarted = false;    // _goal walks it
    unsigned long _goalTakenMs = 0;
    socket_message_GoalStatus _goalReported = socket_message_GoalStatus_init_zero;
    static constexpr int GOAL_STATUS_STEPS = 20; // a push every 5 % of progress
    // A goal takes its posture and gait in a few seconds; one still waiting after this never will.
    static constexpr unsigned long GOAL_START_TIMEOUT_MS = 10000;
    // The period a stop is stepped ahead at to predict it, four control ticks, which predicts as well as one on the
    // virtual robot's goal matrix at a quarter of the cost, and the longest stop it follows.
    static constexpr float STOP_PREDICTION_DT_S = 0.02f;
    static constexpr int STOP_PREDICTION_STEPS = 100;
    // A foot that lifts in the stop's tail still sets down off its stance, so the stop is taken to step a little
    // further than its ramp. On the virtual robot's TRI and BI goals, 1.05 to 1.15 reach a metre at full speed in one
    // pass and every centimetre and few-degree goal at half speed; 1.0 needs a second pass at full speed, 1.2 misses
    // 5 deg turns, and above 1.05 a TRI metre's predicted progress steps back more often.
    static constexpr float STOP_TAIL = 1.05f;

    static constexpr float smoothing_factor = BODY_SMOOTHING;
    // Ramp time constant for the locomotion command: ~0.45 s to settle. Slow enough that a slammed
    // stick does not jerk the stride, fast enough that the robot still feels directly driven.
    static constexpr float GAIT_COMMAND_TAU_S = 0.15f;
    static_assert(GoalController::RAMP_TAU_S == GAIT_COMMAND_TAU_S, "a goal predicts its stop from this ramp");
    static constexpr unsigned long COMMAND_TIMEOUT_MS = 2000;
    // A walking robot covers ~0.9 m in COMMAND_TIMEOUT_MS. While a stick drives locomotion the
    // senders repeat it (app heartbeat 200 ms, ESP-NOW controller 50 Hz), so silence for three
    // heartbeats means the link is gone and the robot must stop walking.
    static constexpr unsigned long STEERING_TIMEOUT_MS = 600;
    static_assert(MotionInbox::HANDHELD_PRIORITY_MS == STEERING_TIMEOUT_MS,
                  "the handheld drives for as long as the walk obeys its last stick");
    static_assert(MotionInbox::CLIENT_IN_FORCE_MS == COMMAND_TIMEOUT_MS,
                  "a client's command is in force for as long as the dead-man obeys any command");

    float default_feet_pos[6][4];

    GaitType _requestedGait = GaitType::TRI_GATE;
    // The last WALK stick, posture clamped, its share of BI's top speed and the gait AUTO picks for it; all are
    // cleared when the locomotion command stops.
    CommandMsg lastWalkCommand_ {};
    float _walkSpeed01 = 0.f;
    // The stride the classic walk walks, slewed toward the commanded one (slewStride).
    float _stride = 0.f;
    static constexpr float STRIDE_SLEW_PER_S = 1.0f;
    GaitType _autoGait = GaitType::RIPPLE;
    struct Posture {
        float s1, h, fd;
    };
    // The posture the classic walk stands in, valid once it was taken between steps (settleWalkPosture) or from the
    // stand the walk was entered from (takeRestPosture).
    Posture _walkPosture {};
    bool _walkPostureValid = false;
    // The posture of a stand at rest, its lift as an s1, for a walk entered next: the targets the stand holds. Anything
    // that moves the robot in WALK before the first classic command takes it consumes it (clears _atRest).
    Posture _restPosture {};
    bool _atRest = false;
    // A posture is waiting for the gait to stop, then for the robot to settle; the walk is held at zero meanwhile.
    bool _posturePending = false;
    bool _postureSettling = false;
    // The stand posture last commanded, waiting until the feet are still (takeStandPosture).
    Posture _standPosture {};
    bool _standPosturePending = false;
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
    // The modes and the gait the tick took, and the velocity the last TUNED command asked for, logged after its servo
    // write.
    MotionInbox::Modes _modesTaken;
    std::optional<GaitType> _gaitTaken;
    float _tunedVelocity[3] = {0.f, 0.f, 0.f};
    bool _tunedLogDue = false;
    // The dead-man stop is in force: it fired, and no command or goal has come since.
    bool commandTimedOut = false;
    bool steering = false;
    // The last command taken, for the telemetry.
    CommandMsg lastCommand{};

    ServoAnglesMsg msgAngles = {.angles = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, .limp = 0};

  public:
    // Any task: the mode the control task applied last, the one the robot reports.
    MOTION_STATE appliedMode() const { return _inbox.appliedMode(); }

    // Adapter task. Lying on its belly with the legs limp or folded, in the mode the control task applied last: still
    // enough to calibrate the IMU against.
    bool resting() const {
        const MOTION_STATE mode = _inbox.appliedMode();
        return mode == MOTION_STATE::DEACTIVATED || mode == MOTION_STATE::IDLE;
    }

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
        // Held, the stick still tells AUTO how fast the walk will be, so the gait is chosen while the walk waits.
        const float stride = _stride;
        const AutoDemand demand = autoDemand(c.lx, c.ly, c.rx, c.s, p.s1, p.h, p.fd, stride);
        _walkSpeed01 = demand.speed01;
        _autoGait = selectAutoGait(demand, gait_state.gait_type, p.s1, p.h, p.fd, stride);
        const WalkTarget w =
            _requestedGait == GaitType::AUTO
                ? autoWalkTarget(gait_state.gait_type, demand, c.lx, c.ly, c.rx, p.s1, p.h, p.fd, stride)
                : classicWalkTarget(gait_state.gait_type, c.lx, c.ly, c.rx, c.s, p.s1, p.h, p.fd, stride);
        target_gait_state.step_x = held ? 0.f : w.stepX;
        target_gait_state.step_y = held ? 0.f : w.stepY;
        target_gait_state.step_angle = held ? 0.f : w.stepAngle;
        target_gait_state.phase_rate = held ? 0.f : w.phaseRate;
    }

    // Control task, IDLE, after the tick's modes and the centring before them: the joints dragged while the robot
    // idled. A joint writes its own servo alone, so every other servo holds the pulse it had when the motion stopped, or
    // stays limp; a joint posted before a mode leaving IDLE was applied never gets here. The angles the servos hold, within travel as
    // the controller held the motion's last pose, are published on entering IDLE and after each tick that writes, as the
    // app's 3D view draws them, and never on a timer.
    void driveIdleServos(const MotionInbox::Mail &mail, bool entering) {
        if (entering)
            for (int servo = 0; servo < NUM_SERVO; ++servo)
                msgAngles.angles[servo] = withinTravel(msgAngles.angles[servo], servo % 3);
        bool written = false;
        for (int servo = 0; servo < NUM_SERVO; ++servo) {
            if (!(mail.joints.servos & (1u << servo))) continue;
            _servoController->setJointAngle(servo, mail.joints.degrees[servo]);
            msgAngles.angles[servo] = withinTravel(mail.joints.degrees[servo], servo % 3);
            written = true;
        }
        if (written) holdIdle();
        if (entering || written) publishState();
    }

    // Control task, IDLE, before the tick's modes: every servo to its centre pulse, published as the centre pose.
    void centreServos() {
        _servoController->setCenterPwm();
        for (int servo = 0; servo < NUM_SERVO; ++servo) msgAngles.angles[servo] = _servoController->centreAngle(servo);
        holdIdle();
        publishState();
    }

    // Control task, before the tick's modes: the servo page's switch, in any mode. The boards sleep, or wake with every
    // channel off, and the angles are published with the servos that leaves limp.
    void switchServos(bool on) {
        on ? _servoController->activate() : _servoController->deactivate();
        publishState();
    }

    // Control task, IDLE: the servos hold a pose IDLE wrote, so a mode leaving it eases out of it.
    void holdIdle() {
        _idleHeld = true;
        noteModeState();
    }

    // Control task: the arbiter's state, for the decisions taken on arrival (MotionInbox::postMode).
    void noteModeState() { _inbox.noteMode({motionState, _borrowedMode, _previousMode, _idleHeld}); }

    // Control task, WALK. A stride change is a change of the walk, not of its posture, so it never pauses the walk; but
    // the command ramp alone eases a stroke change fast enough to move a joint up to 5.5 deg in a tick, past the
    // servo's 3.75. Slewed at STRIDE_SLEW_PER_S, the walk passes only through strides it could walk steadily, and peaks
    // as a steady walk does. A gait at rest takes the stride at once, since no foot moves with it.
    void slewStride(float dt) {
        const float commanded = lastWalkCommand_.stride;
        const float current = _stride;
        if (current == commanded) return;
        const float step = STRIDE_SLEW_PER_S * dt;
        _stride = gaitIsCommanded(gait_state) ? std::clamp(commanded, current - step, current + step) : commanded;
        applyWalkTarget();
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
            // Settling holds the walk at zero throughout (applyWalkTarget).
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

    struct GoalTops {
        float speed, turn; // mm/s and rad/s: what a full stick walks at the posture
        bool walkable() const { return speed > 0.f && turn > 0.f; }
    };

    // AUTO's full stick walks BI's top speed (autoDemand); a sideways goal's stick is the lateral one. TUNED has no
    // envelope.
    static GoalTops goalTops(GoalKind kind, GaitType gait, const Posture &p, float stride) {
        const GaitType walked = gait == GaitType::AUTO ? GaitType::BI_GATE : gait;
        const int axis = kind == GoalKind::SIDEWAYS ? 1 : 0;
        return {envelopeAt(walked, axis, p.s1, p.h, p.fd, stride).vMax,
                envelopeAt(walked, 2, p.s1, p.h, p.fd, stride).vMax};
    }

    static void setPose(socket_message_GoalStatus &s, const OdometryEstimate &pose) {
        s.x = pose.x;
        s.y = pose.y;
        s.yaw = pose.yaw;
    }

    bool goalActive() const { return _goalState == GoalState::SETTLING || _goalState == GoalState::RUNNING; }

    // Control task, every tick: takes a goal accepted since the last, and ends one whose end was asked for. A goal
    // ended by anything but an operator's stick leaves the walk centred, so it stops through the stop ramp.
    void takeGoalMail(const MotionInbox::Mail &mail) {
        if (mail.goalDriveEnded) centreWalk();
        const bool cancelled = mail.goalCancelled;
        if (mail.goal) {
            cancelGoalTaken();
            _goalTaken = *mail.goal;
            _goalStyleTaken = _goalStarted = false;
            _goalTakenMs = millis();
            const MOTION_STATE mode = motionState;
            // The mode can have changed since the reply, on another task.
            _goalState = cancelled                                                   ? GoalState::CANCELLED
                         : mode == MOTION_STATE::STAND || mode == MOTION_STATE::WALK ? GoalState::SETTLING
                                                                                     : GoalState::REFUSED;
            if (_goalState == GoalState::SETTLING && mode == MOTION_STATE::STAND)
                EventBus<ModeMsg>::publish({MOTION_STATE::WALK});
            // The goal commands the walk from here on, so a dead-man stop is over, and the stop watches the link again
            // once the goal ends.
            if (goalActive()) commandTimedOut = false;
            if (!goalActive()) releaseGoalWalk();
            reportGoal();
        } else if (cancelled) {
            cancelGoalTaken();
        } else if (goalActive() && !_goalStarted && millis() - _goalTakenMs > GOAL_START_TIMEOUT_MS) {
            _goalState = GoalState::TIMED_OUT;
            releaseGoalWalk();
            reportGoal();
        }
    }

    // Control task: ends the goal in progress, if any, cancelled; whoever ended it has the walk's sticks.
    void cancelGoalTaken() {
        if (!goalActive()) return;
        if (_goalStarted) _goal.cancel();
        _goalState = GoalState::CANCELLED;
        reportGoal();
    }

    // Control task: the goal no longer drives the walk; its last sticks are centred unless an operator took them.
    void releaseGoalWalk() {
        if (_inbox.endGoalDrive()) centreWalk();
    }

    // Control task.
    void centreWalk() {
        lastWalkCommand_.lx = lastWalkCommand_.ly = lastWalkCommand_.rx = 0.f;
        applyWalkTarget();
    }

    // Control task, WALK, after the odometry. The goal hands its style to the walk, starts once the walk has taken its
    // posture and gait, and then walks its sticks, judging rest as the stop ramp leaves it: no stroke and the feet
    // home.
    void stepGoal(float dt) {
        if (!goalActive()) return;
        if (!_goalStyleTaken) {
            takeGoalStyle();
            return;
        }
        if (!_goalStarted) {
            if (_posturePending || _postureSettling || !_walkPostureValid) return;
            if (_goalTaken.gait != GaitType::AUTO && gait_state.gait_type != _goalTaken.gait) return;
            const GoalTops tops =
                goalTops(_goalTaken.goal.kind, _goalTaken.gait, _walkPosture, _goalTaken.style.stride);
            _goal.start(_goalTaken.goal, restPose(), tops.speed, tops.turn);
            _goalStarted = true;
        } else {
            const GoalSticks sticks = _goal.update(restPose(), !gaitIsCommanded(gait_state) && feetAtRest(), dt);
            if (_inbox.goalDrives()) {
                lastWalkCommand_.lx = sticks.lx;
                lastWalkCommand_.ly = sticks.ly;
                lastWalkCommand_.rx = sticks.rx;
                applyWalkTarget();
            }
        }
        _goalState = _goal.state();
        if (!goalActive()) releaseGoalWalk();
        reportGoal();
    }

    // Control task, the first WALK tick of a goal: its posture goes through the walk's posture paths (a walk from a
    // stand at rest in it starts at once, any other settles first) and its gait through the deferred swap. TUNED steps
    // on the stick it last had, which only a stop ends.
    void takeGoalStyle() {
        if (!_inbox.goalDrives()) return;
        if (gait_state.gait_type == GaitType::TUNED) stopLocomotionCommand(false);
        const bool newGait = _requestedGait != _goalTaken.gait;
        _requestedGait = _goalTaken.gait;
        _inbox.selectGait(_goalTaken.gait);
        lastWalkCommand_ = _goalTaken.style;
        walkClassic(_goalTaken.style);
        _goalStyleTaken = true;
        // For the app and the controller; MotionService would take its own selection for a wire gait change.
        if (newGait) EventBus<GaitMsg>::publish({_goalTaken.gait}, _gaitSubHandle);
    }

    // Control task: pushes the status on a change of goal or state, or a step of progress.
    void reportGoal() {
        socket_message_GoalStatus s =
            goalStatusOf(_goalTaken.id, _goalState, _goalTaken.goal, _goalStarted ? _goal.progress() : 0.f,
                         _goalStarted ? _goal.passes() : 0);
        const auto step = [](const socket_message_GoalStatus &status) {
            return static_cast<int>(std::floor(status.progress * GOAL_STATUS_STEPS));
        };
        if (s.id == _goalReported.id && s.state == _goalReported.state && step(s) == step(_goalReported)) return;
        _goalReported = s;
        setPose(s, _odometry.estimate());
        _inbox.report(s);
        GoalStatusBus::publish(s);
    }

    // Control task. The envelope measured a level, centred body: the tilt stick and the stand's
    // offsets ease out. The envelope sets the cadence; step_speed is left to pace a stance change.
    void walkClassic(const CommandMsg &c) {
        target_body_state.xm = target_body_state.ym = 0;
        target_body_state.phi = target_body_state.omega = 0;
        target_gait_state.step_depth = 0.002f;
        target_gait_state.step_speed = 1.f;
        takeRestPosture(c);
        applyWalkTarget();
    }

    // Control task, after the gait stepped: the feet it left, the legs it has in stance and the DMP's yaw read on this
    // tick.
    void updateOdometry(float dt, bool imuFresh) {
        std::optional<ImuYaw> packet;
        if (imuFresh) packet = ImuYaw {imu_frame::YAW_SIGN * _peripherals->angleZ(), _peripherals->imuFifoResets()};
        _odometry.update(body_state, stanceFlags(), packet, dt);
        _inbox.noteOdometry(_odometry.estimate());
    }

    // Control task: which legs the gait has in stance after its last step.
    std::array<bool, 6> stanceFlags() const {
        std::array<bool, 6> stance;
        for (int leg = 0; leg < 6; ++leg) stance[leg] = gait.inStance(gait_state, leg);
        return stance;
    }

    // Control task: where the robot will stand once a stop's ease home has set its planted feet on their places. A goal
    // judges this pose rather than the odometry's: feet that only slid within their stance carry the body back as they
    // ease home, so a walk too short for any foot to swing has gone nowhere. A planted foot the stop still lifts before
    // the gait comes to rest steps on instead, as the coast the goal predicts.
    OdometryEstimate restPose() const {
        const float phase = stopPhase();
        std::array<bool, 6> stays;
        for (int leg = 0; leg < 6; ++leg) stays[leg] = gait.phaseToLift(gait_state, leg) > phase;
        return KinematicOdometry::advance(
            _odometry.estimate(), KinematicOdometry::restStep(body_state, stanceFlags(), default_feet_pos, stays));
    }

    // Control task: the phase a stop from now still steps, from the stop ramp run ahead on a copy of it and of the
    // gait, so a stop already under way goes on from where it is. Its cadence follows the stroke left, and the gait
    // stops cycling once the stroke is under its deadband, so a stop from a stroke near the deadband steps almost none.
    float stopPhase() const {
        GaitCommandRamp ramp = _commandRamp;
        gait_state_t live = gait_state, stopped = target_gait_state;
        stopped.step_x = stopped.step_y = stopped.step_angle = stopped.phase_rate = 0;
        const float a = easeShare(STOP_PREDICTION_DT_S, GAIT_COMMAND_TAU_S);
        float phase = 0;
        for (int t = 0; t < STOP_PREDICTION_STEPS; ++t) {
            ramp.stepBy(live, stopped, a);
            if (!gaitIsCommanded(live)) break;
            phase += live.phase_rate * STOP_PREDICTION_DT_S;
        }
        return phase * STOP_TAIL;
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

    // Control task, the first classic walk command after a stand at rest. A command that keeps the stand's height
    // and stance walks at once in it: only the lift differs, and a lift that rises eases in alongside the stroke,
    // which keeps every state between inside the envelope (checked over its walkable grid). One that falls does not:
    // the stand's lift over the posture's stroke can leave travel, so that walk, like any other, settles first.
    void takeRestPosture(const CommandMsg &c) {
        if (_walkPostureValid || _postureSettling || !std::exchange(_atRest, false)) return;
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
        if (!_standPosturePending || gaitIsCommanded(gait_state) || !feetAtRest() || !liftArrived()) return;
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

    // The next classic walk takes its posture again, unless it starts from a stand at rest in it (takeRestPosture).
    void forgetWalkPosture() {
        _walkPostureValid = false;
        _posturePending = false;
        _postureSettling = false;
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
        target_gait_state.step_depth = DEFAULT_STEP_DEPTH;
        target_gait_state.phase_rate = 0.f;
        // Else a gait loaded later, AUTO's included, would walk the stick the dead-man stopped.
        lastWalkCommand_ = {};
        _walkSpeed01 = 0.f;
        _autoGait = GaitType::RIPPLE;
        forgetWalkPosture();
        // A stand posture waits only within the command stream it came in; the dead-man and a new mode end that.
        _standPosturePending = false;
        if (!snap) return;
        approachGaitCommand(gait_state, target_gait_state, 0.f, 0.f);
        gait.setPhase(0.f);
    }

    // The dead-man: the stick stops and the posture goes to neutral. A body height changed under stepping feet steps
    // through postures the legs cannot hold, so a classic walk settles into the neutral posture once it has stopped, as
    // into any other (settleWalkPosture), and a stand takes it once its feet are still (takeStandPosture). TUNED,
    // WALK_NN and the modes that drive no gait take the height at once.
    void applyZeroCommand() {
        target_body_state.xm = 0;
        target_body_state.ym = 0;
        target_body_state.phi = 0;
        target_body_state.omega = 0;
        if (motionState == MOTION_STATE::WALK && gait_state.gait_type != GaitType::TUNED) {
            lastWalkCommand_ = {};
            applyWalkTarget();
            return;
        }
        stopLocomotionCommand(false);
        if (motionState == MOTION_STATE::STAND) {
            _standPosture = {};
            _standPosturePending = true;
            return;
        }
        target_body_state.zm = 0;
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
        // zeroing it would drop the body mid-play. A goal is the robot's own walk and ends on its own terms. The
        // window restarts instead, so the app has a full window after a hand-back or a goal to resume its heartbeat.
        if (motionState == MOTION_STATE::ANIMATE || goalActive()) {
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
