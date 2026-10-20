#pragma once

// What the other tasks hand MotionService's control task, under one mutex: the modes in arrival order, the newest
// command and gait, a goal asked for and its end, an odometry reset, and in IDLE the joints dragged and a centring of
// the servos; and back from the control task, the goal
// status, the mode and the pose the replies carry. The decisions a reply or a goal depends on are made as each arrives,
// in arrival order, against the mode the control task applied last; the control task takes what arrived at the start
// of its next tick, so nothing changes the motion in the middle of one, and no task ever waits for the control task.
// Ported in shape from SpotMicroESP32-Leika's esp32/include/motion_inbox.h.

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <mutex>
#include <optional>
#include <utility>
#include <animation/mode_arbiter.h>
#include <goal_controller.h>
#include <odometry.h>
#include <message_types.h>
#include <platform_shared/message.pb.h>

/** A transport's client, its adapter and its id there: ids repeat across transports. */
struct GoalOwner {
    const void *adapter = nullptr;
    int id = -1;
    bool operator==(const GoalOwner &other) const { return adapter == other.adapter && id == other.id; }
};

static_assert((int)GoalState::IDLE == socket_message_GoalState_GOAL_IDLE &&
                  (int)GoalState::SETTLING == socket_message_GoalState_GOAL_SETTLING &&
                  (int)GoalState::RUNNING == socket_message_GoalState_GOAL_RUNNING &&
                  (int)GoalState::REACHED == socket_message_GoalState_GOAL_REACHED &&
                  (int)GoalState::CANCELLED == socket_message_GoalState_GOAL_CANCELLED &&
                  (int)GoalState::TIMED_OUT == socket_message_GoalState_GOAL_TIMED_OUT &&
                  (int)GoalState::REFUSED == socket_message_GoalState_GOAL_REFUSED,
              "GoalStatus.state carries GoalState by value");
static_assert((int)GoalKind::FORWARD == socket_message_GoalKind_GOAL_FORWARD &&
                  (int)GoalKind::SIDEWAYS == socket_message_GoalKind_GOAL_SIDEWAYS &&
                  (int)GoalKind::TURN == socket_message_GoalKind_GOAL_TURN &&
                  (int)GoalKind::ARC == socket_message_GoalKind_GOAL_ARC,
              "GoalStatus.kind and GoalMove.kind carry GoalKind by value");

inline socket_message_GoalStatus goalStatusOf(uint32_t id, GoalState state, const Goal &goal, float progress,
                                              int passes) {
    socket_message_GoalStatus s = socket_message_GoalStatus_init_zero;
    s.id = id;
    s.state = static_cast<socket_message_GoalState>(state);
    s.kind = static_cast<socket_message_GoalKind>(goal.kind);
    s.target = goal.kind == GoalKind::FORWARD || goal.kind == GoalKind::SIDEWAYS ? goal.distance : goal.angle;
    s.progress = progress;
    s.passes = passes;
    return s;
}

class MotionInbox {
  public:
    struct GoalRequest {
        Goal goal;
        CommandMsg style;
        GaitType gait;
        uint32_t id;
    };

    // The arbiter's state (animation/mode_arbiter.h): the mode, whether ANIMATE is borrowed, the mode a hand-back
    // returns to and whether IDLE holds servos at a pose no motion set. It starts as MotionService does.
    struct ModeState {
        MOTION_STATE mode = MOTION_STATE::DEACTIVATED;
        bool borrowed = false;
        MOTION_STATE previous = MOTION_STATE::STAND;
        bool idleHeld = false;
    };

    // Eight modes within one tick, or within an overrun of 20 ms per timed-out I2C transfer, against the ModeMsg bus
    // queue's ten upstream.
    static constexpr int MODE_DEPTH = 8;

    struct Modes {
        ModeMsg items[MODE_DEPTH]; // in arrival order
        int count = 0;
        unsigned dropped = 0; // the oldest, when a full queue took a newer one
    };

    // The newest angle of each joint dragged since the last take, in degrees; bit i of `servos` marks servo i's.
    struct Joints {
        float degrees[NUM_SERVO] = {};
        uint32_t servos = 0;
    };
    static_assert(NUM_SERVO <= 32, "a servo's mark is one bit of Joints::servos");

    struct Mail {
        Modes modes;
        std::optional<CommandMsg> command; // the newest since the last take
        unsigned long commandMs = 0;       // when it arrived
        std::optional<GaitType> gait;      // the newest since the last take
        std::optional<GoalRequest> goal;   // accepted since the last take
        bool goalCancelled = false;  // the end of the goal driving the walk was asked for, or its owner fell silent
        bool goalDriveEnded = false; // so a goal no longer drives the walk, whose sticks are to be centred
        bool odometryReset = false;
        bool handheldDriving = false; // the handheld has moved a stick within HANDHELD_PRIORITY_MS
        Joints joints;                // dragged in IDLE
        bool centreServos = false;    // every servo to its centre pulse, asked for in IDLE
    };

    // The facade and the app send every 0.2 s; an owner silent ten times as long has gone, whether or not its transport
    // has said so. A goal with no transport, the host's own, has no owner to fall silent.
    static constexpr unsigned long OWNER_SILENCE_MS = 2000;

    // The ESP-NOW handheld drives for this long after its last frame with a stick off centre: as long as the walk obeys
    // its last stick (MotionService's STEERING_TIMEOUT_MS), so a handheld whose link drops mid-stride is stopped by the
    // dead-man at the moment the other sources are heard again.
    static constexpr unsigned long HANDHELD_PRIORITY_MS = 600;

    // A client's command is in force, and an idle handheld's frame dropped, for this long after its arrival: the
    // dead-man's longest window (MotionService's COMMAND_TIMEOUT_MS), so a client's posture outlasts every stall the
    // dead-man forgives. A dropped frame is no command to the dead-man, so a lost client's stick still stops at the
    // steering timeout.
    static constexpr unsigned long CLIENT_IN_FORCE_MS = 2000;

    // Adapter task: a goal accepted, replacing one not yet taken. It drives the walk's sticks from now until it ends or
    // an operator takes the walk back. Refused, false, while the handheld drives.
    bool postGoal(const Goal &goal, const CommandMsg &style, GaitType gait, GoalOwner owner, unsigned long nowMs) {
        std::lock_guard<std::mutex> lock(_mutex);
        if (handheldDrivingLocked(nowMs)) return false;
        _goal = GoalRequest {goal, style, gait, ++_goalCount};
        _goalOwner = owner;
        _ownerHeardMs = nowMs;
        _goalDrives = true;
        // Whatever ended the goal before is done with: a cancel from now on is this goal's.
        _goalCancelRequested = false;
        return true;
    }

    // Any task: ends the goal driving the walk, if any.
    void cancelGoal() {
        std::lock_guard<std::mutex> lock(_mutex);
        cancelGoalLocked();
    }

    // Adapter task: a client left. The goal it asked for ends; another client's does not.
    void clientGone(GoalOwner owner) {
        std::lock_guard<std::mutex> lock(_mutex);
        if (owner == _goalOwner) cancelGoalLocked();
    }

    // Adapter task: a client sent something, which keeps the goal it owns.
    void ownerHeard(GoalOwner owner, unsigned long nowMs) {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_goalDrives && owner == _goalOwner) _ownerHeardMs = nowMs;
    }

    // Command worker. Each command is a full snapshot of the sticks and sliders, so the newest replaces one not yet
    // taken. While the handheld drives, a client's command is dropped, centred or not: the app's heartbeat would stop
    // the handheld's walk for a tick and move its posture. The handheld's own centred frame is its stop, and is walked.
    // After the window the robot stays stopped: a client's stick off centre is dropped until a client has sent a
    // centred command, so neither a script's keep-alive nor a stick the app held through the drive walks the robot
    // again by itself while that client is the only one. Commands carry no client's identity, so one centred command
    // ends the wait for every client, as the app and the scripts are last-writer between themselves anyway. A command
    // dropped is no command to the dead-man either. Outside the window the handheld's centred frames, which it sends
    // whenever it is switched on, are dropped whole while a client's command is in force, one the handheld left
    // through, a goal's swallowed heartbeat included, so an idle handheld neither stops that client's walk nor moves
    // its posture; with no client in force they are the handheld's commands, so a handheld on its own holds its trim
    // and the dead-man watches it (roadmap 11.1b). While a goal drives the walk, a centred command is no command, its
    // posture and the rest of it too: the heartbeats of the app, the facade and the ESP-NOW controller. A stick off
    // centre, or the handheld moving any of its own, takes the walk back, so the goal ends and never writes the sticks
    // again, and the command is walked.
    void postCommand(const CommandMsg &c, unsigned long nowMs) {
        std::lock_guard<std::mutex> lock(_mutex);
        const bool handheldMoving = c.origin == CommandOrigin::HANDHELD_MOVING;
        const bool steers = c.lx != 0.f || c.ly != 0.f || c.rx != 0.f;
        if (handheldMoving) {
            _handheldMovedMs = nowMs;
            _handheldMoved = true;
            _clientsWaitForCentre = true;
        } else if (c.origin == CommandOrigin::CLIENT) {
            if (handheldDrivingLocked(nowMs)) return;
            if (_clientsWaitForCentre) {
                if (steers) return;
                _clientsWaitForCentre = false;
            }
            _clientHeardMs = nowMs;
            _clientHeard = true;
        } else if (!handheldDrivingLocked(nowMs) && clientInForceLocked(nowMs)) {
            return;
        }
        if (_goalDrives) {
            if (!handheldMoving && !steers) return;
            _goalDrives = false;
            _goalCancelRequested = true;
        }
        _command = c;
        _commandMs = nowMs;
    }

    // Mode worker, and the control task for its own hand-back, which so queues behind every mode that arrived before
    // it. Modes are never merged, since a borrow and a hand-back are decided against the mode they find. One that the
    // arbiter, from the mode applied last and with the runner's `busy` now, has apply a mode other than WALK ends the
    // goal driving the walk on arrival, as a goal walks in WALK. An APPLIED is no request. Full, the queue drops its
    // oldest, so the newest intent, a DEACTIVATED above all, is kept.
    void postMode(const ModeMsg &m, bool busy) {
        if (m.kind == ModeMsgKind::APPLIED) return;
        std::lock_guard<std::mutex> lock(_mutex);
        if (_modes.count == MODE_DEPTH) {
            std::move(std::begin(_modes.items) + 1, std::end(_modes.items), std::begin(_modes.items));
            --_modes.count;
            ++_modes.dropped;
        }
        _modes.items[_modes.count++] = m;
        const ModeArbiterDecision d =
            decideMode({_applied.mode, _applied.borrowed, _applied.previous, m.mode, m.kind, busy, _applied.idleHeld});
        if (d.action == ModeArbiterDecision::APPLY && d.mode != MOTION_STATE::WALK) cancelGoalLocked();
    }

    // Adapter task: a servo's joint angle, dragged on the app's 3D model. A joint moves the robot only in IDLE, so it
    // is dropped on arrival unless the mode applied last is IDLE, and the control task drops it again unless the modes
    // it takes before it leave the robot in IDLE. A drag is a stream of positions, so the newest per servo replaces one
    // not yet taken.
    void postJoint(int servo, float degrees) {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_applied.mode != MOTION_STATE::IDLE) return;
        _joints.degrees[servo] = degrees;
        _joints.servos |= 1u << servo;
    }

    // Adapter task: every servo to its centre pulse, as the calibration page asks. False, nothing asked, unless the mode
    // applied last is IDLE and no mode is queued, since a queued mode is applied first and may leave IDLE; so a
    // centring taken comes before every mode taken with it. It moves every servo, so it replaces the joints posted
    // before it, and the joints posted after it are written after it.
    bool postCentreServos() {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_applied.mode != MOTION_STATE::IDLE || _modes.count > 0) return false;
        _centreServos = true;
        _joints = {};
        return true;
    }

    // Any task: the mode the control task applied last, which is the mode the robot reports; a mode still queued is not
    // the robot's yet.
    MOTION_STATE appliedMode() const {
        std::lock_guard<std::mutex> lock(_mutex);
        return _applied.mode;
    }

    // Any task: whether the animation calls are to be honoured, the mode being ANIMATE or a play's borrow of it queued
    // and so to be applied at the next take. The play such a borrow follows is already loaded, so a stop, a pose or a
    // lost control that arrives before the borrow is applied still reaches it.
    bool animating() const {
        std::lock_guard<std::mutex> lock(_mutex);
        const ModeMsg *end = _modes.items + _modes.count;
        return _applied.mode == MOTION_STATE::ANIMATE ||
               std::any_of(_modes.items, end, [](const ModeMsg &m) { return m.kind == ModeMsgKind::BORROW; });
    }

    // Control task, as it applies or keeps each mode it took, before it reports it, and as IDLE writes a servo.
    void noteMode(const ModeState &applied) {
        std::lock_guard<std::mutex> lock(_mutex);
        _applied = applied;
    }

    // Gait worker. A goal's timeout was reckoned at its gait's pace, so a gait other than the one selected ends it, on
    // arrival: a newer gait replacing this one before the take ends nothing more.
    void postGait(GaitType gait) {
        std::lock_guard<std::mutex> lock(_mutex);
        if (gait != _selection) cancelGoalLocked();
        _selection = gait;
        _gait = gait;
    }

    // Control task: a goal's own gait becomes the selection, which ends nothing; its publish is not delivered back. A
    // gait posted since the take is the one the next take puts in force, so it stays selected, and the goal, which
    // would wait for its own gait, ends unless that is the gait posted.
    void selectGait(GaitType gait) {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_gait)
            _selection = gait;
        else if (*_gait != gait)
            cancelGoalLocked();
    }

    // Any task: the control task zeroes the pose before its next update. Refused while a goal drives: the pose is its
    // origin.
    bool requestOdometryReset() {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_goalDrives) return false;
        _odometryReset = true;
        return true;
    }

    // Any task: the newest goal's status, one accepted but not yet taken as settling, with the pose of the last tick.
    socket_message_GoalStatus goalStatus() const {
        std::lock_guard<std::mutex> lock(_mutex);
        socket_message_GoalStatus s =
            _goal ? goalStatusOf(_goal->id, GoalState::SETTLING, _goal->goal, 0.f, 0) : _goalStatus;
        s.x = _odometry.x;
        s.y = _odometry.y;
        s.yaw = _odometry.yaw;
        return s;
    }

    // Any task: the pose of the last tick, whole.
    OdometryEstimate odometry() const {
        std::lock_guard<std::mutex> lock(_mutex);
        return _odometry;
    }

    // Control task: the pose the odometry moved to.
    void noteOdometry(const OdometryEstimate &pose) {
        std::lock_guard<std::mutex> lock(_mutex);
        _odometry = pose;
    }

    // Control task, at the start of every tick: what arrived since the last take. A goal whose end was asked for no
    // longer drives the walk from here on. Its clock is read before the lock, so an owner heard in between is heard
    // after it: the silence is a signed difference.
    Mail take(unsigned long nowMs) {
        std::lock_guard<std::mutex> lock(_mutex);
        const long ownerSilenceMs = static_cast<long>(nowMs - _ownerHeardMs);
        Mail mail;
        mail.modes = std::exchange(_modes, Modes {});
        mail.command = std::exchange(_command, std::nullopt);
        mail.commandMs = _commandMs;
        mail.gait = std::exchange(_gait, std::nullopt);
        mail.goal = std::exchange(_goal, std::nullopt);
        mail.goalCancelled =
            std::exchange(_goalCancelRequested, false) ||
            (_goalDrives && _goalOwner.adapter && ownerSilenceMs > static_cast<long>(OWNER_SILENCE_MS));
        mail.goalDriveEnded = mail.goalCancelled && std::exchange(_goalDrives, false);
        mail.odometryReset = std::exchange(_odometryReset, false);
        mail.joints = std::exchange(_joints, Joints {});
        mail.centreServos = std::exchange(_centreServos, false);
        mail.handheldDriving = _handheldDriving = handheldDrivingLocked(nowMs);
        // A lapsed window is forgotten, or the signed difference would reopen it half the clock's range later.
        if (!_handheldDriving) _handheldMoved = false;
        if (!clientInForceLocked(nowMs)) _clientHeard = false;
        return mail;
    }

    // Any task: whether the handheld drove at the last take, as the robot reports it.
    bool handheldDriving() const {
        std::lock_guard<std::mutex> lock(_mutex);
        return _handheldDriving;
    }

    // Control task: the goal no longer drives the walk. False when it did not, since an operator took the walk back.
    bool endGoalDrive() {
        std::lock_guard<std::mutex> lock(_mutex);
        return std::exchange(_goalDrives, false);
    }

    // Whether a goal drives the walk's sticks. A command posted after the read takes them back from the next tick.
    bool goalDrives() const {
        std::lock_guard<std::mutex> lock(_mutex);
        return _goalDrives;
    }

    // Control task: the status last pushed.
    void report(const socket_message_GoalStatus &status) {
        std::lock_guard<std::mutex> lock(_mutex);
        _goalStatus = status;
    }

  private:
    mutable std::mutex _mutex;
    Modes _modes;
    ModeState _applied; // as the control task noted it last
    std::optional<CommandMsg> _command;
    unsigned long _commandMs = 0;
    std::optional<GaitType> _gait;
    GaitType _selection = GaitType::TRI_GATE; // the gait selected last, from the gait MotionService starts in
    std::optional<GoalRequest> _goal;
    uint32_t _goalCount = 0;
    GoalOwner _goalOwner {};
    unsigned long _ownerHeardMs = 0;
    // A goal drives the walk's sticks from its acceptance until it ends or an operator takes the walk back.
    bool _goalDrives = false;
    bool _goalCancelRequested = false;
    socket_message_GoalStatus _goalStatus = socket_message_GoalStatus_init_zero;
    bool _odometryReset = false;
    Joints _joints;
    bool _centreServos = false;
    OdometryEstimate _odometry {};
    // The arrival of the handheld's last frame with a stick off centre, until the first take past its window.
    unsigned long _handheldMovedMs = 0;
    bool _handheldMoved = false;
    // From the handheld's moving until a client's centred command after its window.
    bool _clientsWaitForCentre = false;
    bool _handheldDriving = false; // at the last take
    // The arrival of a client's last command that the handheld left through, until the first take past
    // CLIENT_IN_FORCE_MS.
    unsigned long _clientHeardMs = 0;
    bool _clientHeard = false;

    void cancelGoalLocked() {
        if (_goalDrives) _goalCancelRequested = true;
    }

    // A poster reads its clock before the lock, so a frame posted after that read carries a later time: the difference
    // is signed, as the owner's silence is.
    bool handheldDrivingLocked(unsigned long nowMs) const {
        return _handheldMoved && static_cast<long>(nowMs - _handheldMovedMs) < static_cast<long>(HANDHELD_PRIORITY_MS);
    }

    bool clientInForceLocked(unsigned long nowMs) const {
        return _clientHeard && static_cast<long>(nowMs - _clientHeardMs) < static_cast<long>(CLIENT_IN_FORCE_MS);
    }
};
