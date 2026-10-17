#pragma once

// What the other tasks hand MotionService's control task, under one mutex: the newest command, a goal asked for and
// its end, and an odometry reset. The decisions a reply or a goal depends on are made as each arrives, in arrival
// order; the control task takes what arrived at the start of its next tick, so nothing changes the motion in the
// middle of one. Ported in shape from SpotMicroESP32-Leika's esp32/include/motion_inbox.h.

#include <cstdint>
#include <mutex>
#include <optional>
#include <utility>
#include <goal_controller.h>
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

    struct Mail {
        std::optional<CommandMsg> command; // the newest since the last take
        unsigned long commandMs = 0;       // when it arrived
        std::optional<GoalRequest> goal;   // accepted since the last take
        bool goalCancelled = false;  // the end of the goal driving the walk was asked for, or its owner fell silent
        bool goalDriveEnded = false; // so a goal no longer drives the walk, whose sticks are to be centred
        bool odometryReset = false;
    };

    // The facade and the app send every 0.2 s; an owner silent ten times as long has gone, whether or not its transport
    // has said so. A goal with no transport, the host's own, has no owner to fall silent.
    static constexpr unsigned long OWNER_SILENCE_MS = 2000;

    // Adapter task: a goal accepted, replacing one not yet taken. It drives the walk's sticks from now until it ends or
    // an operator takes the walk back.
    void postGoal(const Goal &goal, const CommandMsg &style, GaitType gait, GoalOwner owner, unsigned long nowMs) {
        std::lock_guard<std::mutex> lock(_mutex);
        _goal = GoalRequest {goal, style, gait, ++_goalCount};
        _goalOwner = owner;
        _ownerHeardMs = nowMs;
        _goalDrives = true;
        // Whatever ended the goal before is done with: a cancel from now on is this goal's.
        _goalCancelRequested = false;
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
    // taken. While a goal drives the walk, a centred command is no command, its posture and the rest of it too: the
    // heartbeats of the app, the facade and the ESP-NOW controller. A stick off centre takes the walk back, so the goal
    // ends and never writes the sticks again, and the command is walked.
    void postCommand(const CommandMsg &c, unsigned long nowMs) {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_goalDrives) {
            if (c.lx == 0.f && c.ly == 0.f && c.rx == 0.f) return;
            _goalDrives = false;
            _goalCancelRequested = true;
        }
        _command = c;
        _commandMs = nowMs;
    }

    // Any task: the control task zeroes the pose before its next update. Refused while a goal drives: the pose is its
    // origin.
    bool requestOdometryReset() {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_goalDrives) return false;
        _odometryReset = true;
        return true;
    }

    // Any task: the newest goal's status, one accepted but not yet taken as settling.
    socket_message_GoalStatus goalStatus() const {
        std::lock_guard<std::mutex> lock(_mutex);
        return _goal ? goalStatusOf(_goal->id, GoalState::SETTLING, _goal->goal, 0.f, 0) : _goalStatus;
    }

    // Control task, at the start of every tick: what arrived since the last take. A goal whose end was asked for no
    // longer drives the walk from here on.
    Mail take(unsigned long nowMs) {
        std::lock_guard<std::mutex> lock(_mutex);
        Mail mail;
        mail.command = std::exchange(_command, std::nullopt);
        mail.commandMs = _commandMs;
        mail.goal = std::exchange(_goal, std::nullopt);
        mail.goalCancelled = std::exchange(_goalCancelRequested, false) ||
                             (_goalDrives && _goalOwner.adapter && nowMs - _ownerHeardMs > OWNER_SILENCE_MS);
        mail.goalDriveEnded = mail.goalCancelled && std::exchange(_goalDrives, false);
        mail.odometryReset = std::exchange(_odometryReset, false);
        return mail;
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
    std::optional<CommandMsg> _command;
    unsigned long _commandMs = 0;
    std::optional<GoalRequest> _goal;
    uint32_t _goalCount = 0;
    GoalOwner _goalOwner {};
    unsigned long _ownerHeardMs = 0;
    // A goal drives the walk's sticks from its acceptance until it ends or an operator takes the walk back.
    bool _goalDrives = false;
    bool _goalCancelRequested = false;
    socket_message_GoalStatus _goalStatus = socket_message_GoalStatus_init_zero;
    bool _odometryReset = false;

    void cancelGoalLocked() {
        if (_goalDrives) _goalCancelRequested = true;
    }
};
