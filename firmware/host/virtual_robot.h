#pragma once

// The firmware's Hexapod running off the robot: the real MotionService, gait, kinematics, policy and
// animation runner, built against the host stand-ins in firmware/host for the clock, the EventBus, the
// IMU and the servos. Each tick is one pass of main.cpp's 5 ms control loop. The commands a transport
// would publish are published the same way, so they reach the motion through its own subscriptions.
// The bus, clock and IMU stand-ins are process-wide, as on the robot, so one VirtualRobot exists at a time.

#include <controller_input.h>
#include <hexapod.h>
#include <host_clock.h>

class VirtualRobot {
  public:
    static constexpr int64_t TICK_US = 5000;

    VirtualRobot() {
        ServoController::reset();
        _robot.initialize();
        host_bus::drainAll();
    }

    // Delivers what this robot still has queued before its subscriptions go.
    ~VirtualRobot() { host_bus::drainAll(); }

    VirtualRobot(const VirtualRobot &) = delete;
    VirtualRobot &operator=(const VirtualRobot &) = delete;

    // As main.cpp's controller-input handler: a command with a non-finite field is dropped, the rest clamped.
    void command(CommandMsg c) {
        if (c.sanitize()) EventBus<CommandMsg>::publish(c);
    }
    void input(const socket_message_ControllerInputData &in) { command(commandFromInput(in)); }
    void setMode(MOTION_STATE mode) { EventBus<ModeMsg>::publish({mode}); }
    void setGait(GaitType gait) { EventBus<GaitMsg>::publish({gait}); }
    void setImu(const Peripherals::Imu &imu) { Peripherals::setImu(imu); }

    void tick() {
        host_clock::advance(TICK_US);
        host_bus::drainAll();
        _robot.readSensors();
        _robot.planMotion();
        _robot.updateActuators();
        host_bus::drainAll();
        _sample = socket_message_TickSample_init_zero;
        _robot.fillTelemetry(_sample);
    }

    // The joint angles the motion produced on the last tick, in degrees, three per leg in leg order.
    const float *angles() const { return _sample.angles; }
    MOTION_STATE mode() const { return static_cast<MOTION_STATE>(_sample.mode); }
    // The last command as the telemetry reports it, in TickSample.command's order.
    const float *telemetryCommand() const { return _sample.command; }
    // Whether the last tick's telemetry reports the dead-man stop in force.
    bool linkLost() const { return _sample.link_lost; }

    // What the servos receive: they are driven once the boards are awake and a pose has been sent, to the last pose.
    // The count of poses sent tells a tick that wrote one from a tick that left the servos where they were.
    bool servosDriven() const { return ServoController::output().awake && ServoController::output().writes > 0; }
    const float *servoAngles() const { return ServoController::output().angles; }
    unsigned servoWrites() const { return ServoController::output().writes; }
    GaitType gait() const { return static_cast<GaitType>(_sample.gait); }
    // The stroke and cadence the gait is easing toward, as the motion holds them now.
    const gait_state_t &gaitCommand() const { return _robot.motion().gaitCommand(); }
    // The stroke and cadence the gait walks this tick.
    const gait_state_t &gaitState() const { return _robot.motion().gaitState(); }
    // The body pose the motion is easing toward.
    const BodyStateMsg &bodyCommand() const { return _robot.motion().bodyCommand(); }
    // The odometry as the last tick's telemetry reports it.
    OdometryEstimate odometry() const {
        return {.x = _sample.odom_x,
                .y = _sample.odom_y,
                .yaw = _sample.odom_yaw,
                .kinematicYaw = _sample.odom_kinematic_yaw,
                .distance = _sample.odom_distance};
    }
    // Takes effect on the next tick, as a reset from a transport does; refused while a goal drives.
    bool resetOdometry() { return _robot.resetOdometry(); }
    // As main.cpp's goal_move handler, whose OTA check has no counterpart here: the reply's status. The goal starts on
    // the next tick.
    int goalMove(const Goal &goal, const CommandMsg &style, GaitType gait, GoalOwner owner = {}) {
        return _robot.startGoal(goal, style, gait, owner);
    }
    void goalCancel() { _robot.cancelGoal(); }
    socket_message_GoalStatus goalStatus() const { return _robot.motion().goalStatus(); }
    // A transport's client has gone.
    void clientGone(GoalOwner owner) { _robot.goalClientGone(owner); }
    // A transport's client sent something.
    void ownerHeard(GoalOwner owner) { _robot.goalOwnerHeard(owner); }

  private:
    Hexapod _robot;
    socket_message_TickSample _sample = socket_message_TickSample_init_zero;
};
