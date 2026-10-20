#pragma once

// The firmware's Hexapod running off the robot: the real MotionService, gait, kinematics, policy and
// animation runner, built against the host stand-ins in firmware/host for the clock, the EventBus, the
// IMU and the servos. Each tick is one pass of main.cpp's 5 ms control loop. The commands a transport
// would publish are published the same way, so they reach the motion through its own subscriptions.
// The bus, clock and IMU stand-ins are process-wide, as on the robot, so one VirtualRobot exists at a time.

#include <communication/handheld_input.h>
#include <controller_input.h>
#include <hexapod.h>
#include <host_clock.h>

class VirtualRobot {
  public:
    static constexpr int64_t TICK_US = 5000;

    // Starts over from a level, still IMU as well, which the stand-in otherwise keeps from the robot before.
    VirtualRobot() {
        ServoController::reset();
        Peripherals::setImu({});
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
    // As the ESP-NOW adapter publishes the handheld controller's sticks, its height trim following the mode the last
    // tick reported; its buttons are left out.
    void handheld(const controller_packet_t &pkt) {
        CommandMsg c = handheldCommand(pkt, mode() == MOTION_STATE::STAND, _handheldHeight);
        if (c.sanitize()) EventBus<CommandMsg>::publish(c);
    }
    void setMode(MOTION_STATE mode) { EventBus<ModeMsg>::publish({mode}); }
    void setGait(GaitType gait) { EventBus<GaitMsg>::publish({gait}); }
    void setImu(const Peripherals::Imu &imu) { Peripherals::setImu(imu); }
    // As the animation adapters call the motion: the clip is loaded from ANIMATION_DIRECTORY, and a play the motion
    // accepts borrows ANIMATE through the bus.
    void animationPlay(const AnimationCommandMsg &play) { _robot.animationPlay(play); }
    void animationStop() { _robot.animationStop(); }
    void animationPose(const PoseMsg &pose) { _robot.animationPose(pose); }
    void animationControlLost() { _robot.animationControlLost(); }
    // As main.cpp's handlers: a joint dragged in the app, and the calibration page's centring with its reply's status.
    void setJoint(int servo, float degrees) { _robot.setJoint(servo, degrees); }
    int centreServos() { return _robot.centreServos(); }
    // As main.cpp's handler of the servo page's switch: the boards woken or put to sleep, whatever the mode.
    void servoSwitch(bool on) { EventBus<ServoStateMsg>::publish({on}); }

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
    // Whether the handheld drives, as main.cpp's mode bridge reads it.
    bool handheldDriving() const { return _robot.motion().handheldDriving(); }
    // The last command as the telemetry reports it, in TickSample.command's order.
    const float *telemetryCommand() const { return _sample.command; }
    // Whether the last tick's telemetry reports the dead-man stop in force.
    bool linkLost() const { return _sample.link_lost; }
    // What the last tick's telemetry reports of its servo write, as the stand-in controller records it.
    socket_message_ServoWrite servoWrite() const { return _sample.servo_write_right; }

    // What the servos receive: every one is driven once the boards are awake and each has been written since they woke,
    // to the last pose. The count of poses sent tells a tick that wrote one from a tick that left the servos where they
    // were.
    bool servosDriven() const {
        return ServoController::output().awake && ServoController::output().driven == ServoController::ALL_SERVOS;
    }
    // Whether the boards are awake, so the servos they drive hold their pulses.
    bool servosAwake() const { return ServoController::output().awake; }
    // A bit per servo written since the boards woke; the others are limp.
    uint32_t drivenServos() const { return ServoController::output().driven; }
    const float *servoAngles() const { return ServoController::output().angles; }
    unsigned servoWrites() const { return ServoController::output().writes; }
    // Single joints written in IDLE, and centrings of every servo.
    unsigned jointWrites() const { return ServoController::output().jointWrites; }
    unsigned centrings() const { return ServoController::output().centrings; }
    GaitType gait() const { return static_cast<GaitType>(_sample.gait); }
    // The stroke and cadence the gait is easing toward, as the motion holds them now.
    const gait_state_t &gaitCommand() const { return _robot.motion().gaitCommand(); }
    // The stroke and cadence the gait walks this tick.
    const gait_state_t &gaitState() const { return _robot.motion().gaitState(); }
    // The body pose the motion is easing toward.
    const BodyStateMsg &bodyCommand() const { return _robot.motion().bodyCommand(); }
    // The body pose and feet the last tick solved the legs for.
    const BodyStateMsg &bodyState() const { return _robot.motion().bodyState(); }
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
    float _handheldHeight = 0.0f;
};
