#pragma once

// The firmware's Hexapod running off the robot: the real MotionService, gait, kinematics, policy and
// animation runner, built against the host stand-ins in firmware/host for the clock, the EventBus, the
// IMU and the servos. Each tick is one pass of main.cpp's 5 ms control loop. The commands a transport
// would publish are published the same way, so they reach the motion through its own subscriptions.
// The bus, clock and IMU stand-ins are process-wide, as on the robot, so one VirtualRobot exists at a time.

#include <hexapod.h>
#include <host_clock.h>

class VirtualRobot {
  public:
    static constexpr int64_t TICK_US = 5000;

    VirtualRobot() {
        _robot.initialize();
        host_bus::drainAll();
    }

    // Delivers what this robot still has queued before its subscriptions go.
    ~VirtualRobot() { host_bus::drainAll(); }

    VirtualRobot(const VirtualRobot &) = delete;
    VirtualRobot &operator=(const VirtualRobot &) = delete;

    void command(const CommandMsg &c) { EventBus<CommandMsg>::publish(c); }
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
    GaitType gait() const { return static_cast<GaitType>(_sample.gait); }

  private:
    Hexapod _robot;
    socket_message_TickSample _sample = socket_message_TickSample_init_zero;
};
