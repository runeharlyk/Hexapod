#pragma once

#include <peripherals/peripherals.h>
#include <peripherals/servo_controller.h>
#include <motion.h>
#include <event_bus.h>

class Hexapod {
  public:
    Hexapod() : _motionService(&_servoController, &_peripherals) {}

    void initialize() {
        _peripherals.begin();
        _servoController.begin();
        _motionService.begin();
    }

    void readSensors() {
        imuFresh = false;
        magFresh = false;
        EXECUTE_EVERY_N_MS(25, {
            const bool imu = _peripherals.readIMU();
            imuFresh = imu;
            // 15 Hz part; polling it on the 40 Hz IMU tick just returns the last sample.
            magFresh = _peripherals.readMagnetometer();
            if (imu || magFresh) EventBus<IMUAnglesMsg>::publish(_peripherals.getIMUAngles());
        });
    }

    // The servo write of a tick happens here (POSE's centre pulses) or in updateActuators.
    void planMotion() {
        _servoController.startTick();
        updatedMotion = _motionService.updateMotion(imuFresh);
    }

    void updateActuators() {
        if (updatedMotion) {
            _servoController.setAngles(_motionService.getAngles());
            _servoController.updateServoState();
        }
        _motionService.logMail();
    }

    std::vector<uint8_t> scanI2C() { return _peripherals.scanI2C(); }

    void animationPlay(const AnimationCommandMsg &c) { _motionService.handleAnimationCommand(c); }
    void animationStop() { _motionService.handleAnimationStop(); }
    void animationPose(const PoseMsg &p) { _motionService.handleAnimationPose(p); }
    void animationControlLost() { _motionService.animationControlLost(); }
    void validateAnimation(const char *name, socket_message_AnimationReport &r) {
        _motionService.validateAnimation(name, r);
    }

    bool imuPresent() const { return _peripherals.imuActive(); }

    void fillTelemetry(socket_message_TickSample &tick) {
        tick.imu_fresh = imuFresh;
        tick.mag_fresh = magFresh;
        _peripherals.fillTelemetry(tick);
        _motionService.fillTelemetry(tick);
        _servoController.fillTelemetry(tick);
    }
    bool resting() const { return _motionService.resting(); }
    const MotionService &motion() const { return _motionService; }
    bool resetOdometry() { return _motionService.resetOdometry(); }
    int startGoal(const Goal &goal, const CommandMsg &style, GaitType gait, GoalOwner owner) {
        return _motionService.startGoal(goal, style, gait, owner);
    }
    void cancelGoal() { _motionService.cancelGoal(); }
    void goalClientGone(GoalOwner owner) { _motionService.clientGone(owner); }
    void goalOwnerHeard(GoalOwner owner) { _motionService.ownerHeard(owner); }
    imu_calibration::Check calibrateIMU(ImuCalibration &result) { return _peripherals.calibrateIMU(result); }
    bool magnetometerPresent() const { return _peripherals.magActive(); }

    void emitTelemetry() {
        if (updatedMotion) EXECUTE_EVERY_N_MS(20, { _motionService.publishState(); });
    }

  private:
    MotionService _motionService;
    Peripherals _peripherals;
    ServoController _servoController;
    bool updatedMotion = false;
    bool imuFresh = false;
    bool magFresh = false;
};
