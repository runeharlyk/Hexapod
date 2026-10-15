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

    void planMotion() { updatedMotion = _motionService.updateMotion(); }

    void updateActuators() {
        if (updatedMotion) {
            _servoController.setAngles(_motionService.getAngles());
            _servoController.updateServoState();
        }
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
    }
    bool resting() const { return _motionService.resting(); }
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
