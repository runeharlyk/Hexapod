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

    // sense
    void readSensors() {
        EXECUTE_EVERY_N_MS(25, {
            if (_peripherals.readIMU()) {
                // EventBus<IMUAnglesMsg>::publish(_peripherals.getIMUAngles());
            }
        });
        // _peripherals.readMag();
    }

    // plan
    void planMotion() { updatedMotion = _motionService.updateMotion(); }

    // act
    void updateActuators() {
        if (updatedMotion) {
            _servoController.setAngles(_motionService.getAngles());
            _servoController.updateServoState();
        }
    }

    // communicate
    void emitTelemetry() {
        // EXECUTE_EVERY_N_MS(250, { _peripherals.emitIMU(); });
        if (updatedMotion) EXECUTE_EVERY_N_MS(20, { _motionService.publishState(); });
    }

  private:
    MotionService _motionService;
    Peripherals _peripherals;
    ServoController _servoController;
    bool updatedMotion = false;
};
