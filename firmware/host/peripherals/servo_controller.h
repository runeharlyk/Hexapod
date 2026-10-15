#pragma once

// Host stand-in for firmware/include/peripherals/servo_controller.h with the interface Hexapod and the
// motion code use. It drives no PCA9685; it keeps the joint angles last sent and whether the servos are
// powered, which is what a simulated robot's actuators need.

#include <algorithm>
#include <message_types.h>

class ServoController {
  public:
    void begin() {}
    void activate() { _active = true; }
    void deactivate() { _active = false; }
    // The centre PWMs are the calibration's, which this stand-in does not hold; the joints keep their angles.
    void setCenterPwm() {}
    void setAngles(float angles[NUM_SERVO]) { std::copy_n(angles, NUM_SERVO, _angles); }
    void updateServoState() {}

    bool active() const { return _active; }
    const float *angles() const { return _angles; }

  private:
    bool _active = false;
    float _angles[NUM_SERVO] = {};
};
