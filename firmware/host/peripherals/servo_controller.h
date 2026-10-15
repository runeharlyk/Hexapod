#pragma once

// Host stand-in for firmware/include/peripherals/servo_controller.h with the interface Hexapod and the
// motion code use. It drives no PCA9685; it keeps what the servos would receive, the joint angles last sent
// and whether the boards are awake, which is what a simulated robot's actuators need. Hexapod owns its
// ServoController, so the output is shared by every instance and read through output().

#include <algorithm>
#include <message_types.h>

class ServoController {
  public:
    struct Output {
        bool awake = false;    // activated, so the PCA9685s run
        unsigned writes = 0;   // poses sent since boot; until the first an awake board drives no pulse
        float angles[NUM_SERVO] = {};
    };

    static const Output &output() { return state(); }
    static void reset() { state() = {}; }

    void begin() {}
    void activate() { state().awake = true; }
    void deactivate() { state().awake = false; }
    // The centre PWMs are the calibration's, which this stand-in does not hold; the joints keep their angles.
    void setCenterPwm() {}
    void setAngles(float angles[NUM_SERVO]) {
        std::copy_n(angles, NUM_SERVO, state().angles);
        state().writes++;
    }
    void updateServoState() {}

  private:
    static Output &state() {
        static Output output;
        return output;
    }
};
