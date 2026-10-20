#pragma once

// Host stand-in for firmware/include/peripherals/servo_controller.h with the interface Hexapod and the
// motion code use. It drives no PCA9685; it keeps what the servos would receive, the joint angles last sent
// held within travel as the robot's controller holds them, and whether the boards are awake, which is what a simulated
// robot's actuators need. Hexapod owns its ServoController, so the output is shared by every instance and read through
// output().

#include <cmath>
#include <kinematics.h>
#include <message_types.h>
#include <platform_shared/message.pb.h>

class ServoController {
  public:
    struct Output {
        bool awake = false;    // activated, so the PCA9685s run
        unsigned writes = 0;   // poses sent since boot; until the first an awake board drives no pulse
        unsigned jointWrites = 0; // single joints written, in IDLE
        unsigned centrings = 0;   // every servo sent to its centre pulse, in IDLE
        float angles[NUM_SERVO] = {};
    };

    static const Output &output() { return state(); }
    static void reset() { state() = {}; }

    void begin() {}
    void activate() { state().awake = true; }
    void deactivate() { state().awake = false; }
    // At the factory calibration, which this stand-in holds alone: every coxa and femur at 0, every tibia at -90.
    float centreAngle(int servo) const { return servo % 3 == 2 ? -90.0f : 0.0f; }
    void setCenterPwm() {
        for (int i = 0; i < NUM_SERVO; ++i) state().angles[i] = centreAngle(i);
        state().centrings++;
        _right = _left = socket_message_ServoWrite_SERVO_WRITE_OK;
    }
    // As the robot's controller does: that joint alone, held within its travel, a write of its own board (servos 0-8
    // on the right one); a malformed one writes nothing.
    void setJointAngle(int servo, float angleDeg) {
        if (servo < 0 || servo >= NUM_SERVO || !std::isfinite(angleDeg)) return;
        state().angles[servo] = withinTravel(angleDeg, servo % 3);
        state().jointWrites++;
        (servo < 9 ? _right : _left) = socket_message_ServoWrite_SERVO_WRITE_OK;
    }
    // As the robot's controller does: each joint held within its travel, and a non-finite command ignored.
    void setAngles(float angles[NUM_SERVO]) {
        for (int i = 0; i < NUM_SERVO; ++i)
            if (std::isfinite(angles[i])) state().angles[i] = withinTravel(angles[i], i % 3);
        state().writes++;
        _right = _left = socket_message_ServoWrite_SERVO_WRITE_OK;
    }
    void updateServoState() {}
    // As the robot's controller does: a tick that sends nothing reports no write, and one that does reports it
    // succeeded on each board it wrote, which this stand-in never fails.
    void startTick() { _right = _left = socket_message_ServoWrite_SERVO_WRITE_NONE; }
    void fillTelemetry(socket_message_TickSample &tick) const {
        tick.servo_write_right = _right;
        tick.servo_write_left = _left;
    }

  private:
    socket_message_ServoWrite _right = socket_message_ServoWrite_SERVO_WRITE_NONE;
    socket_message_ServoWrite _left = socket_message_ServoWrite_SERVO_WRITE_NONE;

    static Output &state() {
        static Output output;
        return output;
    }
};
