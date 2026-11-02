#pragma once

// Host stand-in for firmware/include/peripherals/servo_controller.h with the interface Hexapod and the
// motion code use. It drives no PCA9685; it keeps what the servos would receive, the joint angles last sent
// held within travel as the robot's controller holds them, whether the boards are awake and which servos a pulse
// drives, which is what a simulated robot's actuators need. It keeps angles rather than pulses, so of a calibration it
// takes only each coxa's travel, the factory's until one arrives on ServoSettingsBus, and none of the servo page's
// single pulses (ServoSignalMsg), and a write never fails, so no board is backed off, left asleep or started again.
// Hexapod owns its ServoController, so the output and the coxa travel are shared by every instance and read through
// output() and coxaTravelOf().

#include <cmath>
#include <kinematics.h>
#include <message_types.h>
#include <platform_shared/message.pb.h>
#include <servo_travel.h>
#include <settings/servo_settings.h>

class ServoController {
  public:
    static constexpr uint32_t ALL_SERVOS = (1u << NUM_SERVO) - 1;

    struct Output {
        bool awake = false;    // activated, so the PCA9685s run
        unsigned writes = 0;   // poses sent since boot
        uint32_t driven = 0;   // a bit per servo written since the boards were activated; the rest are limp
        unsigned jointWrites = 0; // single joints written, in IDLE
        unsigned centrings = 0;   // every servo sent to its centre pulse, in IDLE
        float angles[NUM_SERVO] = {};
        JointTravel coxa[6] = {factoryCoxa(0), factoryCoxa(1), factoryCoxa(2),
                               factoryCoxa(3), factoryCoxa(4), factoryCoxa(5)};
    };

    static const Output &output() { return state(); }
    static void reset() { state() = {}; }

    // As the robot's controller does: a calibration published on ServoSettingsBus is applied when the bus delivers it,
    // and one published before, as by a settings service that started first, at once. The bus keeps its last value
    // until host_bus::resetAll(), so a robot made after another takes the calibration last published to that one.
    void begin() {
        _calibrationSub = ServoSettingsBus::subscribe([](ServoSettings const &s) { applyCalibration(s); });
        ServoSettings pending;
        if (ServoSettingsBus::peek(pending)) applyCalibration(pending);
    }
    static void applyCalibration(const ServoSettings &s) {
        for (int leg = 0; leg < 6 && leg * 3 < static_cast<int>(s.servos_count); ++leg) {
            const api_Servo &c = s.servos[leg * 3];
            state().coxa[leg] = coxaTravel(leg, c.center_pwm, c.conversion, c.direction, c.center_angle);
        }
    }
    static JointTravel coxaTravelOf(int leg) { return state().coxa[leg]; }
    // As the robot's controller does: the boards wake with every channel off, so every servo is limp until written.
    void activate() {
        if (state().awake) return;
        state().awake = true;
        state().driven = 0;
    }
    uint32_t driven() const { return state().driven; }
    void deactivate() {
        state().awake = false;
        state().driven = 0;
    }
    // At the factory centres, which this stand-in keeps whatever calibration arrives, as it takes only each coxa's
    // travel from one: every coxa and femur at 0, every tibia at -90.
    float centreAngle(int servo) const { return servo % 3 == 2 ? -90.0f : 0.0f; }
    // As the robot's controller holds a joint: a coxa within its calibration's travel, at the factory one its leg's
    // full JOINT_LIMIT_DEG.
    static float withinTravel(int servo, float angleDeg, CoxaTravel travel = CoxaTravel::SERVO) {
        return jointWithinTravel(angleDeg, servo, state().coxa[servo / 3], travel);
    }
    // As the robot's controller does: a pulse sent to sleeping boards drives no servo.
    void setCenterPwm() {
        for (int i = 0; i < NUM_SERVO; ++i) state().angles[i] = centreAngle(i);
        state().centrings++;
        if (state().awake) state().driven = ALL_SERVOS;
        _right = _left = socket_message_ServoWrite_SERVO_WRITE_OK;
    }
    // As the robot's controller does: that joint alone, held within its travel, a write of its own board (servos 0-8
    // on the right one); a malformed one writes nothing.
    void setJointAngle(int servo, float angleDeg) {
        if (servo < 0 || servo >= NUM_SERVO || !std::isfinite(angleDeg)) return;
        state().angles[servo] = withinTravel(servo, angleDeg);
        state().jointWrites++;
        if (state().awake) state().driven |= 1u << servo;
        (servo < 9 ? _right : _left) = socket_message_ServoWrite_SERVO_WRITE_OK;
    }
    // As the robot's controller does: each joint held within its travel, and a non-finite command ignored.
    void setAngles(float angles[NUM_SERVO], CoxaTravel travel) {
        for (int i = 0; i < NUM_SERVO; ++i)
            if (std::isfinite(angles[i])) state().angles[i] = withinTravel(i, angles[i], travel);
        state().writes++;
        if (state().awake) state().driven = ALL_SERVOS;
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
    static constexpr JointTravel factoryCoxa(int leg) {
        return coxaTravel(leg, FACTORY_CENTER_PWM, FACTORY_CONVERSION, FACTORY_DIRECTION[leg * 3],
                          FACTORY_CENTER_ANGLE);
    }

    ServoSettingsBus::Handle _calibrationSub;
    socket_message_ServoWrite _right = socket_message_ServoWrite_SERVO_WRITE_NONE;
    socket_message_ServoWrite _left = socket_message_ServoWrite_SERVO_WRITE_NONE;

    static Output &state() {
        static Output output;
        return output;
    }
};
