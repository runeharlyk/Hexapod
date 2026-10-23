// Host tests of which PCA9685 channel each joint drives, and which way, against the I2C register fake: the
// robot's legs 0-2 are its right side (docs/conventions.md), wired to the board at 0x41, and a positive coxa angle
// turns every leg counter-clockwise seen from above. The include path puts the host stand-in of the controller first,
// so the robot's own is included by its path.

#include <unity.h>

#include <array>
#include <cmath>

#include "../../include/peripherals/servo_controller.h"
#include <event_bus.h>

namespace {

constexpr uint8_t RIGHT_BOARD = 0x41;
constexpr uint8_t LEFT_BOARD = 0x40;
constexpr uint8_t LED0_ON_L = 0x06;
constexpr uint8_t PRE_SCALE = 0xFE;
constexpr uint8_t MODE1 = 0x00;
constexpr int CENTRE = 306; // the factory centre PWM
constexpr int CONVERSION = 2;

I2CBus& bus() { return I2CBus::instance(); }

void powerOnChip(uint8_t addr) {
    auto& chip = bus().registers[addr];
    chip = {};
    chip[MODE1] = 0x11;
    chip[PRE_SCALE] = 0x1E;
    for (int ch = 0; ch < 16; ch++) chip[LED0_ON_L + 4 * ch + 3] = 0x10;
}

int offCount(uint8_t addr, int channel) {
    const auto& chip = bus().registers[addr];
    return chip[LED0_ON_L + 4 * channel + 2] | (chip[LED0_ON_L + 4 * channel + 3] << 8);
}

ServoController& started() {
    static ServoController controller;
    controller.begin();
    return controller;
}

// The robot standing with every joint at zero except `servo` at `angle` degrees, as the control loop sends it.
void drive(int servo, float angle) {
    ServoController& controller = started();
    controller.activate();
    float angles[NUM_SERVO] = {};
    for (int leg = 0; leg < 6; leg++) angles[leg * 3 + 2] = -90.0f; // tibia at its servo centre
    angles[servo] = servo % 3 == 2 ? angle - 90.0f : angle;
    controller.setAngles(angles);
    controller.updateServoState();
}

} // namespace

void setUp() {
    host_bus::resetAll();
    bus().reset();
    powerOnChip(RIGHT_BOARD);
    powerOnChip(LEFT_BOARD);
}

void tearDown() {}

void test_leg_0_coxa_is_the_right_boards_channel_8_and_turns_counter_clockwise() {
    drive(0, 10);
    TEST_ASSERT_EQUAL_INT(CENTRE + 10 * CONVERSION, offCount(RIGHT_BOARD, 8));
    TEST_ASSERT_EQUAL_INT(CENTRE, offCount(LEFT_BOARD, 8));
}

void test_leg_3_coxa_is_the_left_boards_channel_8_and_turns_the_same_way() {
    drive(9, 10);
    TEST_ASSERT_EQUAL_INT(CENTRE + 10 * CONVERSION, offCount(LEFT_BOARD, 8));
    TEST_ASSERT_EQUAL_INT(CENTRE, offCount(RIGHT_BOARD, 8));
}

// The femur and tibia servos are mounted mirrored across the body, so one angle drives the two sides' pulses apart.
void test_femur_and_tibia_run_opposite_ways_on_the_two_sides() {
    drive(1, 10);
    TEST_ASSERT_EQUAL_INT(CENTRE + 10 * CONVERSION, offCount(RIGHT_BOARD, 9));
    setUp();
    drive(10, 10);
    TEST_ASSERT_EQUAL_INT(CENTRE - 10 * CONVERSION, offCount(LEFT_BOARD, 9));
    setUp();
    drive(2, 10);
    TEST_ASSERT_EQUAL_INT(CENTRE - 10 * CONVERSION, offCount(RIGHT_BOARD, 10));
    setUp();
    drive(11, 10);
    TEST_ASSERT_EQUAL_INT(CENTRE + 10 * CONVERSION, offCount(LEFT_BOARD, 10));
}

// Front, middle and rear legs keep the same channels on both boards.
void test_each_side_runs_front_to_rear_on_channels_8_2_and_5() {
    const int coxaChannel[3] = {8, 2, 5};
    for (int leg = 0; leg < 3; leg++) {
        setUp();
        drive(leg * 3, 10);
        TEST_ASSERT_EQUAL_INT(CENTRE + 10 * CONVERSION, offCount(RIGHT_BOARD, coxaChannel[leg]));
        setUp();
        drive((leg + 3) * 3, 10);
        TEST_ASSERT_EQUAL_INT(CENTRE + 10 * CONVERSION, offCount(LEFT_BOARD, coxaChannel[leg]));
    }
}

// The calibration page writes one servo's raw pulse, and its "Centre all servos" sends every servo to its calibrated
// centre; both reach the same physical servo as the gait does.
void test_a_raw_pulse_reaches_the_servo_the_gait_drives() {
    ServoController& controller = started();
    controller.servoEvent({0, 400});
    controller.servoEvent({9, 410});
    TEST_ASSERT_EQUAL_INT(400, offCount(RIGHT_BOARD, 8));
    TEST_ASSERT_EQUAL_INT(410, offCount(LEFT_BOARD, 8));
}

void test_the_calibrated_centres_reach_their_own_servos() {
    ServoController& controller = started();
    api_ServoSettings settings = ServoSettings_defaults();
    settings.servos[0].center_pwm = 317;
    settings.servos[9].center_pwm = 321;
    controller.applyCalibration(settings);
    controller.activate();
    controller.setCenterPwm();
    TEST_ASSERT_EQUAL_INT(317, offCount(RIGHT_BOARD, 8));
    TEST_ASSERT_EQUAL_INT(321, offCount(LEFT_BOARD, 8));
    controller.applyCalibration(ServoSettings_defaults());
}

// The centre pose in joint angles: each servo's angle whose pulse is its calibrated centre, through the calibration's
// centre angle and the tibia's square knee, whichever way the servo turns.
void test_the_centre_angle_is_the_angle_of_the_centre_pulse() {
    ServoController& controller = started();
    api_ServoSettings settings = ServoSettings_defaults();
    settings.servos[0].center_angle = 5.0f;
    settings.servos[11].center_angle = -7.0f;
    controller.applyCalibration(settings);
    controller.activate();
    TEST_ASSERT_EQUAL_FLOAT(-5.0f, controller.centreAngle(0));
    TEST_ASSERT_EQUAL_FLOAT(7.0f - 90.0f, controller.centreAngle(11));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, controller.centreAngle(10));
    TEST_ASSERT_EQUAL_FLOAT(-90.0f, controller.centreAngle(2));
    const int channel[] = {8, 9, 10};
    for (int servo : {0, 2, 10, 11}) {
        controller.setJointAngle(servo, controller.centreAngle(servo));
        const uint8_t board = servo < 9 ? RIGHT_BOARD : LEFT_BOARD;
        TEST_ASSERT_EQUAL_INT_MESSAGE(CENTRE, offCount(board, channel[servo % 3]), "centre pulse");
    }
    controller.applyCalibration(ServoSettings_defaults());
}

// Both boards' channel registers, to tell which a write touched.
std::array<int, 32> channels() {
    std::array<int, 32> pulses {};
    for (int ch = 0; ch < 16; ch++) {
        pulses[ch] = offCount(RIGHT_BOARD, ch);
        pulses[16 + ch] = offCount(LEFT_BOARD, ch);
    }
    return pulses;
}

// A joint dragged in IDLE: its angle reaches its own servo through the calibration and direction the gait's angles
// take, and no other channel on either board is written, so a servo nobody touched holds whatever pulse it had.
void test_a_joint_angle_moves_only_its_own_servo() {
    ServoController& controller = started();
    controller.activate();
    const std::array<int, 32> before = channels();
    controller.setJointAngle(4, 20.0f); // right middle femur, the right board's channel 3
    std::array<int, 32> after = channels();
    TEST_ASSERT_EQUAL_INT(CENTRE + 20 * CONVERSION, after[3]);
    after[3] = before[3];
    TEST_ASSERT_EQUAL_INT_ARRAY(before.data(), after.data(), 32);
}

// The tibia's angle is the kinematics', so its servo takes the square knee's 90 degrees first, and the left femur
// runs the other way, as for the gait.
void test_a_joint_angle_takes_the_tibia_offset_and_the_sides_direction() {
    ServoController& controller = started();
    controller.activate();
    controller.setJointAngle(2, -100.0f); // right front tibia, channel 10: -100 + 90 at direction -1
    TEST_ASSERT_EQUAL_INT(CENTRE + 10 * CONVERSION, offCount(RIGHT_BOARD, 10));
    controller.setJointAngle(10, 15.0f); // left front femur, channel 9, direction -1
    TEST_ASSERT_EQUAL_INT(CENTRE - 15 * CONVERSION, offCount(LEFT_BOARD, 9));
}

void test_a_joint_angle_is_held_within_its_travel() {
    ServoController& controller = started();
    controller.activate();
    controller.setJointAngle(0, 80.0f);
    TEST_ASSERT_EQUAL_INT(CENTRE + static_cast<int>(JOINT_LIMIT_DEG[0] * CONVERSION), offCount(RIGHT_BOARD, 8));
    controller.setJointAngle(9, -80.0f);
    TEST_ASSERT_EQUAL_INT(CENTRE - static_cast<int>(JOINT_LIMIT_DEG[0] * CONVERSION), offCount(LEFT_BOARD, 8));
}

// Input from the network: a servo that does not exist or an angle that is no number writes nothing.
void test_a_malformed_joint_angle_writes_nothing() {
    ServoController& controller = started();
    controller.activate();
    const std::array<int, 32> before = channels();
    controller.setJointAngle(-1, 10.0f);
    controller.setJointAngle(NUM_SERVO, 10.0f);
    controller.setJointAngle(4, std::nanf(""));
    controller.setJointAngle(4, INFINITY);
    const std::array<int, 32> after = channels();
    TEST_ASSERT_EQUAL_INT_ARRAY(before.data(), after.data(), 32);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_leg_0_coxa_is_the_right_boards_channel_8_and_turns_counter_clockwise);
    RUN_TEST(test_leg_3_coxa_is_the_left_boards_channel_8_and_turns_the_same_way);
    RUN_TEST(test_femur_and_tibia_run_opposite_ways_on_the_two_sides);
    RUN_TEST(test_each_side_runs_front_to_rear_on_channels_8_2_and_5);
    RUN_TEST(test_a_raw_pulse_reaches_the_servo_the_gait_drives);
    RUN_TEST(test_the_calibrated_centres_reach_their_own_servos);
    RUN_TEST(test_the_centre_angle_is_the_angle_of_the_centre_pulse);
    RUN_TEST(test_a_joint_angle_moves_only_its_own_servo);
    RUN_TEST(test_a_joint_angle_takes_the_tibia_offset_and_the_sides_direction);
    RUN_TEST(test_a_joint_angle_is_held_within_its_travel);
    RUN_TEST(test_a_malformed_joint_angle_writes_nothing);
    return UNITY_END();
}
