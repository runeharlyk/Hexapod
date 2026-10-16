// Host tests of which PCA9685 channel each joint drives, and which way, against the I2C register fake: the
// robot's legs 0-2 are its right side (docs/conventions.md), wired to the board at 0x41, and a positive coxa angle
// turns every leg counter-clockwise seen from above. The include path puts the host stand-in of the controller first,
// so the robot's own is included by its path.

#include <unity.h>

#include "../../include/peripherals/servo_controller.h"

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

// The robot standing with every joint at zero except `servo` at `angle` degrees, as the control loop sends it.
void drive(int servo, float angle) {
    static ServoController controller;
    controller.begin();
    controller.activate();
    float angles[NUM_SERVO] = {};
    for (int leg = 0; leg < 6; leg++) angles[leg * 3 + 2] = -90.0f; // tibia at its servo centre
    angles[servo] = servo % 3 == 2 ? angle - 90.0f : angle;
    controller.setAngles(angles);
    controller.updateServoState();
}

} // namespace

void setUp() {
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

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_leg_0_coxa_is_the_right_boards_channel_8_and_turns_counter_clockwise);
    RUN_TEST(test_leg_3_coxa_is_the_left_boards_channel_8_and_turns_the_same_way);
    RUN_TEST(test_femur_and_tibia_run_opposite_ways_on_the_two_sides);
    RUN_TEST(test_each_side_runs_front_to_rear_on_channels_8_2_and_5);
    return UNITY_END();
}
