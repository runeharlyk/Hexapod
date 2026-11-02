// Host tests of which PCA9685 channel each joint drives, and which way, against the I2C register fake: the
// robot's legs 0-2 are its right side (docs/conventions.md), wired to the board at 0x41, and a positive coxa angle
// turns every leg counter-clockwise seen from above. The include path puts the host stand-in of the controller first,
// so the robot's own is included by its path.

#include <unity.h>

#include <algorithm>
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
// The MG92B's safe 500 and 2500 us in PCA9685 counts at 50 Hz, 4096 to the 20 ms: 102.4 and 512, rounded inward.
constexpr int SAFE_PULSE_MIN = 103;
constexpr int SAFE_PULSE_MAX = 512;

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
    controller.setAngles(angles, CoxaTravel::SERVO);
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
    controller.servoEvent({9, 405});
    TEST_ASSERT_EQUAL_INT(400, offCount(RIGHT_BOARD, 8));
    TEST_ASSERT_EQUAL_INT(405, offCount(LEFT_BOARD, 8));
}

// The servo page's slider drives a coxa only within the servo's safe pulses, holding one beyond at the span's end; a
// femur or tibia takes the pulse it is sent.
void test_a_raw_pulse_holds_a_coxa_within_the_safe_pulses() {
    ServoController& controller = started();
    controller.servoEvent({0, 600});
    controller.servoEvent({9, 80});
    controller.servoEvent({3, 300});
    controller.servoEvent({1, 600});
    controller.servoEvent({11, 80});
    TEST_ASSERT_EQUAL_INT(SAFE_PULSE_MAX, offCount(RIGHT_BOARD, 8));
    TEST_ASSERT_EQUAL_INT(SAFE_PULSE_MIN, offCount(LEFT_BOARD, 8));
    TEST_ASSERT_EQUAL_INT(300, offCount(RIGHT_BOARD, 2));
    TEST_ASSERT_EQUAL_INT(600, offCount(RIGHT_BOARD, 9));
    TEST_ASSERT_EQUAL_INT(80, offCount(LEFT_BOARD, 10));
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

// A coxa centre from before the robot refused one outside the safe pulses, stored and applied: centring sends that coxa
// to the end of the span instead, and publishes the angle of the pulse it sent.
void test_centring_holds_a_coxa_within_the_safe_pulses() {
    ServoController& controller = started();
    api_ServoSettings settings = ServoSettings_defaults();
    settings.servos[0].center_pwm = 560.0f;
    settings.servos[9].center_pwm = 60.0f;
    settings.servos[1].center_pwm = 560.0f;
    controller.applyCalibration(settings);
    controller.activate();
    controller.setCenterPwm();
    TEST_ASSERT_EQUAL_INT(SAFE_PULSE_MAX, offCount(RIGHT_BOARD, 8));
    TEST_ASSERT_EQUAL_INT(SAFE_PULSE_MIN, offCount(LEFT_BOARD, 8));
    TEST_ASSERT_EQUAL_INT(560, offCount(RIGHT_BOARD, 9));       // a femur's centre is not held to it
    TEST_ASSERT_EQUAL_FLOAT(-24.0f, controller.centreAngle(0)); // (512 - 560) / 2
    TEST_ASSERT_EQUAL_FLOAT(21.5f, controller.centreAngle(9));  // (103 - 60) / 2
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

// A coxa centred off the factory's 306 counts has less than its leg's JOINT_LIMIT_DEG on one side, which the
// calibration names: centred at 338 counts at 2 counts a degree, servo 6 reaches +87 deg of its 90. The factory
// calibration leaves no side short, nor does a middle coxa's 45 deg at the same centre.
void test_a_calibration_names_each_coxa_side_short_of_the_joint_limit() {
    ServoController& controller = started();
    char sides[SHORT_COXA_SIDES_SIZE];
    const auto shortSides = [&] {
        JointTravel travel[6];
        for (int leg = 0; leg < 6; leg++) travel[leg] = controller.coxaTravelOf(leg);
        return shortCoxaSides(travel, sides, sizeof(sides));
    };
    api_ServoSettings settings = ServoSettings_defaults();
    controller.applyCalibration(settings);
    TEST_ASSERT_EQUAL_INT(0, shortSides());
    settings.servos[6].center_pwm = 338;
    settings.servos[3].center_pwm = 338;
    controller.applyCalibration(settings);
    TEST_ASSERT_EQUAL_INT(1, shortSides());
    TEST_ASSERT_EQUAL_STRING(" servo 6 +87.0", sides);
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

// A coxa turns as far as its leg allows within its servo's safe pulses: a corner leg's 90 deg, the servo's travel, and
// a middle leg's 45 deg, short of its servo connections meeting the body.
void test_a_joint_angle_is_held_within_its_legs_travel() {
    ServoController& controller = started();
    controller.activate();
    controller.setJointAngle(0, 120.0f);
    TEST_ASSERT_EQUAL_INT(CENTRE + 90 * CONVERSION, offCount(RIGHT_BOARD, 8));
    controller.setJointAngle(9, -120.0f);
    TEST_ASSERT_EQUAL_INT(CENTRE - 90 * CONVERSION, offCount(LEFT_BOARD, 8));
    controller.setJointAngle(3, 60.0f);
    TEST_ASSERT_EQUAL_INT(CENTRE + 45 * CONVERSION, offCount(RIGHT_BOARD, 2));
    controller.setJointAngle(12, -60.0f);
    TEST_ASSERT_EQUAL_INT(CENTRE - 45 * CONVERSION, offCount(LEFT_BOARD, 2));
}

// The classic gaits and IDLE turn a coxa 45 deg, which a front or rear leg needs to point straight out to its side.
void test_a_coxa_turns_45_deg_where_its_safe_pulses_allow() {
    ServoController& controller = started();
    controller.activate();
    drive(0, 45.0f);
    TEST_ASSERT_EQUAL_INT(CENTRE + 45 * CONVERSION, offCount(RIGHT_BOARD, 8));
    drive(15, -45.0f);
    TEST_ASSERT_EQUAL_INT(CENTRE - 45 * CONVERSION, offCount(LEFT_BOARD, 5));
    controller.setJointAngle(6, -45.0f);
    TEST_ASSERT_EQUAL_INT(CENTRE - 45 * CONVERSION, offCount(RIGHT_BOARD, 5));
}

// TUNED was searched and WALK_NN trained within 31.5 deg of coxa, which the servos keep for them alone.
void test_the_trained_travel_holds_the_coxa_at_31_5_deg() {
    ServoController& controller = started();
    controller.activate();
    float angles[NUM_SERVO] = {};
    for (int leg = 0; leg < 6; leg++) angles[leg * 3 + 2] = -90.0f;
    angles[0] = 45.0f;
    angles[9] = -45.0f;
    angles[1] = 45.0f;
    controller.setAngles(angles, CoxaTravel::TRAINED);
    controller.updateServoState();
    TEST_ASSERT_EQUAL_INT(CENTRE + 63, offCount(RIGHT_BOARD, 8)); // 31.5 deg at 2 counts per degree
    TEST_ASSERT_EQUAL_INT(CENTRE - 63, offCount(LEFT_BOARD, 8));
    TEST_ASSERT_EQUAL_INT(CENTRE + 45 * CONVERSION, offCount(RIGHT_BOARD, 9)); // the femur is no coxa
}

// A calibrated centre off the nominal 1.5 ms leaves a coxa more travel one way than the other, and never a pulse
// outside the safe span, whichever way the servo turns and whatever its counts per degree, from the gait or from IDLE.
void test_an_off_centre_calibration_never_leaves_the_safe_pulses() {
    ServoController& controller = started();
    controller.activate();
    api_ServoSettings settings = ServoSettings_defaults();
    settings.servos[0].center_pwm = 349.2f;
    settings.servos[0].conversion = 3.1f;
    settings.servos[9].center_pwm = 266.3f;
    settings.servos[9].conversion = 3.3f;
    settings.servos[9].direction = -1.0f;
    settings.servos[9].center_angle = 3.7f;
    controller.applyCalibration(settings);
    int lowest[2] = {4096, 4096}, highest[2] = {0, 0};
    for (float angle = -180.0f; angle <= 180.0f; angle += 0.37f) {
        drive(9, angle); // writes every channel, so before the joint written alone
        controller.setJointAngle(0, angle);
        const int pulses[2] = {offCount(RIGHT_BOARD, 8), offCount(LEFT_BOARD, 8)};
        for (int side = 0; side < 2; side++) {
            lowest[side] = std::min(lowest[side], pulses[side]);
            highest[side] = std::max(highest[side], pulses[side]);
        }
    }
    // Each side reaches its end of the span; toPwm truncates to a whole count, so the upper end, an angle clipped to
    // it in floating point, may land a count inside.
    for (int side = 0; side < 2; side++) {
        TEST_ASSERT_EQUAL_INT(SAFE_PULSE_MIN, lowest[side]);
        TEST_ASSERT_LESS_OR_EQUAL_INT(SAFE_PULSE_MAX, highest[side]);
        TEST_ASSERT_GREATER_OR_EQUAL_INT(SAFE_PULSE_MAX - 1, highest[side]);
    }
    // Servo 0's centre sits 43.2 counts high, at 3.1 counts per degree: 52.5 deg left to turn up, 79.4 down, both
    // inside its leg's 90. The pulse's own clamp holds an edge's angle that comes back a hair outside in floating
    // point.
    controller.setJointAngle(0, -60.0f);
    TEST_ASSERT_EQUAL_INT(163, offCount(RIGHT_BOARD, 8)); // 349.2 - 60 x 3.1 = 163.2
    controller.setJointAngle(0, 52.0f);
    TEST_ASSERT_LESS_OR_EQUAL_INT(SAFE_PULSE_MAX, offCount(RIGHT_BOARD, 8));
    controller.setJointAngle(0, -80.0f);
    TEST_ASSERT_EQUAL_INT(SAFE_PULSE_MIN, offCount(RIGHT_BOARD, 8));
    controller.applyCalibration(ServoSettings_defaults());
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
    RUN_TEST(test_a_raw_pulse_holds_a_coxa_within_the_safe_pulses);
    RUN_TEST(test_the_calibrated_centres_reach_their_own_servos);
    RUN_TEST(test_the_centre_angle_is_the_angle_of_the_centre_pulse);
    RUN_TEST(test_centring_holds_a_coxa_within_the_safe_pulses);
    RUN_TEST(test_a_joint_angle_moves_only_its_own_servo);
    RUN_TEST(test_a_joint_angle_takes_the_tibia_offset_and_the_sides_direction);
    RUN_TEST(test_a_joint_angle_is_held_within_its_legs_travel);
    RUN_TEST(test_a_coxa_turns_45_deg_where_its_safe_pulses_allow);
    RUN_TEST(test_the_trained_travel_holds_the_coxa_at_31_5_deg);
    RUN_TEST(test_an_off_centre_calibration_never_leaves_the_safe_pulses);
    RUN_TEST(test_a_malformed_joint_angle_writes_nothing);
    RUN_TEST(test_a_calibration_names_each_coxa_side_short_of_the_joint_limit);
    return UNITY_END();
}
