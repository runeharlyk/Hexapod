// Host tests of whether the stored servo calibration is still the factory one, which /api/features
// reports so the app's robot list can say which robot still needs calibrating.

#include <unity.h>

#include <settings/servo_factory.h>
#include <settings/servo_settings.h>

void setUp() {}
void tearDown() {}

void test_the_factory_calibration_is_not_a_calibration() {
    TEST_ASSERT_FALSE(ServoSettings_calibrated(ServoSettings_defaults()));
}

void test_one_moved_centre_makes_the_robot_calibrated() {
    ServoSettings settings = ServoSettings_defaults();
    settings.servos[17].center_pwm = 302;
    TEST_ASSERT_TRUE(ServoSettings_calibrated(settings));
}

void test_every_calibration_value_counts() {
    ServoSettings angle = ServoSettings_defaults();
    angle.servos[4].center_angle = 15;
    TEST_ASSERT_TRUE(ServoSettings_calibrated(angle));

    ServoSettings conversion = ServoSettings_defaults();
    conversion.servos[9].conversion = 2.2f;
    TEST_ASSERT_TRUE(ServoSettings_calibrated(conversion));

    // Servo 3 runs the other way round at the factory, so flipping it is a change while servo 1's
    // factory direction is not.
    ServoSettings direction = ServoSettings_defaults();
    direction.servos[2].direction = 1;
    TEST_ASSERT_TRUE(ServoSettings_calibrated(direction));
    direction = ServoSettings_defaults();
    direction.servos[0].direction = 1;
    TEST_ASSERT_FALSE(ServoSettings_calibrated(direction));
}

void test_a_renamed_servo_is_not_calibrated() {
    ServoSettings settings = ServoSettings_defaults();
    settings.servos[0].name[0] = 'L';
    TEST_ASSERT_FALSE(ServoSettings_calibrated(settings));
}

// Read from a robot at 192.168.1.151 on 2026-10-15: one centre still sits at the factory 306.
void test_a_real_calibration_with_a_factory_centre_left_in_it_counts() {
    const float centres[FACTORY_SERVO_COUNT] = {321, 271, 306, 329, 304, 307, 304, 292, 285,
                                                317, 307, 334, 291, 295, 305, 338, 292, 302};
    ServoSettings settings = ServoSettings_defaults();
    for (pb_size_t i = 0; i < FACTORY_SERVO_COUNT; i++) settings.servos[i].center_pwm = centres[i];
    TEST_ASSERT_TRUE(ServoSettings_calibrated(settings));
}

// Only the servos the settings hold are compared; values past servos_count are not settings.
void test_values_beyond_the_stored_count_are_ignored() {
    ServoSettings settings = ServoSettings_defaults();
    settings.servos_count = 6;
    settings.servos[6].center_pwm = 400;
    TEST_ASSERT_FALSE(ServoSettings_calibrated(settings));
    settings.servos[5].center_pwm = 400;
    TEST_ASSERT_TRUE(ServoSettings_calibrated(settings));
}

// A coxa's centre is a pulse it is driven to ("centre all servos") and the middle of its travel, so the robot refuses
// one outside the MG92B's safe 500 to 2500 us, 103 to 512 counts, and keeps the calibration it had. A femur or tibia
// centre is not held to it.
void test_a_coxa_centre_outside_the_safe_pulses_is_refused() {
    const ServoSettings stored = ServoSettings_defaults();
    for (const float centre : {102.9f, 512.1f, 80.0f, 600.0f}) {
        ServoSettings asked = stored, held = stored;
        asked.servos[3].center_pwm = centre;
        TEST_ASSERT_EQUAL(StateUpdateResult::ERROR, ServoSettings_update(asked, held));
        TEST_ASSERT_EQUAL_FLOAT(FACTORY_CENTER_PWM, held.servos[3].center_pwm);
    }
    for (const float centre : {103.0f, 512.0f}) {
        ServoSettings asked = stored, held = stored;
        asked.servos[15].center_pwm = centre;
        TEST_ASSERT_EQUAL(StateUpdateResult::CHANGED, ServoSettings_update(asked, held));
        TEST_ASSERT_EQUAL_FLOAT(centre, held.servos[15].center_pwm);
    }
    ServoSettings asked = stored, held = stored;
    asked.servos[4].center_pwm = 450.0f;
    asked.servos[5].center_pwm = 150.0f;
    TEST_ASSERT_EQUAL(StateUpdateResult::CHANGED, ServoSettings_update(asked, held));
}

// A coxa's travel at its calibration, the safe pulses within its leg's JOINT_LIMIT_DEG, holds angle 0, the coxa's in
// the stand: a centre angle or a conversion that moves the safe pulses wholly to one side would leave the coxa no
// travel within its limit at all (coxaTravel's min past its max), or none that stands. Centred at 306 counts, a centre
// angle of 180 deg at 100 counts a degree puts the safe pulses at -182 to -178 deg, and one of 110 deg at 2 counts a
// degree drives angle 0 at 526 counts, past 512; 100 deg at 2 counts drives it at 506.
void test_a_coxa_calibration_whose_travel_misses_angle_0_is_refused() {
    struct Case {
        float conversion, centerAngle, direction;
        StateUpdateResult result;
    };
    for (const Case c :
         {Case {100.0f, 180.0f, 1.0f, StateUpdateResult::ERROR}, Case {2.0f, 110.0f, 1.0f, StateUpdateResult::ERROR},
          Case {2.0f, -110.0f, -1.0f, StateUpdateResult::ERROR}, Case {2.0f, 100.0f, 1.0f, StateUpdateResult::CHANGED},
          Case {2.0f, 100.0f, -1.0f, StateUpdateResult::CHANGED}}) {
        const ServoSettings stored = ServoSettings_defaults();
        ServoSettings asked = stored, held = stored;
        asked.servos[9].conversion = c.conversion;
        asked.servos[9].center_angle = c.centerAngle;
        asked.servos[9].direction = c.direction;
        char msg[80];
        std::snprintf(msg, sizeof(msg), "conversion %.0f, centre angle %.0f, direction %+.0f", c.conversion,
                      c.centerAngle, c.direction);
        TEST_ASSERT_EQUAL_MESSAGE(c.result, ServoSettings_update(asked, held), msg);
    }
    // A femur or tibia is not held to it.
    const ServoSettings stored = ServoSettings_defaults();
    ServoSettings asked = stored, held = stored;
    asked.servos[10].conversion = 100.0f;
    asked.servos[10].center_angle = 180.0f;
    TEST_ASSERT_EQUAL(StateUpdateResult::CHANGED, ServoSettings_update(asked, held));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_the_factory_calibration_is_not_a_calibration);
    RUN_TEST(test_one_moved_centre_makes_the_robot_calibrated);
    RUN_TEST(test_every_calibration_value_counts);
    RUN_TEST(test_a_renamed_servo_is_not_calibrated);
    RUN_TEST(test_a_real_calibration_with_a_factory_centre_left_in_it_counts);
    RUN_TEST(test_values_beyond_the_stored_count_are_ignored);
    RUN_TEST(test_a_coxa_centre_outside_the_safe_pulses_is_refused);
    RUN_TEST(test_a_coxa_calibration_whose_travel_misses_angle_0_is_refused);
    return UNITY_END();
}
