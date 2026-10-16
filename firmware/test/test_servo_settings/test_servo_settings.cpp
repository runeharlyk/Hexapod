// Host tests of whether the stored servo calibration is still the factory one, which /api/features
// reports so the app's robot list can say which robot still needs calibrating.

#include <unity.h>

#include <settings/servo_factory.h>

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

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_the_factory_calibration_is_not_a_calibration);
    RUN_TEST(test_one_moved_centre_makes_the_robot_calibrated);
    RUN_TEST(test_every_calibration_value_counts);
    RUN_TEST(test_a_renamed_servo_is_not_calibrated);
    RUN_TEST(test_a_real_calibration_with_a_factory_centre_left_in_it_counts);
    RUN_TEST(test_values_beyond_the_stored_count_are_ignored);
    return UNITY_END();
}
