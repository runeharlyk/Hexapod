// Host tests of the check that guards the MPU6050 calibration: it must refuse a robot that moves or
// lies tilted, since the calibration would bake either into every later reading, and must accept
// one at rest whatever its uncalibrated gyro bias.

#include <unity.h>

#include <cmath>
#include <peripherals/imu_calibration.h>

namespace {

using imu_calibration::Check;

constexpr size_t N = 100;
int16_t gyro[N][3];

// A resting chip: a constant bias (the thing being calibrated away) plus +-1 count of noise.
void fillResting(int16_t bias) {
    for (size_t i = 0; i < N; ++i)
        for (int a = 0; a < 3; ++a) gyro[i][a] = bias + ((i + a) % 3) - 1;
}

const int16_t LEVEL[3] = {0, 0, 16384};

}  // namespace

void setUp() { fillResting(0); }
void tearDown() {}

void test_a_resting_level_robot_may_be_calibrated() { TEST_ASSERT_TRUE(imu_calibration::check(gyro, N, LEVEL) == Check::OK); }

void test_a_large_bias_alone_is_no_motion() {
    fillResting(328);  // 20 deg/s, the MPU6050's worst-case zero-rate offset
    TEST_ASSERT_TRUE(imu_calibration::check(gyro, N, LEVEL) == Check::OK);
}

void test_a_robot_being_moved_is_refused() {
    // A slow 3 deg/s rock about one axis.
    for (size_t i = 0; i < N; ++i) gyro[i][1] = (int16_t)(49 * std::sin(i * 0.2));
    TEST_ASSERT_TRUE(imu_calibration::check(gyro, N, LEVEL) == Check::MOVING);
}

void test_a_robot_tilted_past_the_limit_is_refused() {
    // 30 deg of roll.
    const int16_t tilted[3] = {0, (int16_t)(16384 * std::sin(0.524)), (int16_t)(16384 * std::cos(0.524))};
    TEST_ASSERT_TRUE(imu_calibration::check(gyro, N, tilted) == Check::TILTED);
    // 15 deg is a floor that is not quite flat, and is accepted.
    const int16_t slight[3] = {(int16_t)(16384 * std::sin(0.262)), 0, (int16_t)(16384 * std::cos(0.262))};
    TEST_ASSERT_TRUE(imu_calibration::check(gyro, N, slight) == Check::OK);
}

void test_an_upside_down_or_silent_accelerometer_is_refused() {
    const int16_t upsideDown[3] = {0, 0, -16384};
    TEST_ASSERT_TRUE(imu_calibration::check(gyro, N, upsideDown) == Check::TILTED);
    const int16_t silent[3] = {0, 0, 0};
    TEST_ASSERT_TRUE(imu_calibration::check(gyro, N, silent) == Check::TILTED);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_a_resting_level_robot_may_be_calibrated);
    RUN_TEST(test_a_large_bias_alone_is_no_motion);
    RUN_TEST(test_a_robot_being_moved_is_refused);
    RUN_TEST(test_a_robot_tilted_past_the_limit_is_refused);
    RUN_TEST(test_an_upside_down_or_silent_accelerometer_is_refused);
    return UNITY_END();
}
