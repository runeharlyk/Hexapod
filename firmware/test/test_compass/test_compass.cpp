// Host tests of the compass maths: a hard iron shifts every reading by a constant, so the heading is wrong by an
// amount that depends on the direction until the fitted offset is removed, and an offset that cannot be a hard iron
// is refused.

#include <unity.h>

#include <cmath>
#include <initializer_list>
#include <limits>
#include <peripherals/compass.h>

namespace {

constexpr float DEG_TO_RAD = 3.14159265f / 180.0f;
constexpr float EARTH_UT = 48.0f;
const compass::Offset HARD_IRON {60.0f, -85.0f, 30.0f};

// What the chip reads when it points `heading` degrees from the +X axis in a field of Earth's strength.
void chipReading(float heading, const compass::Offset& hardIron, float raw[3]) {
    raw[0] = EARTH_UT * std::cos(heading * DEG_TO_RAD) + hardIron.x;
    raw[1] = EARTH_UT * std::sin(heading * DEG_TO_RAD) + hardIron.y;
    raw[2] = -30.0f + hardIron.z;
}

float angleBetween(float a, float b) {
    float d = std::fmod(std::fabs(a - b), 360.0f);
    return d > 180.0f ? 360.0f - d : d;
}

} // namespace

void setUp() {}
void tearDown() {}

void test_the_offset_is_subtracted_from_every_axis() {
    const float raw[3] = {10.0f, 20.0f, 30.0f};
    float corrected[3];
    compass::correct(raw, {1.0f, -2.0f, 3.0f}, corrected);
    TEST_ASSERT_EQUAL_FLOAT(9.0f, corrected[0]);
    TEST_ASSERT_EQUAL_FLOAT(22.0f, corrected[1]);
    TEST_ASSERT_EQUAL_FLOAT(27.0f, corrected[2]);
}

void test_removing_the_hard_iron_restores_the_heading_in_every_direction() {
    float worstWithout = 0.0f;
    for (float heading = 0.0f; heading < 360.0f; heading += 15.0f) {
        float raw[3], corrected[3];
        chipReading(heading, HARD_IRON, raw);
        compass::correct(raw, HARD_IRON, corrected);
        TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, angleBetween(compass::headingDegrees(corrected, 0.0f), heading));
        worstWithout = std::fmax(worstWithout, angleBetween(compass::headingDegrees(raw, 0.0f), heading));
    }
    // The test is only worth something if the uncorrected compass really is off.
    TEST_ASSERT_GREATER_THAN_FLOAT(45.0f, worstWithout);
}

void test_the_declination_is_added_and_the_heading_stays_in_range() {
    const float field[3] = {1.0f, 0.0f, 0.0f};
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 90.0f, compass::headingDegrees(field, 90.0f * DEG_TO_RAD));
    const float west[3] = {-1.0f, -0.01f, 0.0f};
    for (float declination : {-0.4f, 0.0f, 0.4f}) {
        const float heading = compass::headingDegrees(west, declination);
        TEST_ASSERT_TRUE(heading >= 0.0f && heading < 360.0f);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 351.0f, compass::headingDegrees(field, -9.0f * DEG_TO_RAD));
}

void test_an_offset_that_cannot_be_a_hard_iron_is_refused() {
    TEST_ASSERT_TRUE(compass::valid({}));
    TEST_ASSERT_TRUE(compass::valid(HARD_IRON));
    TEST_ASSERT_FALSE(compass::valid({compass::OFFSET_LIMIT_UT + 1.0f, 0.0f, 0.0f}));
    TEST_ASSERT_FALSE(compass::valid({0.0f, -compass::OFFSET_LIMIT_UT - 1.0f, 0.0f}));
    TEST_ASSERT_FALSE(compass::valid({0.0f, 0.0f, std::numeric_limits<float>::quiet_NaN()}));
    TEST_ASSERT_FALSE(compass::valid({std::numeric_limits<float>::infinity(), 0.0f, 0.0f}));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_the_offset_is_subtracted_from_every_axis);
    RUN_TEST(test_removing_the_hard_iron_restores_the_heading_in_every_direction);
    RUN_TEST(test_the_declination_is_added_and_the_heading_stays_in_range);
    RUN_TEST(test_an_offset_that_cannot_be_a_hard_iron_is_refused);
    return UNITY_END();
}
