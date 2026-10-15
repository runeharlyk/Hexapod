// Host tests of what the status LED shows: one colour per mode that a person can tell apart, steady
// with a client and blinking without one, sampled at the service task's 100 ms rate.

#include <unity.h>

#include <iterator>
#include <peripherals/status_led_pattern.h>

using status_led::Rgb;

namespace {

constexpr MOTION_STATE MODES[] = {MOTION_STATE::DEACTIVATED, MOTION_STATE::IDLE, MOTION_STATE::POSE,
                                  MOTION_STATE::STAND,       MOTION_STATE::WALK, MOTION_STATE::WALK_NN,
                                  MOTION_STATE::ANIMATE};
constexpr int64_t SERVICE_PERIOD_US = 100000;

bool dark(const Rgb& c) { return c.r == 0 && c.g == 0 && c.b == 0; }

} // namespace

void setUp() {}
void tearDown() {}

void test_every_mode_has_its_own_visible_colour() {
    for (size_t i = 0; i < std::size(MODES); i++) {
        const Rgb a = status_led::modeColour(MODES[i]);
        TEST_ASSERT_FALSE_MESSAGE(dark(a), "a mode shows nothing, which reads as no power");
        for (size_t j = i + 1; j < std::size(MODES); j++) {
            TEST_ASSERT_FALSE_MESSAGE(a == status_led::modeColour(MODES[j]), "two modes share a colour");
        }
    }
}

// Pure red is the battery warning's; a mode in red would hide it.
void test_no_mode_is_shown_in_red() {
    for (MOTION_STATE mode : MODES) {
        const Rgb c = status_led::modeColour(mode);
        TEST_ASSERT_TRUE(c.g > 0 || c.b > 0);
    }
}

void test_with_a_client_the_mode_colour_is_steady() {
    for (MOTION_STATE mode : MODES) {
        for (int64_t t = 0; t < 3 * status_led::BLINK_PERIOD_US; t += SERVICE_PERIOD_US) {
            TEST_ASSERT_TRUE(status_led::colourAt({mode, true}, t) == status_led::modeColour(mode));
        }
    }
}

// Without a client the mode stays readable: its colour half of each second, dark the other half.
void test_without_a_client_the_mode_colour_blinks_once_a_second() {
    for (MOTION_STATE mode : MODES) {
        int lit = 0, darkSamples = 0, changes = 0;
        Rgb previous = status_led::colourAt({mode, false}, 0);
        for (int64_t t = 0; t < 3 * status_led::BLINK_PERIOD_US; t += SERVICE_PERIOD_US) {
            const Rgb c = status_led::colourAt({mode, false}, t);
            if (dark(c)) {
                darkSamples++;
            } else {
                TEST_ASSERT_TRUE(c == status_led::modeColour(mode));
                lit++;
            }
            if (!(c == previous)) changes++;
            previous = c;
        }
        TEST_ASSERT_EQUAL_INT(15, lit);
        TEST_ASSERT_EQUAL_INT(15, darkSamples);
        TEST_ASSERT_EQUAL_INT(5, changes); // on, off, on, off, on, off over three seconds
    }
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_every_mode_has_its_own_visible_colour);
    RUN_TEST(test_no_mode_is_shown_in_red);
    RUN_TEST(test_with_a_client_the_mode_colour_is_steady);
    RUN_TEST(test_without_a_client_the_mode_colour_blinks_once_a_second);
    return UNITY_END();
}
