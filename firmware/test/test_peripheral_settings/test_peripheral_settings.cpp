// Host tests of the I2C bus settings check, which runs both on an update from the app and on the
// settings file read at boot.

#include <unity.h>

#include <settings/peripherals_settings.h>

namespace {

PeripheralSettings withFrequency(int32_t frequency) {
    PeripheralSettings settings = PeripheralSettings_defaults();
    settings.frequency = frequency;
    return settings;
}

}  // namespace

void setUp() {}
void tearDown() {}

// The MPU6050 and HMC5883L share the bus and are rated for 400 kHz; earlier firmware accepted up to
// 1 MHz, so robots may have that stored.
void test_the_bus_runs_no_faster_than_its_slowest_devices() {
    PeripheralSettings stored = withFrequency(100000);
    stored.sda = 4;
    PeripheralSettings fast = withFrequency(1000000);
    fast.sda = 4;
    TEST_ASSERT_EQUAL(StateUpdateResult::CHANGED, PeripheralSettings_update(fast, stored));
    TEST_ASSERT_EQUAL_INT32(400000, stored.frequency);
    TEST_ASSERT_EQUAL_INT32(4, stored.sda);
    TEST_ASSERT_EQUAL(StateUpdateResult::UNCHANGED, PeripheralSettings_update(fast, stored));
}

void test_a_clock_within_the_rating_is_kept() {
    PeripheralSettings stored = withFrequency(400000);
    TEST_ASSERT_EQUAL(StateUpdateResult::CHANGED, PeripheralSettings_update(withFrequency(100000), stored));
    TEST_ASSERT_EQUAL_INT32(100000, stored.frequency);
    TEST_ASSERT_EQUAL(StateUpdateResult::CHANGED, PeripheralSettings_update(withFrequency(400000), stored));
    TEST_ASSERT_EQUAL_INT32(400000, stored.frequency);
}

void test_a_clock_outside_the_i2c_range_is_refused() {
    PeripheralSettings stored = withFrequency(400000);
    TEST_ASSERT_EQUAL(StateUpdateResult::ERROR, PeripheralSettings_update(withFrequency(99999), stored));
    TEST_ASSERT_EQUAL(StateUpdateResult::ERROR, PeripheralSettings_update(withFrequency(1000001), stored));
    TEST_ASSERT_EQUAL_INT32(400000, stored.frequency);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_the_bus_runs_no_faster_than_its_slowest_devices);
    RUN_TEST(test_a_clock_within_the_rating_is_kept);
    RUN_TEST(test_a_clock_outside_the_i2c_range_is_refused);
    return UNITY_END();
}
