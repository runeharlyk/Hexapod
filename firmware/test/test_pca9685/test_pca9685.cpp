// Host tests of the PCA9685 start-up against the register fake in test/fakes. The chip keeps its
// registers across an ESP reset, so start-up is where a stale pose would reach the legs.

#include <unity.h>

#include <peripherals/drivers/pca9685.h>

namespace {

constexpr uint8_t ADDR = 0x40;
constexpr uint8_t MODE1 = 0x00;
constexpr uint8_t MODE1_RESTART = 0x80;
constexpr uint8_t MODE1_SLEEP = 0x10;
constexpr uint8_t MODE1_AI = 0x20;
constexpr uint8_t LED0_OFF_H = 0x09;
constexpr uint8_t ALL_LED_OFF_H = 0xFD;
constexpr uint8_t FULL_OFF = 0x10;
constexpr uint8_t PRE_SCALE = 0xFE;

// A chip that slept holding the last pose, as it does after a deactivate and an ESP reset: RESTART
// pending, asleep, auto-increment, and a pulse still in channel 0.
void startWithAStaleSleepingChip() {
    I2CBus& bus = I2CBus::instance();
    bus.reset();
    bus.registers[ADDR] = {};
    bus.registers[ADDR][MODE1] = MODE1_RESTART | MODE1_SLEEP | MODE1_AI;
    bus.registers[ADDR][LED0_OFF_H] = 0x01;
}

}  // namespace

void setUp() { startWithAStaleSleepingChip(); }
void tearDown() {}

void test_start_up_never_wakes_the_chip() {
    PCA9685Driver pca(ADDR);
    TEST_ASSERT_TRUE(pca.begin(25000000, 50));

    int mode1Writes = 0;
    for (const FakeWrite& w : I2CBus::instance().writes) {
        if (w.reg != MODE1) continue;
        mode1Writes++;
        TEST_ASSERT_TRUE_MESSAGE(w.bytes[0] & MODE1_SLEEP, "a MODE1 write during start-up woke the chip");
    }
    TEST_ASSERT_GREATER_THAN_INT(0, mode1Writes);
}

void test_every_channel_is_off_before_the_chip_is_touched() {
    PCA9685Driver pca(ADDR);
    TEST_ASSERT_TRUE(pca.begin(25000000, 50));

    const auto& writes = I2CBus::instance().writes;
    TEST_ASSERT_FALSE(writes.empty());
    TEST_ASSERT_EQUAL_UINT8(ALL_LED_OFF_H, writes.front().reg);
    TEST_ASSERT_TRUE(writes.front().bytes[0] & FULL_OFF);
}

void test_the_prescale_comes_from_the_given_oscillator() {
    PCA9685Driver pca(ADDR);
    TEST_ASSERT_TRUE(pca.begin(25000000, 50));
    // round(25 MHz / (4096 * 50 Hz)) - 1
    TEST_ASSERT_EQUAL_UINT8(121, I2CBus::instance().registers[ADDR][PRE_SCALE]);

    startWithAStaleSleepingChip();
    TEST_ASSERT_TRUE(pca.begin(27000000, 50));
    TEST_ASSERT_EQUAL_UINT8(131, I2CBus::instance().registers[ADDR][PRE_SCALE]);
}

void test_an_absent_chip_is_left_alone() {
    PCA9685Driver pca(0x41);
    TEST_ASSERT_FALSE(pca.begin(25000000, 50));
    TEST_ASSERT_FALSE(pca.isInitialized());
    TEST_ASSERT_TRUE(I2CBus::instance().writes.empty());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_start_up_never_wakes_the_chip);
    RUN_TEST(test_every_channel_is_off_before_the_chip_is_touched);
    RUN_TEST(test_the_prescale_comes_from_the_given_oscillator);
    RUN_TEST(test_an_absent_chip_is_left_alone);
    return UNITY_END();
}
