// Host tests of the I2C bus's error handling: telling a NACK from a timeout when the driver reports both
// alike, the per-device counts, how often a held bus is clocked free, and the bus clear itself against
// a modelled device on open-drain lines.

#include <unity.h>

#include <peripherals/i2c_health.h>

using i2c_health::Failure;

namespace {

constexpr int TIMEOUT_MS = 20;
constexpr int TICK_MS = 1;

// Two open-drain lines with pull-ups, the master's drivers, and a device that was cut off mid-read: it
// holds SDA low until SCL has risen `heldForPulses` more times, or for good when that is negative. It
// counts the master's SCL pulses and the START and STOP conditions it puts on the bus.
struct OpenDrainBus {
    bool masterSda = true;
    bool masterScl = true;
    int heldForPulses = 0;
    int pulses = 0;
    int starts = 0;
    int stops = 0;

    bool deviceHoldsSda() const { return heldForPulses != 0; }
    bool sdaLevel() const { return masterSda && !deviceHoldsSda(); }
    bool sclLevel() const { return masterScl; }

    void sda(bool high) {
        const bool before = sdaLevel();
        masterSda = high;
        const bool after = sdaLevel();
        if (sclLevel() && before && !after) starts++;
        if (sclLevel() && !before && after) stops++;
    }
    void scl(bool high) {
        const bool rose = !sclLevel() && high;
        masterScl = high;
        if (!rose) return;
        pulses++;
        if (heldForPulses > 0) heldForPulses--;
    }
    bool sdaLow() const { return !sdaLevel(); }
    void halfPeriod() {}
};

}  // namespace

void setUp() {}
void tearDown() {}

// A servo board without power does not acknowledge on a free bus: the longest write here, all 16
// channels at 100 kHz, fails after about 4.5 ms.
void test_a_device_that_does_not_acknowledge_on_a_free_bus_is_a_nack() {
    TEST_ASSERT(i2c_health::classify(false, 4500, TIMEOUT_MS, TICK_MS, false, false) == Failure::NACK);
    TEST_ASSERT(i2c_health::classify(false, 18999, TIMEOUT_MS, TICK_MS, false, false) == Failure::NACK);
}

// The driver waits whole ticks, so a failure that ends one tick short of the timeout spent it.
void test_a_failure_that_spent_the_timeout_is_a_timeout() {
    TEST_ASSERT(i2c_health::classify(false, 19000, TIMEOUT_MS, TICK_MS, false, false) == Failure::TIMEOUT);
    TEST_ASSERT(i2c_health::classify(false, 20000, TIMEOUT_MS, TICK_MS, false, false) == Failure::TIMEOUT);
    // With a 10 ms tick the same 20 ms timeout can end after 10.
    TEST_ASSERT(i2c_health::classify(false, 10000, TIMEOUT_MS, 10, false, false) == Failure::TIMEOUT);
}

void test_a_line_held_low_after_a_fast_failure_is_a_timeout() {
    TEST_ASSERT(i2c_health::classify(false, 100, TIMEOUT_MS, TICK_MS, true, false) == Failure::TIMEOUT);
    TEST_ASSERT(i2c_health::classify(false, 100, TIMEOUT_MS, TICK_MS, false, true) == Failure::TIMEOUT);
    TEST_ASSERT(i2c_health::classify(true, 100, TIMEOUT_MS, TICK_MS, false, false) == Failure::TIMEOUT);
}

void test_devices_are_listed_from_their_first_transfer_in_address_order() {
    i2c_health::ErrorTable<8> table;
    table.track(0x68);
    table.track(0x40);
    table.count(0x41, Failure::NACK);
    table.track(0x40);
    table.count(0x1E, Failure::TIMEOUT);

    TEST_ASSERT_EQUAL_UINT32(4, table.size());
    const uint8_t order[] = {0x1E, 0x40, 0x41, 0x68};
    for (size_t i = 0; i < 4; i++) TEST_ASSERT_EQUAL_HEX8(order[i], table[i].address);
    TEST_ASSERT_EQUAL_UINT32(0, table[1].nacks + table[1].timeouts);
    TEST_ASSERT_EQUAL_UINT32(0, table[3].nacks + table[3].timeouts);
}

void test_failures_are_counted_per_device_and_kind() {
    i2c_health::ErrorTable<8> table;
    for (int i = 0; i < 5; i++) table.count(0x41, Failure::NACK);
    table.count(0x68, Failure::TIMEOUT);
    table.count(0x41, Failure::TIMEOUT);

    TEST_ASSERT_EQUAL_HEX8(0x41, table[0].address);
    TEST_ASSERT_EQUAL_UINT32(5, table[0].nacks);
    TEST_ASSERT_EQUAL_UINT32(1, table[0].timeouts);
    TEST_ASSERT_EQUAL_HEX8(0x68, table[1].address);
    TEST_ASSERT_EQUAL_UINT32(0, table[1].nacks);
    TEST_ASSERT_EQUAL_UINT32(1, table[1].timeouts);
}

// A full table drops a new address rather than overwrite a listed device's counts.
void test_a_full_table_keeps_the_devices_it_lists() {
    i2c_health::ErrorTable<2> table;
    table.count(0x40, Failure::NACK);
    table.count(0x68, Failure::NACK);
    table.count(0x10, Failure::NACK);
    table.count(0x50, Failure::TIMEOUT);

    TEST_ASSERT_EQUAL_UINT32(2, table.size());
    TEST_ASSERT_EQUAL_HEX8(0x40, table[0].address);
    TEST_ASSERT_EQUAL_HEX8(0x68, table[1].address);
    TEST_ASSERT_EQUAL_UINT32(1, table[0].nacks);
    TEST_ASSERT_EQUAL_UINT32(1, table[1].nacks);
    TEST_ASSERT_EQUAL_UINT32(0, table[0].timeouts + table[1].timeouts);
}

// A device that never lets go fails every transfer: 150 of them 10 ms apart get two recoveries, not 150.
void test_a_held_sda_is_recovered_at_most_once_a_second() {
    i2c_health::RecoveryLimiter limiter;
    int64_t now = 5000000;
    int recoveries = 0;
    for (int i = 0; i < 150; i++, now += 10000) recoveries += limiter.due(true, false, now);
    TEST_ASSERT_EQUAL_INT(2, recoveries);
    TEST_ASSERT_EQUAL_UINT32(2, limiter.count());
}

// SCL held is clock stretching or an unpowered device clamping the lines, and a free SDA is a NACK:
// pulses free neither.
void test_only_a_held_sda_on_a_free_clock_is_recovered() {
    i2c_health::RecoveryLimiter limiter;
    TEST_ASSERT_FALSE(limiter.due(true, true, 5000000));
    TEST_ASSERT_FALSE(limiter.due(false, true, 5000000));
    TEST_ASSERT_FALSE(limiter.due(false, false, 5000000));
    TEST_ASSERT_EQUAL_UINT32(0, limiter.count());
    TEST_ASSERT_TRUE(limiter.due(true, false, 5000000));
}

// The ESP32 reset while the IMU was sending a byte of zeros with four bits left: four pulses free it,
// and a STOP ends the transfer it was in.
void test_a_device_holding_sda_is_clocked_free_and_stopped() {
    OpenDrainBus bus;
    bus.heldForPulses = 4;
    TEST_ASSERT_EQUAL_INT(4, i2c_health::clockOut(bus));
    TEST_ASSERT_FALSE(bus.deviceHoldsSda());
    TEST_ASSERT_EQUAL_INT(1, bus.stops);
    TEST_ASSERT_EQUAL_INT(0, bus.starts);
    TEST_ASSERT_TRUE(bus.sdaLevel());
    TEST_ASSERT_TRUE(bus.sclLevel());
}

// Nine pulses outlast any byte; a device still holding SDA after them is left for the next recovery.
void test_a_device_that_never_lets_go_gets_nine_pulses() {
    OpenDrainBus bus;
    bus.heldForPulses = -1;
    TEST_ASSERT_EQUAL_INT(9, i2c_health::clockOut(bus));
    TEST_ASSERT_EQUAL_INT(0, bus.starts);
    TEST_ASSERT_TRUE(bus.sclLevel());
}

void test_a_free_bus_gets_only_a_stop() {
    OpenDrainBus bus;
    TEST_ASSERT_EQUAL_INT(0, i2c_health::clockOut(bus));
    TEST_ASSERT_EQUAL_INT(1, bus.pulses);  // the STOP's own clock rise
    TEST_ASSERT_EQUAL_INT(1, bus.stops);
    TEST_ASSERT_EQUAL_INT(0, bus.starts);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_a_device_that_does_not_acknowledge_on_a_free_bus_is_a_nack);
    RUN_TEST(test_a_failure_that_spent_the_timeout_is_a_timeout);
    RUN_TEST(test_a_line_held_low_after_a_fast_failure_is_a_timeout);
    RUN_TEST(test_devices_are_listed_from_their_first_transfer_in_address_order);
    RUN_TEST(test_failures_are_counted_per_device_and_kind);
    RUN_TEST(test_a_full_table_keeps_the_devices_it_lists);
    RUN_TEST(test_a_held_sda_is_recovered_at_most_once_a_second);
    RUN_TEST(test_only_a_held_sda_on_a_free_clock_is_recovered);
    RUN_TEST(test_a_device_holding_sda_is_clocked_free_and_stopped);
    RUN_TEST(test_a_device_that_never_lets_go_gets_nine_pulses);
    RUN_TEST(test_a_free_bus_gets_only_a_stop);
    return UNITY_END();
}
