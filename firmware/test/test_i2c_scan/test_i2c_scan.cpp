// Host tests of the I2C scan against the register fake in test/fakes: the requester posts a sweep and waits, and the
// control task probes a few addresses per tick. The fake bypasses the IDF driver, so the cross-core race the sweep
// avoids by running on the control task cannot show here; what can is the set reported, the range, the cost per
// tick, and the refusals.

#include <unity.h>

#include <set>
#include <vector>
#include <peripherals/i2c_scan.h>

namespace {

using Outcome = I2CScan::Outcome;

void plug(std::initializer_list<uint8_t> addresses) {
    for (uint8_t a : addresses) I2CBus::instance().registers[a] = {};
}

std::vector<uint8_t> reported(const I2CScan::Found& found) {
    return std::vector<uint8_t>(found.addresses, found.addresses + found.count);
}

// The control task's ticks while the requester waits: each wait is one tick, and each tick one step.
struct Ticks {
    I2CScan& scan;
    int ticks = 0;
    std::vector<size_t> probesPerTick;

    void operator()() {
        const size_t before = I2CBus::instance().probed.size();
        scan.step();
        probesPerTick.push_back(I2CBus::instance().probed.size() - before);
        ticks++;
    }
};

} // namespace

void setUp() { I2CBus::instance().reset(); }
void tearDown() {}

void test_the_scan_reports_exactly_the_devices_that_answer() {
    plug({0x1E, 0x40, 0x41, 0x68});
    I2CScan scan;
    Ticks ticks {scan};
    I2CScan::Found found;

    TEST_ASSERT(scan.request(found, ticks, 100) == Outcome::DONE);
    TEST_ASSERT(reported(found) == (std::vector<uint8_t> {0x1E, 0x40, 0x41, 0x68}));
}

// The reply was cut at 16 without a word, so 16 false hits below 0x1E hid every real device.
void test_more_than_sixteen_devices_are_all_reported() {
    std::vector<uint8_t> expected;
    for (uint8_t a = 0x10; a < 0x10 + 20; a++) {
        plug({a});
        expected.push_back(a);
    }
    plug({0x68});
    expected.push_back(0x68);
    I2CScan scan;
    Ticks ticks {scan};
    I2CScan::Found found;

    TEST_ASSERT(scan.request(found, ticks, 100) == Outcome::DONE);
    TEST_ASSERT_EQUAL(21, found.count);
    TEST_ASSERT(reported(found) == expected);
}

void test_every_address_answering_fills_the_whole_range() {
    for (int a = 0; a < 128; a++) plug({static_cast<uint8_t>(a)});
    I2CScan scan;
    Ticks ticks {scan};
    I2CScan::Found found;

    TEST_ASSERT(scan.request(found, ticks, 100) == Outcome::DONE);
    TEST_ASSERT_EQUAL(112, found.count);
    TEST_ASSERT_EQUAL_HEX8(0x08, found.addresses[0]);
    TEST_ASSERT_EQUAL_HEX8(0x77, found.addresses[111]);
}

// 0x00-0x07 and 0x78-0x7F are reserved: probed once each, in order, from 0x08 to 0x77 and nothing else.
void test_the_sweep_probes_the_unreserved_range_once_each() {
    plug({0x00, 0x03, 0x07, 0x08, 0x77, 0x78, 0x7F});
    I2CScan scan;
    Ticks ticks {scan};
    I2CScan::Found found;

    TEST_ASSERT(scan.request(found, ticks, 100) == Outcome::DONE);
    std::vector<uint8_t> range;
    for (int a = 0x08; a <= 0x77; a++) range.push_back(static_cast<uint8_t>(a));
    TEST_ASSERT(I2CBus::instance().probed == range);
    TEST_ASSERT(reported(found) == (std::vector<uint8_t> {0x08, 0x77}));
}

// A tick spends at most eight probes on the sweep, about 1 ms of its 5 ms, so the sweep takes 14 ticks.
void test_a_tick_probes_at_most_eight_addresses() {
    I2CScan scan;
    Ticks ticks {scan};
    I2CScan::Found found;

    TEST_ASSERT(scan.request(found, ticks, 100) == Outcome::DONE);
    TEST_ASSERT_EQUAL(14, ticks.ticks);
    for (size_t probes : ticks.probesPerTick) TEST_ASSERT_LESS_OR_EQUAL(8, probes);
}

void test_a_tick_without_a_request_probes_nothing() {
    I2CScan scan;
    for (int i = 0; i < 20; i++) scan.step();
    TEST_ASSERT_EQUAL(0, I2CBus::instance().probed.size());
}

// A bus another task holds is not an empty bus: the scan fails instead of reporting nothing there.
void test_a_busy_bus_fails_the_scan_instead_of_reporting_no_devices() {
    plug({0x40});
    I2CBus::instance().busy = true;
    I2CScan scan;
    Ticks ticks {scan};
    I2CScan::Found found;

    TEST_ASSERT(scan.request(found, ticks, 100) == Outcome::BUS_BUSY);
    TEST_ASSERT_EQUAL(0, found.count);
    TEST_ASSERT_EQUAL(1, ticks.ticks);

    I2CBus::instance().busy = false;
    TEST_ASSERT(scan.request(found, ticks, 100) == Outcome::DONE);
    TEST_ASSERT(reported(found) == (std::vector<uint8_t> {0x40}));
}

// A bus that is down, or a probe the driver times out, is not reported as a busy bus, which would send the user looking
// for another task that holds it.
void test_a_faulty_bus_fails_the_scan_as_a_fault_rather_than_as_busy() {
    plug({0x40});
    I2CBus::instance().faulty = true;
    I2CScan scan;
    Ticks ticks {scan};
    I2CScan::Found found;

    TEST_ASSERT(scan.request(found, ticks, 100) == Outcome::BUS_FAULT);
    TEST_ASSERT_EQUAL(0, found.count);
    TEST_ASSERT_EQUAL(1, ticks.ticks);

    I2CBus::instance().faulty = false;
    TEST_ASSERT(scan.request(found, ticks, 100) == Outcome::DONE);
    TEST_ASSERT(reported(found) == (std::vector<uint8_t> {0x40}));
}

void test_a_bus_that_turns_busy_mid_sweep_fails_the_scan() {
    plug({0x10, 0x60});
    I2CScan scan;
    I2CScan::Found found;
    int tick = 0;
    auto busyFromTheThirdTick = [&] {
        if (++tick == 3) I2CBus::instance().busy = true;
        scan.step();
    };

    TEST_ASSERT(scan.request(found, busyFromTheThirdTick, 100) == Outcome::BUS_BUSY);
    TEST_ASSERT_EQUAL(0, found.count);
}

// Two transports can ask at once; the second is refused while the first sweep runs, and its request posts nothing.
void test_a_second_request_while_one_runs_is_refused() {
    plug({0x40});
    I2CScan scan;
    I2CScan::Found found, second;
    Outcome secondOutcome = Outcome::DONE;
    int secondWaits = 0;
    bool asked = false;
    auto tickAndAskAgain = [&] {
        if (!asked) {
            asked = true;
            secondOutcome = scan.request(second, [&] { secondWaits++; }, 100);
        }
        scan.step();
    };

    TEST_ASSERT(scan.request(found, tickAndAskAgain, 100) == Outcome::DONE);
    TEST_ASSERT(secondOutcome == Outcome::ALREADY_RUNNING);
    TEST_ASSERT_EQUAL(0, secondWaits);
    TEST_ASSERT(reported(found) == (std::vector<uint8_t> {0x40}));
    TEST_ASSERT_EQUAL(112, I2CBus::instance().probed.size());
}

// A control task that does not sweep, stalled or not yet started, costs the requester its bounded wait, not more; the
// abandoned sweep is dropped at the control task's next tick, and the scan works again after it.
void test_a_sweep_that_does_not_end_in_time_is_abandoned_and_dropped() {
    plug({0x40});
    I2CScan scan;
    I2CScan::Found found;
    int waits = 0;

    TEST_ASSERT(scan.request(found, [&] { waits++; }, 5) == Outcome::TIMED_OUT);
    TEST_ASSERT_EQUAL(5, waits);
    TEST_ASSERT_EQUAL(0, found.count);
    TEST_ASSERT(scan.request(found, [] {}, 5) == Outcome::ALREADY_RUNNING);

    scan.step();
    TEST_ASSERT_EQUAL(0, I2CBus::instance().probed.size());
    Ticks ticks {scan};
    TEST_ASSERT(scan.request(found, ticks, 100) == Outcome::DONE);
    TEST_ASSERT(reported(found) == (std::vector<uint8_t> {0x40}));
}

void test_a_sweep_abandoned_midway_reports_nothing_and_frees_the_scan() {
    plug({0x09, 0x40});
    I2CScan scan;
    I2CScan::Found found;
    auto twoTicks = [&] { scan.step(); };

    TEST_ASSERT(scan.request(found, twoTicks, 2) == Outcome::TIMED_OUT);
    TEST_ASSERT_EQUAL(0, found.count);
    scan.step();
    const size_t probedBefore = I2CBus::instance().probed.size();
    scan.step();
    TEST_ASSERT_EQUAL(probedBefore, I2CBus::instance().probed.size());

    Ticks ticks {scan};
    TEST_ASSERT(scan.request(found, ticks, 100) == Outcome::DONE);
    TEST_ASSERT(reported(found) == (std::vector<uint8_t> {0x09, 0x40}));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_the_scan_reports_exactly_the_devices_that_answer);
    RUN_TEST(test_more_than_sixteen_devices_are_all_reported);
    RUN_TEST(test_every_address_answering_fills_the_whole_range);
    RUN_TEST(test_the_sweep_probes_the_unreserved_range_once_each);
    RUN_TEST(test_a_tick_probes_at_most_eight_addresses);
    RUN_TEST(test_a_tick_without_a_request_probes_nothing);
    RUN_TEST(test_a_busy_bus_fails_the_scan_instead_of_reporting_no_devices);
    RUN_TEST(test_a_faulty_bus_fails_the_scan_as_a_fault_rather_than_as_busy);
    RUN_TEST(test_a_bus_that_turns_busy_mid_sweep_fails_the_scan);
    RUN_TEST(test_a_second_request_while_one_runs_is_refused);
    RUN_TEST(test_a_sweep_that_does_not_end_in_time_is_abandoned_and_dropped);
    RUN_TEST(test_a_sweep_abandoned_midway_reports_nothing_and_frees_the_scan);
    return UNITY_END();
}
