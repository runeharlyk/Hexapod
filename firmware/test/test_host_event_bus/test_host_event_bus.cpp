// The host bus keeps its queue, subscribers and latest value for the whole test process, and a failed assert skips
// the destructors that would unsubscribe, so host_bus::resetAll() starts each test from an empty bus.

#include <unity.h>
#include <event_bus.h>

namespace {

struct Probe {
    int value;
};

using IntBus = EventBus<int, 4, 2>;
using ProbeBus = EventBus<Probe, 4, 2>;

} // namespace

void setUp() { host_bus::resetAll(); }
void tearDown() {}

void test_a_subscriber_from_before_a_reset_gets_nothing() {
    int count = 0;
    IntBus::Handle subscription = IntBus::subscribe([&count](const int&) { ++count; });
    host_bus::resetAll();
    IntBus::publish(1);
    host_bus::drainAll();
    TEST_ASSERT_EQUAL_INT(0, count);
}

void test_a_reset_forgets_the_queue_and_the_latest_value() {
    IntBus::publish(1);
    IntBus::publish(2);
    IntBus::store(7);
    host_bus::resetAll();
    int count = 0;
    IntBus::Handle subscription = IntBus::subscribe([&count](const int&) { ++count; });
    host_bus::drainAll();
    TEST_ASSERT_EQUAL_INT(0, count);
    int latest = 0;
    TEST_ASSERT_FALSE(IntBus::peek(latest));
}

void test_a_consumer_survives_a_reset() {
    static int count = 0;
    ProbeBus::consume([](const Probe&) { ++count; });
    host_bus::resetAll();
    ProbeBus::publish({1});
    host_bus::drainAll();
    TEST_ASSERT_EQUAL_INT(1, count);
}

void test_a_handle_from_before_a_reset_cannot_free_a_later_subscriber() {
    int aCount = 0;
    int bCount = 0;
    auto* old = new IntBus::Handle(IntBus::subscribe([&aCount](const int&) { ++aCount; }));
    host_bus::resetAll();
    IntBus::Handle b = IntBus::subscribe([&bCount](const int&) { ++bCount; });
    old->unsubscribe();
    IntBus::publish(1);
    host_bus::drainAll();
    TEST_ASSERT_EQUAL_INT(1, bCount);
    TEST_ASSERT_EQUAL_INT(0, aCount);
    TEST_ASSERT_FALSE(old->valid());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_a_subscriber_from_before_a_reset_gets_nothing);
    RUN_TEST(test_a_reset_forgets_the_queue_and_the_latest_value);
    RUN_TEST(test_a_consumer_survives_a_reset);
    RUN_TEST(test_a_handle_from_before_a_reset_cannot_free_a_later_subscriber);
    return UNITY_END();
}
