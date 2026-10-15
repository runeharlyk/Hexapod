// Host test of the nanopb struct sizes. Every adapter keeps two Messages (its encoder's and its
// decoder's) and every correlation request builds its CorrelationResponse on the adapter task's
// stack, so a new max_size or max_count in the .options files lands in internal RAM several times
// over. The limits sit just above today's sizes, which are the same on the host and the ESP32-S3;
// raising one is a decision to be taken with the RAM budget in view, not a test to quieten.

#include <unity.h>

#include <platform_shared/message.pb.h>

void setUp() {}
void tearDown() {}

void test_the_protocol_structs_stay_small() {
    TEST_ASSERT_LESS_OR_EQUAL(2304, sizeof(socket_message_Message));
    TEST_ASSERT_LESS_OR_EQUAL(2304, sizeof(socket_message_CorrelationResponse));
    TEST_ASSERT_LESS_OR_EQUAL(768, sizeof(socket_message_CorrelationRequest));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_the_protocol_structs_stay_small);
    return UNITY_END();
}
