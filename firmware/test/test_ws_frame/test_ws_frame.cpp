// Host tests of the WebSocket framing the server does itself: the binary frame header it writes before
// a payload, and the answers it owes to control frames (RFC 6455 sections 5.2 and 5.5).

#include <unity.h>

#include <vector>
#include <communication/ws_frame.h>

namespace {

std::vector<uint8_t> header(size_t length) {
    uint8_t out[ws_frame::MAX_HEADER];
    return std::vector<uint8_t>(out, out + ws_frame::binaryHeader(out, length));
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_short_payloads_carry_their_length_in_the_second_byte() {
    TEST_ASSERT_TRUE(header(0) == std::vector<uint8_t>({0x82, 0}));
    TEST_ASSERT_TRUE(header(125) == std::vector<uint8_t>({0x82, 125}));
}

void test_medium_payloads_use_a_sixteen_bit_length() {
    TEST_ASSERT_TRUE(header(126) == std::vector<uint8_t>({0x82, 126, 0x00, 0x7e}));
    TEST_ASSERT_TRUE(header(65535) == std::vector<uint8_t>({0x82, 126, 0xff, 0xff}));
}

void test_large_payloads_use_a_sixty_four_bit_length() {
    TEST_ASSERT_TRUE(header(65536) == std::vector<uint8_t>({0x82, 127, 0, 0, 0, 0, 0x00, 0x01, 0x00, 0x00}));
}

// A close is answered with a close carrying only its status code, and the session ends.
void test_a_close_is_echoed_with_its_status_code() {
    const uint8_t normal[] = {0x03, 0xe8, 'b', 'y', 'e'};
    ws_frame::Reply reply = ws_frame::controlReply(ws_frame::CLOSE, normal, sizeof(normal));
    TEST_ASSERT_EQUAL(ws_frame::CLOSE, reply.opcode);
    TEST_ASSERT_EQUAL_PTR(normal, reply.payload);
    TEST_ASSERT_EQUAL(2, reply.len);
    TEST_ASSERT_TRUE(reply.endsSession);

    reply = ws_frame::controlReply(ws_frame::CLOSE, nullptr, 0);
    TEST_ASSERT_EQUAL(ws_frame::CLOSE, reply.opcode);
    TEST_ASSERT_EQUAL(0, reply.len);
    TEST_ASSERT_TRUE(reply.endsSession);
}

void test_a_ping_is_answered_with_a_pong_carrying_its_payload() {
    const uint8_t data[] = {1, 2, 3};
    const ws_frame::Reply reply = ws_frame::controlReply(ws_frame::PING, data, sizeof(data));
    TEST_ASSERT_EQUAL(ws_frame::PONG, reply.opcode);
    TEST_ASSERT_EQUAL_PTR(data, reply.payload);
    TEST_ASSERT_EQUAL(3, reply.len);
    TEST_ASSERT_FALSE(reply.endsSession);
}

void test_pongs_and_data_frames_get_no_answer() {
    const uint8_t data[] = {1};
    for (ws_frame::Opcode opcode : {ws_frame::PONG, ws_frame::BINARY}) {
        const ws_frame::Reply reply = ws_frame::controlReply(opcode, data, sizeof(data));
        TEST_ASSERT_EQUAL(ws_frame::NONE, reply.opcode);
        TEST_ASSERT_FALSE(reply.endsSession);
    }
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_short_payloads_carry_their_length_in_the_second_byte);
    RUN_TEST(test_medium_payloads_use_a_sixteen_bit_length);
    RUN_TEST(test_large_payloads_use_a_sixty_four_bit_length);
    RUN_TEST(test_a_close_is_echoed_with_its_status_code);
    RUN_TEST(test_a_ping_is_answered_with_a_pong_carrying_its_payload);
    RUN_TEST(test_pongs_and_data_frames_get_no_answer);
    return UNITY_END();
}
