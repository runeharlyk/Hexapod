// Host tests of the robot-to-app BLE framing over a fake notify that can fail chosen chunks. The app reassembles a
// frame from its chunks by the length prefix, so a frame must arrive whole or end the link with it.

#include <unity.h>

#include <vector>
#include <communication/ble_frame.h>

namespace {

using Bytes = std::vector<uint8_t>;

// One BLE link as the app sees it: the notifications that reached it since it was opened.
struct FakeLink {
    std::vector<Bytes> received;
    size_t attempts = 0;
    size_t pauses = 0;
    size_t ends = 0;
    // Notification attempts, counted from 0 across the link's life, that find no mbuf.
    std::vector<size_t> failing;

    bool notify(const uint8_t* data, size_t len) {
        const size_t attempt = attempts++;
        for (size_t f : failing)
            if (f == attempt) return false;
        received.emplace_back(data, data + len);
        return true;
    }

    // The app opens a new link, with nothing of the old one's frames.
    void end() {
        ends++;
        received.clear();
    }

    bool send(const Bytes& frame) {
        return ble_frame::send(
            frame.data(), frame.size(), [this](const uint8_t* d, size_t n) { return notify(d, n); },
            [this] { pauses++; }, [this] { end(); });
    }

    Bytes stream() const {
        Bytes all;
        for (const Bytes& n : received) all.insert(all.end(), n.begin(), n.end());
        return all;
    }
};

Bytes payload(size_t len, uint8_t seed) {
    Bytes out(len);
    for (size_t i = 0; i < len; i++) out[i] = (uint8_t)(seed + i);
    return out;
}

Bytes framed(const Bytes& message) {
    Bytes out = {(uint8_t)(message.size() & 0xFF), (uint8_t)(message.size() >> 8)};
    out.insert(out.end(), message.begin(), message.end());
    return out;
}

void test_a_frame_goes_out_as_full_chunks_and_a_shorter_last() {
    FakeLink link;
    const Bytes message = payload(400, 1);
    TEST_ASSERT_TRUE(link.send(message));
    TEST_ASSERT_EQUAL(3, link.received.size());
    TEST_ASSERT_EQUAL(ble_frame::CHUNK, link.received[0].size());
    TEST_ASSERT_EQUAL(ble_frame::CHUNK, link.received[1].size());
    TEST_ASSERT_EQUAL(402 - 2 * ble_frame::CHUNK, link.received[2].size());
    TEST_ASSERT_TRUE(link.stream() == framed(message));
}

void test_a_later_chunk_without_an_mbuf_is_retried_and_the_frame_arrives_whole() {
    FakeLink link;
    link.failing = {1, 2};
    const Bytes message = payload(400, 7);
    TEST_ASSERT_TRUE(link.send(message));
    TEST_ASSERT_TRUE(link.stream() == framed(message));
    TEST_ASSERT_EQUAL(2, link.pauses);
    TEST_ASSERT_EQUAL(0, link.ends);
}

void test_a_later_chunk_that_never_goes_out_ends_the_link_and_leaves_no_partial_frame() {
    FakeLink link;
    for (size_t a = 1; a < 100; a++) link.failing.push_back(a);
    TEST_ASSERT_FALSE(link.send(payload(400, 3)));
    TEST_ASSERT_EQUAL(1, link.ends);
    TEST_ASSERT_TRUE(link.received.empty());
    TEST_ASSERT_EQUAL(2 + ble_frame::CHUNK_RETRIES, link.attempts);

    link.failing.clear();
    const Bytes next = payload(50, 9);
    TEST_ASSERT_TRUE(link.send(next));
    TEST_ASSERT_TRUE(link.stream() == framed(next));
}

void test_a_first_chunk_without_an_mbuf_loses_the_frame_whole_and_keeps_the_link() {
    FakeLink link;
    link.failing = {0};
    TEST_ASSERT_FALSE(link.send(payload(400, 5)));
    TEST_ASSERT_EQUAL(1, link.attempts);
    TEST_ASSERT_EQUAL(0, link.ends);
    TEST_ASSERT_TRUE(link.received.empty());
}

}  // namespace

void setUp() {}
void tearDown() {}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_a_frame_goes_out_as_full_chunks_and_a_shorter_last);
    RUN_TEST(test_a_later_chunk_without_an_mbuf_is_retried_and_the_frame_arrives_whole);
    RUN_TEST(test_a_later_chunk_that_never_goes_out_ends_the_link_and_leaves_no_partial_frame);
    RUN_TEST(test_a_first_chunk_without_an_mbuf_loses_the_frame_whole_and_keeps_the_link);
    return UNITY_END();
}
