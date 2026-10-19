// Host tests of a subscription on CommAdapterBase (communication/comm_base.hpp), on a fake transport that decodes what
// it sends. A client subscribes to a tag with sub_notif; the robot pushes that tag's frames to every client on its
// list.

#include <unity.h>

#include <vector>
#include <pb_decode.h>
#include <pb_encode.h>
#include <communication/comm_base.hpp>

namespace {

struct Frame {
    int cid;
    socket_message_Message message;
};

// A transport whose clients can always take a frame; it keeps every frame it sent, decoded, with its client.
class FakeAdapter : public CommAdapterBase {
  public:
    std::vector<Frame> sent;
    bool hasClient() const override { return true; }

    void receive(const socket_message_Message& message, int cid) {
        uint8_t bytes[64];
        pb_ostream_t stream = pb_ostream_from_buffer(bytes, sizeof(bytes));
        TEST_ASSERT_TRUE(pb_encode(&stream, socket_message_Message_fields, &message));
        handleIncoming(bytes, stream.bytes_written, cid);
    }

    void subscribeTo(int32_t tag, int cid) {
        socket_message_Message message = socket_message_Message_init_zero;
        message.which_message = socket_message_Message_sub_notif_tag;
        message.message.sub_notif.tag = tag;
        receive(message, cid);
    }

    std::vector<Frame> to(int cid) const {
        std::vector<Frame> frames;
        for (const Frame& frame : sent)
            if (frame.cid == cid) frames.push_back(frame);
        return frames;
    }

  protected:
    void send(const uint8_t* data, size_t len, int cid) override {
        Frame frame {cid, socket_message_Message_init_zero};
        pb_istream_t stream = pb_istream_from_buffer(data, len);
        TEST_ASSERT_TRUE(pb_decode(&stream, socket_message_Message_fields, &frame.message));
        sent.push_back(frame);
    }
};

constexpr int32_t MODE_TAG = socket_message_Message_mode_tag;

socket_message_ModeData modeData(socket_message_ModesEnum mode) {
    socket_message_ModeData data = socket_message_ModeData_init_zero;
    data.mode = mode;
    return data;
}

} // namespace

void setUp() {}
void tearDown() {}

// The app resubscribes every tag on a reconnect and on a fail-over between transports, so the robot hears a subscribe
// from a client that already has it; a second entry on the list would send that client every push twice.
void test_a_repeated_subscribe_from_one_client_gets_each_push_once() {
    FakeAdapter adapter;
    adapter.subscribeTo(MODE_TAG, 3);
    adapter.subscribeTo(MODE_TAG, 3);
    adapter.sent.clear();

    adapter.emit(modeData(socket_message_ModesEnum_STAND));

    TEST_ASSERT_EQUAL(1, adapter.to(3).size());
    TEST_ASSERT_EQUAL(socket_message_ModesEnum_STAND, adapter.to(3)[0].message.message.mode.mode);
}

// One unsubscribe ends the subscription, however often it was asked for.
void test_one_unsubscribe_ends_a_repeated_subscription() {
    FakeAdapter adapter;
    adapter.subscribeTo(MODE_TAG, 3);
    adapter.subscribeTo(MODE_TAG, 3);
    socket_message_Message unsubscribe = socket_message_Message_init_zero;
    unsubscribe.which_message = socket_message_Message_unsub_notif_tag;
    unsubscribe.message.unsub_notif.tag = MODE_TAG;
    adapter.receive(unsubscribe, 3);
    adapter.sent.clear();

    adapter.emit(modeData(socket_message_ModesEnum_STAND));

    TEST_ASSERT_FALSE(adapter.hasSubscribers(MODE_TAG));
    TEST_ASSERT_EQUAL(0, adapter.sent.size());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_a_repeated_subscribe_from_one_client_gets_each_push_once);
    RUN_TEST(test_one_unsubscribe_ends_a_repeated_subscription);
    return UNITY_END();
}
