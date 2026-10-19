// Host tests of a subscription on CommAdapterBase (communication/comm_base.hpp), on a fake transport that decodes what
// it sends. A client subscribes to a tag with sub_notif. A state topic answers with its owner's current value (the
// snapshot), then with the sub_notif echoed as the ack, and pushes every change after; a stream answers with the ack
// alone.

#include <unity.h>

#include <atomic>
#include <chrono>
#include <fstream>
#include <functional>
#include <future>
#include <iterator>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <pb_decode.h>
#include <pb_encode.h>
#include <communication/comm_base.hpp>
#include <communication/topics.h>

namespace {

struct Frame {
    int cid;
    socket_message_Message message;
};

constexpr int32_t MODE_TAG = socket_message_Message_mode_tag;
constexpr int32_t GAIT_TAG = socket_message_Message_gait_tag;
constexpr int32_t IMU_TAG = socket_message_Message_imu_tag;

socket_message_ModeData modeData(socket_message_ModesEnum mode) {
    socket_message_ModeData data = socket_message_ModeData_init_zero;
    data.mode = mode;
    return data;
}

// The mode's owner, as MotionInbox::appliedMode is: read by the snapshot, changed by another task.
std::atomic<socket_message_ModesEnum> ownerMode {socket_message_ModesEnum_DEACTIVATED};
// Runs between the snapshot's read of the owner and its put, where another task may change the owner.
std::function<void()> afterModeRead;
// What happened, in order, for the tests that ask.
std::vector<std::string> events;

void modeSnapshot(SnapshotSink& sink) {
    const socket_message_ModeData current = modeData(ownerMode.load());
    events.push_back("read");
    if (afterModeRead) afterModeRead();
    sink.put(current);
}

void gaitSnapshot(SnapshotSink& sink) {
    socket_message_GaitData current = socket_message_GaitData_init_zero;
    current.gait = socket_message_GaitEnum_WAVE;
    sink.put(current);
}

constexpr Topic TOPICS[] = {
    {IMU_TAG, TopicKind::STREAM, nullptr},
    {MODE_TAG, TopicKind::STATE, modeSnapshot},
    {GAIT_TAG, TopicKind::STATE, gaitSnapshot},
};

// A transport whose clients can take a frame while `writable`; it keeps every frame it sent, decoded, with its client.
// While `sendsFail`, a client looks ready but every send fails, as a BLE notify does without an mbuf.
class FakeAdapter : public CommAdapterBase {
  public:
    FakeAdapter() { setTopics(TOPICS); }

    std::atomic<bool> writable {true};
    std::atomic<bool> sendsFail {false};
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

    std::vector<Frame> to(int cid) {
        std::lock_guard<std::mutex> lock(sentMutex_);
        std::vector<Frame> frames;
        for (const Frame& frame : sent_)
            if (frame.cid == cid) frames.push_back(frame);
        return frames;
    }

    size_t sentCount() {
        std::lock_guard<std::mutex> lock(sentMutex_);
        return sent_.size();
    }

    void forget() {
        std::lock_guard<std::mutex> lock(sentMutex_);
        sent_.clear();
    }

  protected:
    bool ready(int) override { return writable; }
    bool send(const uint8_t* data, size_t len, int cid) override {
        if (sendsFail) return false;
        Frame frame {cid, socket_message_Message_init_zero};
        pb_istream_t stream = pb_istream_from_buffer(data, len);
        TEST_ASSERT_TRUE(pb_decode(&stream, socket_message_Message_fields, &frame.message));
        std::lock_guard<std::mutex> lock(sentMutex_);
        sent_.push_back(frame);
        return true;
    }

  private:
    std::mutex sentMutex_;
    std::vector<Frame> sent_;
};

// A frame as a line: "mode 2", "ack 130".
std::string describe(const Frame& frame) {
    const socket_message_Message& m = frame.message;
    switch (m.which_message) {
        case socket_message_Message_sub_notif_tag: return "ack " + std::to_string(m.message.sub_notif.tag);
        case socket_message_Message_mode_tag: return "mode " + std::to_string((int)m.message.mode.mode);
        case socket_message_Message_gait_tag: return "gait " + std::to_string((int)m.message.gait.gait);
        default: return "tag " + std::to_string((int)m.which_message);
    }
}

void assertFrames(const std::vector<std::string>& expected, const std::vector<Frame>& frames) {
    std::string actual;
    for (const Frame& frame : frames) actual += describe(frame) + "; ";
    std::string wanted;
    for (const std::string& line : expected) wanted += line + "; ";
    TEST_ASSERT_EQUAL_STRING(wanted.c_str(), actual.c_str());
}

const std::string STAND = "mode " + std::to_string((int)socket_message_ModesEnum_STAND);
const std::string WALK = "mode " + std::to_string((int)socket_message_ModesEnum_WALK);
const std::string DEACTIVATED = "mode " + std::to_string((int)socket_message_ModesEnum_DEACTIVATED);
const std::string MODE_ACK = "ack " + std::to_string(MODE_TAG);

// platform_shared/topics.json, the table the app's is checked against too: each entry's tag and kind, in file order.
std::vector<TopicKindEntry> sharedTopics() {
    std::ifstream file("platform_shared/topics.json");
    TEST_ASSERT_TRUE_MESSAGE(file.good(), "platform_shared/topics.json not found; run from the repository root");
    std::stringstream text;
    text << file.rdbuf();
    const std::string json = text.str();
    std::vector<TopicKindEntry> topics;
    for (size_t at = json.find("\"tag\":"); at != std::string::npos; at = json.find("\"tag\":", at + 1)) {
        const int32_t tag = std::stoi(json.substr(at + 6));
        const size_t kindAt = json.find("\"kind\":", at);
        TEST_ASSERT_TRUE(kindAt != std::string::npos);
        const size_t valueAt = json.find('"', kindAt + 7) + 1;
        const std::string kind = json.substr(valueAt, json.find('"', valueAt) - valueAt);
        TEST_ASSERT_TRUE_MESSAGE(kind == "state" || kind == "stream", kind.c_str());
        topics.push_back({tag, kind == "state" ? TopicKind::STATE : TopicKind::STREAM});
    }
    return topics;
}

void nothing(SnapshotSink&) {}

} // namespace

void setUp() {
    ownerMode = socket_message_ModesEnum_DEACTIVATED;
    afterModeRead = nullptr;
    events.clear();
}
void tearDown() {}

void test_a_subscribe_to_a_state_topic_is_answered_with_the_current_value_then_the_ack() {
    FakeAdapter adapter;
    ownerMode = socket_message_ModesEnum_WALK;

    adapter.subscribeTo(MODE_TAG, 3);

    assertFrames({WALK, MODE_ACK}, adapter.to(3));
}

void test_a_second_client_gets_its_own_snapshot_and_the_first_nothing() {
    FakeAdapter adapter;
    adapter.subscribeTo(MODE_TAG, 3);
    adapter.forget();

    adapter.subscribeTo(MODE_TAG, 4);

    assertFrames({DEACTIVATED, MODE_ACK}, adapter.to(4));
    TEST_ASSERT_EQUAL(0, adapter.to(3).size());
}

// The app resubscribes every tag on a reconnect and on a fail-over between transports, so the robot hears a subscribe
// from a client that already has it: it answers again, but a second entry on the list would send every push twice.
void test_a_repeated_subscribe_is_answered_again_and_gets_each_push_once() {
    FakeAdapter adapter;
    adapter.subscribeTo(MODE_TAG, 3);
    adapter.subscribeTo(MODE_TAG, 3);
    assertFrames({DEACTIVATED, MODE_ACK, DEACTIVATED, MODE_ACK}, adapter.to(3));
    adapter.forget();

    adapter.emit(modeData(socket_message_ModesEnum_STAND));

    assertFrames({STAND}, adapter.to(3));
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
    adapter.forget();

    adapter.emit(modeData(socket_message_ModesEnum_STAND));

    TEST_ASSERT_FALSE(adapter.hasSubscribers(MODE_TAG));
    TEST_ASSERT_EQUAL(0, adapter.sentCount());
}

void test_a_stream_and_an_unknown_tag_get_the_ack_alone() {
    FakeAdapter adapter;

    adapter.subscribeTo(IMU_TAG, 3);
    adapter.subscribeTo(999, 3);

    assertFrames({"ack " + std::to_string(IMU_TAG), "ack 999"}, adapter.to(3));
}

// The snapshot is the owner's value at the subscribe, and a change after it is pushed after the ack.
void test_the_snapshot_is_the_value_at_the_subscribe_and_a_later_push_follows_the_ack() {
    FakeAdapter adapter;
    ownerMode = socket_message_ModesEnum_STAND;

    adapter.subscribeTo(MODE_TAG, 3);
    adapter.emit(modeData(socket_message_ModesEnum_WALK));

    assertFrames({STAND, MODE_ACK, WALK}, adapter.to(3));
}

// A bridge holds its EventBus subscription only while someone listens. Read before it attached, the owner could change
// in between and the change would never be pushed.
void test_the_bridge_attaches_before_the_owner_is_read() {
    FakeAdapter adapter;
    adapter.onSubscriptionChange([](int32_t tag) { events.push_back("attach " + std::to_string(tag)); });

    adapter.subscribeTo(MODE_TAG, 3);

    TEST_ASSERT_EQUAL(2, events.size());
    TEST_ASSERT_EQUAL_STRING(("attach " + std::to_string(MODE_TAG)).c_str(), events[0].c_str());
    TEST_ASSERT_EQUAL_STRING("read", events[1].c_str());
}

// The ordering the design names as its top risk: the owner changes after the snapshot read it and before the snapshot
// went out, and the bridge pushes the change from its own task. Sent first, that push would be overtaken by the older
// snapshot, and the client would show a mode the robot has left, as live. The push must wait for the snapshot.
void test_a_change_made_while_the_snapshot_is_sent_is_pushed_after_it() {
    FakeAdapter adapter;
    std::thread bridge;
    std::promise<void> pushed;
    std::future<void> pushedNow = pushed.get_future();
    afterModeRead = [&] {
        bridge = std::thread([&] {
            ownerMode = socket_message_ModesEnum_STAND;
            adapter.emit(modeData(socket_message_ModesEnum_STAND));
            pushed.set_value();
        });
        // A push that nothing holds back lands within this wait.
        pushedNow.wait_for(std::chrono::milliseconds(200));
    };

    adapter.subscribeTo(MODE_TAG, 3);
    if (bridge.joinable()) bridge.join();

    std::string lastMode = "none";
    for (const Frame& frame : adapter.to(3))
        if (frame.message.which_message == socket_message_Message_mode_tag) lastMode = describe(frame);
    TEST_ASSERT_EQUAL_STRING_MESSAGE(STAND.c_str(), lastMode.c_str(),
                                     "the client is left with a mode the robot has left");
    assertFrames({DEACTIVATED, MODE_ACK, STAND}, adapter.to(3));
}

// A state push the client cannot take now is dropped, as every frame is; left at that, the client would show the value
// before it until the next change. The service loop's resend sends the current value once the client can take it.
void test_a_dropped_state_push_is_resent_once_as_the_current_value() {
    FakeAdapter adapter;
    adapter.subscribeTo(MODE_TAG, 3);
    adapter.forget();
    adapter.writable = false;
    ownerMode = socket_message_ModesEnum_STAND;
    adapter.emit(modeData(socket_message_ModesEnum_STAND));
    ownerMode = socket_message_ModesEnum_WALK;
    adapter.emit(modeData(socket_message_ModesEnum_WALK));
    adapter.resendDirty();
    TEST_ASSERT_EQUAL(0, adapter.sentCount());

    adapter.writable = true;
    adapter.resendDirty();
    adapter.resendDirty();

    assertFrames({WALK}, adapter.to(3));
}

// A send can fail on a client that looked ready: a BLE notify without an mbuf, a serial write cut short. The push is
// then lost as a dropped one is, and resent the same way; a resend that fails too keeps the mark for the next pass.
void test_a_state_push_whose_send_fails_is_resent_once_it_goes_out() {
    FakeAdapter adapter;
    adapter.subscribeTo(MODE_TAG, 3);
    adapter.forget();
    adapter.sendsFail = true;
    ownerMode = socket_message_ModesEnum_STAND;
    adapter.emit(modeData(socket_message_ModesEnum_STAND));
    adapter.resendDirty();
    TEST_ASSERT_EQUAL(0, adapter.sentCount());

    adapter.sendsFail = false;
    adapter.resendDirty();
    adapter.resendDirty();

    assertFrames({STAND}, adapter.to(3));
}

// A stream's next frame supersedes the one dropped, and a client that took every push has nothing to catch up on.
void test_a_dropped_stream_frame_and_a_delivered_push_are_not_resent() {
    FakeAdapter adapter;
    adapter.subscribeTo(IMU_TAG, 3);
    adapter.subscribeTo(MODE_TAG, 4);
    adapter.forget();
    adapter.writable = false;
    adapter.emit(socket_message_IMUData(socket_message_IMUData_init_zero));
    adapter.writable = true;
    adapter.emit(modeData(socket_message_ModesEnum_STAND));

    adapter.resendDirty();

    TEST_ASSERT_EQUAL(0, adapter.to(3).size());
    assertFrames({STAND}, adapter.to(4));
}

void test_the_topic_table_matches_the_shared_one() {
    const std::vector<TopicKindEntry> shared = sharedTopics();

    TEST_ASSERT_EQUAL(std::size(TOPIC_KINDS), shared.size());
    for (size_t i = 0; i < shared.size(); i++) {
        TEST_ASSERT_EQUAL(shared[i].tag, TOPIC_KINDS[i].tag);
        TEST_ASSERT_TRUE(shared[i].kind == TOPIC_KINDS[i].kind);
    }
}

// main.cpp asserts its table with servesEveryTopic at compile time; here the check itself is checked.
void test_a_table_serves_every_topic_only_with_each_topic_and_a_snapshot_for_each_state() {
    Topic table[std::size(TOPIC_KINDS)];
    for (size_t i = 0; i < std::size(TOPIC_KINDS); i++) {
        const bool state = TOPIC_KINDS[i].kind == TopicKind::STATE;
        table[i] = {TOPIC_KINDS[i].tag, TOPIC_KINDS[i].kind, state ? nothing : nullptr};
    }
    TEST_ASSERT_TRUE(servesEveryTopic(table));

    const size_t imu = 0, mode = 1;
    TEST_ASSERT_EQUAL(MODE_TAG, table[mode].tag);
    table[mode].snapshot = nullptr;
    TEST_ASSERT_FALSE(servesEveryTopic(table));
    table[mode].snapshot = nothing;

    table[imu].snapshot = nothing;
    TEST_ASSERT_FALSE(servesEveryTopic(table));
    table[imu].snapshot = nullptr;

    table[mode].kind = TopicKind::STREAM;
    table[mode].snapshot = nullptr;
    TEST_ASSERT_FALSE(servesEveryTopic(table));

    Topic missingOne[std::size(TOPIC_KINDS) - 1];
    std::copy(std::begin(table), std::end(table) - 1, std::begin(missingOne));
    TEST_ASSERT_FALSE(servesEveryTopic(missingOne));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_a_subscribe_to_a_state_topic_is_answered_with_the_current_value_then_the_ack);
    RUN_TEST(test_a_second_client_gets_its_own_snapshot_and_the_first_nothing);
    RUN_TEST(test_a_repeated_subscribe_is_answered_again_and_gets_each_push_once);
    RUN_TEST(test_one_unsubscribe_ends_a_repeated_subscription);
    RUN_TEST(test_a_stream_and_an_unknown_tag_get_the_ack_alone);
    RUN_TEST(test_the_snapshot_is_the_value_at_the_subscribe_and_a_later_push_follows_the_ack);
    RUN_TEST(test_the_bridge_attaches_before_the_owner_is_read);
    RUN_TEST(test_a_change_made_while_the_snapshot_is_sent_is_pushed_after_it);
    RUN_TEST(test_a_dropped_state_push_is_resent_once_as_the_current_value);
    RUN_TEST(test_a_state_push_whose_send_fails_is_resent_once_it_goes_out);
    RUN_TEST(test_a_dropped_stream_frame_and_a_delivered_push_are_not_resent);
    RUN_TEST(test_the_topic_table_matches_the_shared_one);
    RUN_TEST(test_a_table_serves_every_topic_only_with_each_topic_and_a_snapshot_for_each_state);
    return UNITY_END();
}
