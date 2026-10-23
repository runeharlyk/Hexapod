// Host tests of the settings groups as state topics (settings/settings_topics.h), on the fake transport of
// test_topic_snapshot. A subscribe is answered with the stored group, redacted, then the ack; a stored change is pushed
// to every subscriber from the service's update handler, with no service lock held, so a push from a correlation
// handler reaches the client that asked.

#include <unity.h>

#include <chrono>
#include <cstring>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <communication/fake_adapter.h>
#include <settings/settings_topics.h>

namespace {

constexpr int32_t WIFI_TAG = socket_message_Message_wifi_settings_tag;
constexpr int32_t AP_TAG = socket_message_Message_ap_settings_tag;

// The robot's five settings services, one robot per test; the topic table reaches them through `robot`.
struct Robot {
    StatefulService<api_ServoSettings> servo;
    StatefulService<api_PeripheralSettings> peripheral;
    StatefulService<api_WifiSettings> wifi;
    StatefulService<api_APSettings> ap;
    StatefulService<api_MDNSSettings> mdns;
};
Robot* robot = nullptr;

constexpr Topic TOPICS[] = {
    {socket_message_Message_servo_settings_tag, TopicKind::STATE,
     [](SnapshotSink& sink) { settings_topics::snapshot(robot->servo, sink); }},
    {socket_message_Message_peripheral_settings_tag, TopicKind::STATE,
     [](SnapshotSink& sink) { settings_topics::snapshot(robot->peripheral, sink); }},
    {WIFI_TAG, TopicKind::STATE, [](SnapshotSink& sink) { settings_topics::snapshot(robot->wifi, sink); }},
    {AP_TAG, TopicKind::STATE, [](SnapshotSink& sink) { settings_topics::snapshot(robot->ap, sink); }},
    {socket_message_Message_mdns_settings_tag, TopicKind::STATE,
     [](SnapshotSink& sink) { settings_topics::snapshot(robot->mdns, sink); }},
};

void addNetwork(api_WifiSettings& settings, const char* ssid, const char* password) {
    api_WifiNetwork& network = settings.wifi_networks[settings.wifi_networks_count++];
    std::strncpy(network.ssid, ssid, sizeof(network.ssid) - 1);
    std::strncpy(network.password, password, sizeof(network.password) - 1);
}

void storeWifi(StatefulService<api_WifiSettings>& service) {
    service.updateWithoutPropagation([](api_WifiSettings& s) {
        s = api_WifiSettings_init_zero;
        std::strncpy(s.hostname, "hexapod", sizeof(s.hostname) - 1);
        addNetwork(s, "home", "pw1");
        addNetwork(s, "lab", "pw2");
        s.applied_secret = 0xABCD;
        return StateUpdateResult::CHANGED;
    });
}

StateUpdateResult renameHost(api_WifiSettings& s) {
    std::strncpy(s.hostname, "renamed", sizeof(s.hostname) - 1);
    return StateUpdateResult::CHANGED;
}

// The tag of each frame, as a line: "tag 312; ack 312; ".
std::string tags(const std::vector<Frame>& frames) {
    std::string out;
    for (const Frame& frame : frames) {
        const socket_message_Message& m = frame.message;
        out += m.which_message == socket_message_Message_sub_notif_tag
                   ? "ack " + std::to_string(m.message.sub_notif.tag) + "; "
                   : "tag " + std::to_string((int)m.which_message) + "; ";
    }
    return out;
}

std::string wifiThenAck() { return "tag " + std::to_string(WIFI_TAG) + "; ack " + std::to_string(WIFI_TAG) + "; "; }

void assertRedacted(const api_WifiSettings& sent) {
    TEST_ASSERT_EQUAL(2, sent.wifi_networks_count);
    TEST_ASSERT_EQUAL_STRING("home", sent.wifi_networks[0].ssid);
    TEST_ASSERT_EQUAL_STRING("lab", sent.wifi_networks[1].ssid);
    TEST_ASSERT_EQUAL_STRING("", sent.wifi_networks[0].password);
    TEST_ASSERT_EQUAL_STRING("", sent.wifi_networks[1].password);
    TEST_ASSERT_EQUAL_UINT32(0, sent.applied_secret);
}

// Whether `work` ends within `limit`. One that does not is left running on its own thread, so a deadlock fails the test
// instead of hanging the suite; what it uses must then outlive the test.
bool endsWithin(std::chrono::milliseconds limit, std::function<void()> work) {
    auto done = std::make_shared<std::promise<void>>();
    std::future<void> ended = done->get_future();
    std::thread worker([work, done] {
        work();
        done->set_value();
    });
    if (ended.wait_for(limit) != std::future_status::ready) {
        worker.detach();
        return false;
    }
    worker.join();
    return true;
}

} // namespace

void setUp() {}
void tearDown() { robot = nullptr; }

void test_a_subscribe_to_wifi_settings_gets_them_without_passwords_or_the_secret_then_the_ack() {
    Robot r;
    robot = &r;
    storeWifi(r.wifi);
    FakeAdapter adapter(TOPICS);

    adapter.subscribeTo(WIFI_TAG, 3);

    const std::vector<Frame> frames = adapter.to(3);
    TEST_ASSERT_EQUAL_STRING(wifiThenAck().c_str(), tags(frames).c_str());
    assertRedacted(frames[0].message.message.wifi_settings);
}

void test_an_ap_settings_snapshot_carries_no_password() {
    Robot r;
    robot = &r;
    r.ap.updateWithoutPropagation([](api_APSettings& s) {
        std::strncpy(s.ssid, "hexapod-ap", sizeof(s.ssid) - 1);
        std::strncpy(s.password, "secret", sizeof(s.password) - 1);
        return StateUpdateResult::CHANGED;
    });
    FakeAdapter adapter(TOPICS);

    adapter.subscribeTo(AP_TAG, 3);

    const std::vector<Frame> frames = adapter.to(3);
    TEST_ASSERT_EQUAL(2, frames.size());
    TEST_ASSERT_EQUAL(AP_TAG, frames[0].message.which_message);
    TEST_ASSERT_EQUAL_STRING("hexapod-ap", frames[0].message.message.ap_settings.ssid);
    TEST_ASSERT_EQUAL_STRING("", frames[0].message.message.ap_settings.password);
}

// The groups without a secret go out as stored.
void test_each_other_group_is_answered_with_its_stored_value_then_the_ack() {
    Robot r;
    robot = &r;
    r.servo.updateWithoutPropagation([](api_ServoSettings& s) {
        s.servos_count = 18;
        s.servos[17].center_angle = 12.5f;
        return StateUpdateResult::CHANGED;
    });
    r.peripheral.updateWithoutPropagation([](api_PeripheralSettings& s) {
        s.sda = 8;
        return StateUpdateResult::CHANGED;
    });
    r.mdns.updateWithoutPropagation([](api_MDNSSettings& s) {
        std::strncpy(s.hostname, "hexapod-2", sizeof(s.hostname) - 1);
        return StateUpdateResult::CHANGED;
    });
    FakeAdapter adapter(TOPICS);

    adapter.subscribeTo(socket_message_Message_servo_settings_tag, 3);
    adapter.subscribeTo(socket_message_Message_peripheral_settings_tag, 3);
    adapter.subscribeTo(socket_message_Message_mdns_settings_tag, 3);

    const std::vector<Frame> frames = adapter.to(3);
    TEST_ASSERT_EQUAL(6, frames.size());
    TEST_ASSERT_EQUAL(18, frames[0].message.message.servo_settings.servos_count);
    TEST_ASSERT_EQUAL_FLOAT(12.5f, frames[0].message.message.servo_settings.servos[17].center_angle);
    TEST_ASSERT_EQUAL(socket_message_Message_sub_notif_tag, frames[1].message.which_message);
    TEST_ASSERT_EQUAL(8, frames[2].message.message.peripheral_settings.sda);
    TEST_ASSERT_EQUAL(socket_message_Message_sub_notif_tag, frames[3].message.which_message);
    TEST_ASSERT_EQUAL_STRING("hexapod-2", frames[4].message.message.mdns_settings.hostname);
    TEST_ASSERT_EQUAL(socket_message_Message_sub_notif_tag, frames[5].message.which_message);
}

void test_a_changed_update_pushes_the_redacted_group_once_to_every_subscriber() {
    Robot r;
    robot = &r;
    storeWifi(r.wifi);
    FakeAdapter adapter(TOPICS);
    settings_topics::pushOnChange(r.wifi, [&adapter](const api_WifiSettings& s) { adapter.emit(s); });
    adapter.subscribeTo(WIFI_TAG, 3);
    adapter.subscribeTo(WIFI_TAG, 4);
    adapter.forget();

    r.wifi.update(renameHost, "test");

    for (int client : {3, 4}) {
        const std::vector<Frame> frames = adapter.to(client);
        TEST_ASSERT_EQUAL_STRING(("tag " + std::to_string(WIFI_TAG) + "; ").c_str(), tags(frames).c_str());
        TEST_ASSERT_EQUAL_STRING("renamed", frames[0].message.message.wifi_settings.hostname);
        assertRedacted(frames[0].message.message.wifi_settings);
    }
}

// The reply settles a write; a group the update left as it was, or refused, has nothing new to push.
void test_an_unchanged_or_refused_update_pushes_nothing() {
    Robot r;
    robot = &r;
    storeWifi(r.wifi);
    FakeAdapter adapter(TOPICS);
    settings_topics::pushOnChange(r.wifi, [&adapter](const api_WifiSettings& s) { adapter.emit(s); });
    adapter.subscribeTo(WIFI_TAG, 3);
    adapter.forget();

    r.wifi.update([](api_WifiSettings&) { return StateUpdateResult::UNCHANGED; }, "test");
    r.wifi.update([](api_WifiSettings&) { return StateUpdateResult::ERROR; }, "test");

    TEST_ASSERT_EQUAL(0, adapter.sentCount());
}

// A push sends under the adapter's encode lock, which a subscribe holds while its snapshot waits for the service lock:
// a push that sent with the service lock held would deadlock against it. Another task must be able to read the service
// while the push sends.
void test_a_push_holds_no_service_lock() {
    Robot r;
    robot = &r;
    storeWifi(r.wifi);
    std::thread reader;
    bool readWhileSending = false;
    settings_topics::pushOnChange(r.wifi, [&](const api_WifiSettings&) {
        auto read = std::make_shared<std::promise<void>>();
        std::future<void> readNow = read->get_future();
        reader = std::thread([&r, read] {
            r.wifi.read([](const api_WifiSettings&) {});
            read->set_value();
        });
        readWhileSending = readNow.wait_for(std::chrono::seconds(1)) == std::future_status::ready;
    });

    r.wifi.update(renameHost, "test");
    reader.join();

    TEST_ASSERT_TRUE_MESSAGE(readWhileSending, "the push held the service lock while it sent");
}

// The correlation handler runs on the transport's task and updates the group there, so its push goes out from inside
// the handler, to the adapter that is decoding the request: the request's own client gets it, then the reply.
void test_a_push_from_a_correlation_handler_reaches_its_own_client() {
    // Left behind, not freed, if the handler never returns: its thread may still hold them.
    auto* r = new Robot;
    robot = r;
    storeWifi(r->wifi);
    auto* adapter = new FakeAdapter(TOPICS);
    settings_topics::pushOnChange(r->wifi, [adapter](const api_WifiSettings& s) { adapter->emit(s); });
    adapter->on<socket_message_CorrelationRequest>([adapter, r](const socket_message_CorrelationRequest& req, int cid) {
        r->wifi.update(renameHost, "correlation");
        socket_message_CorrelationResponse res = socket_message_CorrelationResponse_init_zero;
        res.correlation_id = req.correlation_id;
        res.status_code = 200;
        res.which_response = socket_message_CorrelationResponse_wifi_settings_tag;
        settings_topics::readRedacted(r->wifi, res.response.wifi_settings);
        adapter->emit(res, cid);
    });
    adapter->subscribeTo(WIFI_TAG, 3);
    adapter->forget();
    socket_message_Message request = socket_message_Message_init_zero;
    request.which_message = socket_message_Message_correlation_request_tag;
    request.message.correlation_request.correlation_id = 7;
    request.message.correlation_request.which_request = socket_message_CorrelationRequest_wifi_settings_update_tag;

    const bool returned = endsWithin(std::chrono::seconds(1), [adapter, request] { adapter->receive(request, 3); });

    TEST_ASSERT_TRUE_MESSAGE(returned, "the correlation handler deadlocked on its own adapter");
    const std::vector<Frame> frames = adapter->to(3);
    const std::string expected = "tag " + std::to_string(WIFI_TAG) + "; tag " +
                                 std::to_string(socket_message_Message_correlation_response_tag) + "; ";
    TEST_ASSERT_EQUAL_STRING(expected.c_str(), tags(frames).c_str());
    TEST_ASSERT_EQUAL_STRING("renamed", frames[0].message.message.wifi_settings.hostname);
    delete adapter;
    delete r;
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_a_subscribe_to_wifi_settings_gets_them_without_passwords_or_the_secret_then_the_ack);
    RUN_TEST(test_an_ap_settings_snapshot_carries_no_password);
    RUN_TEST(test_each_other_group_is_answered_with_its_stored_value_then_the_ack);
    RUN_TEST(test_a_changed_update_pushes_the_redacted_group_once_to_every_subscriber);
    RUN_TEST(test_an_unchanged_or_refused_update_pushes_nothing);
    RUN_TEST(test_a_push_holds_no_service_lock);
    RUN_TEST(test_a_push_from_a_correlation_handler_reaches_its_own_client);
    return UNITY_END();
}
