// Host tests of the mDNS page's scan. A query collects answers for 3 s; the transport task that takes the request must
// not wait them out, or its client's sticks, modes and pings would wait with it. So the request only queues the query,
// the service task begins and polls it, and the reply follows once, to the client that asked, with its correlation id.

#include <unity.h>

#include <cstring>
#include <string>
#include <vector>
#include <communication/comm_base.hpp>
#include <mdns_scan.h>

namespace {

// The robot's mDNS stack as the scan sees it: a query answers when the test ends it, never by itself.
class FakeBrowser : public MdnsBrowser {
  public:
    bool running = true; // mDNS started
    std::vector<std::string> begun;
    uint32_t windowMs = 0;
    bool answered = false;
    std::vector<std::string> hosts;
    int taken = 0;

    bool begin(const char* service, const char* protocol, uint32_t timeoutMs) override {
        if (!running) return false;
        begun.push_back(std::string(service) + "." + protocol);
        windowMs = timeoutMs;
        answered = false;
        return true;
    }
    bool ended() override { return answered; }
    void take(api_MDNSQueryResponse& out) override {
        taken++;
        out.services_count = 0;
        for (const std::string& host : hosts) {
            strncpy(out.services[out.services_count].name, host.c_str(), sizeof(out.services[0].name) - 1);
            out.services[out.services_count++].port = 80;
        }
    }
};

int ws = 0, ble = 0; // two transports, by address

// A transport whose client's socket can or cannot take a frame now; it counts the frames that went out.
class FakeAdapter : public CommAdapterBase {
  public:
    bool writable = true;
    int sent = 0;
    bool hasClient() const override { return true; }

  protected:
    bool ready(int) override { return writable; }
    bool send(const uint8_t*, size_t, int) override {
        sent++;
        return true;
    }
};

api_MDNSQueryRequest query(const char* service, const char* protocol) {
    api_MDNSQueryRequest request = api_MDNSQueryRequest_init_zero;
    strncpy(request.service, service, sizeof(request.service) - 1);
    strncpy(request.protocol, protocol, sizeof(request.protocol) - 1);
    return request;
}

const api_MDNSQueryRequest HTTP = query("_http", "_tcp");

} // namespace

void setUp() {}
void tearDown() {}

// The transport task returns at once and leaves the network alone: it is free for the client's next message.
void test_a_request_queues_the_query_and_returns_without_touching_the_network() {
    FakeBrowser browser;
    MdnsScan scan(browser);

    const MdnsScan::Reply reply = scan.start(HTTP, {&ws, 3}, 41);

    TEST_ASSERT_EQUAL(202, reply.status);
    TEST_ASSERT_EQUAL(0, browser.begun.size());
}

void test_the_service_task_begins_the_query_and_answers_once_it_has_ended() {
    FakeBrowser browser;
    browser.hosts = {"printer", "hexapod-2"};
    MdnsScan scan(browser);
    scan.start(HTTP, {&ws, 3}, 41);

    TEST_ASSERT_FALSE(scan.due());
    TEST_ASSERT(browser.begun == std::vector<std::string> {"_http._tcp"});
    TEST_ASSERT_EQUAL(MdnsScan::ANSWER_WINDOW_MS, browser.windowMs);
    TEST_ASSERT_FALSE(scan.due());

    browser.answered = true;
    TEST_ASSERT_TRUE(scan.due());
    socket_message_CorrelationResponse res = socket_message_CorrelationResponse_init_zero;
    const std::optional<MdnsScan::Client> to = scan.answer(res);

    TEST_ASSERT_TRUE(to.has_value());
    TEST_ASSERT(*to == (MdnsScan::Client {&ws, 3}));
    TEST_ASSERT_EQUAL(41, res.correlation_id);
    TEST_ASSERT_EQUAL(200, res.status_code);
    TEST_ASSERT_EQUAL(socket_message_CorrelationResponse_mdns_query_response_tag, res.which_response);
    TEST_ASSERT_EQUAL(2, res.response.mdns_query_response.services_count);
    TEST_ASSERT_EQUAL_STRING("printer", res.response.mdns_query_response.services[0].name);
}

void test_a_query_is_answered_once_and_frees_the_scan() {
    FakeBrowser browser;
    MdnsScan scan(browser);
    scan.start(HTTP, {&ws, 3}, 41);
    scan.due();
    browser.answered = true;
    scan.due();
    socket_message_CorrelationResponse res = socket_message_CorrelationResponse_init_zero;
    scan.answer(res);

    TEST_ASSERT_FALSE(scan.due());
    TEST_ASSERT_FALSE(scan.answer(res).has_value());
    TEST_ASSERT_EQUAL(1, browser.taken);
    TEST_ASSERT_EQUAL(202, scan.start(query("_ws", "_tcp"), {&ble, 0}, 7).status);
}

// One query at a time: a second request is refused at once, and the first still gets its own reply.
void test_a_request_while_a_scan_runs_is_refused_and_the_first_is_still_answered() {
    FakeBrowser browser;
    MdnsScan scan(browser);
    scan.start(HTTP, {&ws, 3}, 41);

    const MdnsScan::Reply queued = scan.start(HTTP, {&ble, 0}, 9);
    scan.due();
    const MdnsScan::Reply running = scan.start(query("_ws", "_tcp"), {&ws, 3}, 42);

    TEST_ASSERT_EQUAL(503, queued.status);
    TEST_ASSERT_EQUAL(503, running.status);
    TEST_ASSERT_EQUAL_STRING("An mDNS scan is already running", running.reason);
    TEST_ASSERT_EQUAL(1, browser.begun.size());

    browser.answered = true;
    TEST_ASSERT_TRUE(scan.due());
    socket_message_CorrelationResponse res = socket_message_CorrelationResponse_init_zero;
    TEST_ASSERT(*scan.answer(res) == (MdnsScan::Client {&ws, 3}));
    TEST_ASSERT_EQUAL(41, res.correlation_id);
}

// The old synchronous query answered an empty list when mDNS was down, which the page showed as "nothing answered".
void test_a_query_mdns_cannot_begin_is_answered_at_once_as_a_failure() {
    FakeBrowser browser;
    browser.running = false;
    MdnsScan scan(browser);
    scan.start(HTTP, {&ws, 3}, 41);

    TEST_ASSERT_TRUE(scan.due());
    socket_message_CorrelationResponse res = socket_message_CorrelationResponse_init_zero;
    TEST_ASSERT_TRUE(scan.answer(res).has_value());
    TEST_ASSERT_EQUAL(41, res.correlation_id);
    TEST_ASSERT_EQUAL(503, res.status_code);
    TEST_ASSERT_EQUAL_STRING("mDNS is not running", res.error_message);
    TEST_ASSERT_EQUAL(0, browser.taken);
    TEST_ASSERT_EQUAL(202, scan.start(HTTP, {&ws, 3}, 42).status);
}

void test_a_request_without_a_service_type_is_refused() {
    FakeBrowser browser;
    MdnsScan scan(browser);

    TEST_ASSERT_EQUAL(400, scan.start(query("", "_tcp"), {&ws, 3}, 41).status);
    TEST_ASSERT_EQUAL(400, scan.start(query("_http", ""), {&ws, 3}, 42).status);
    TEST_ASSERT_FALSE(scan.due());
    TEST_ASSERT_EQUAL(0, browser.begun.size());
}

// A client that left gets no reply; its query runs out and frees the scan, and one not begun yet never goes out.
void test_a_client_that_leaves_mid_scan_gets_no_reply_and_frees_the_scan() {
    FakeBrowser browser;
    MdnsScan scan(browser);
    scan.start(HTTP, {&ws, 3}, 41);
    scan.due();

    scan.clientGone({&ble, 3});
    scan.clientGone({&ws, 3});
    browser.answered = true;
    TEST_ASSERT_TRUE(scan.due());
    socket_message_CorrelationResponse res = socket_message_CorrelationResponse_init_zero;
    TEST_ASSERT_FALSE(scan.answer(res).has_value());
    TEST_ASSERT_EQUAL(1, browser.taken);
    TEST_ASSERT_EQUAL(202, scan.start(HTTP, {&ble, 0}, 7).status);
}

void test_a_client_that_leaves_before_its_query_begins_sends_none() {
    FakeBrowser browser;
    MdnsScan scan(browser);
    scan.start(HTTP, {&ws, 3}, 41);

    scan.clientGone({&ws, 3});

    TEST_ASSERT_FALSE(scan.due());
    TEST_ASSERT_EQUAL(0, browser.begun.size());
    TEST_ASSERT_EQUAL(202, scan.start(HTTP, {&ble, 0}, 7).status);
}

// Ids repeat across transports: the same id on another transport is another client.
void test_the_same_id_on_another_transport_is_another_client() {
    FakeBrowser browser;
    MdnsScan scan(browser);
    scan.start(HTTP, {&ws, 0}, 41);
    scan.due();

    scan.clientGone({&ble, 0});
    browser.answered = true;
    scan.due();
    socket_message_CorrelationResponse res = socket_message_CorrelationResponse_init_zero;

    TEST_ASSERT_TRUE(scan.answer(res).has_value());
}

// The reply goes out on the service task, which ships every client's telemetry: a client that cannot take it now, a
// stalled browser tab, loses it rather than holding that task in a blocking send for the socket's send timeout.
void test_the_reply_is_dropped_rather_than_waited_for_when_the_client_cannot_take_it() {
    FakeAdapter adapter;
    socket_message_CorrelationResponse res = socket_message_CorrelationResponse_init_zero;

    adapter.writable = false;
    TEST_ASSERT_FALSE(adapter.offer(res, 3));
    TEST_ASSERT_EQUAL(0, adapter.sent);

    adapter.writable = true;
    TEST_ASSERT_TRUE(adapter.offer(res, 3));
    TEST_ASSERT_EQUAL(1, adapter.sent);
}

// mDNS restarts only while no scan holds it (MDNSService::applyPendingReconfigure): from the request, through the
// query, until the reply has taken the query's answers, which a restart would free under it.
void test_the_scan_holds_mdns_from_the_request_until_its_reply_is_written() {
    FakeBrowser browser;
    MdnsScan scan(browser);
    TEST_ASSERT_TRUE(scan.idle());

    scan.start(HTTP, {&ws, 3}, 41);
    TEST_ASSERT_FALSE(scan.idle());
    scan.due();
    TEST_ASSERT_FALSE(scan.idle());
    browser.answered = true;
    TEST_ASSERT_TRUE(scan.due());
    TEST_ASSERT_FALSE(scan.idle());

    socket_message_CorrelationResponse res = socket_message_CorrelationResponse_init_zero;
    scan.answer(res);
    TEST_ASSERT_TRUE(scan.idle());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_a_request_queues_the_query_and_returns_without_touching_the_network);
    RUN_TEST(test_the_service_task_begins_the_query_and_answers_once_it_has_ended);
    RUN_TEST(test_a_query_is_answered_once_and_frees_the_scan);
    RUN_TEST(test_a_request_while_a_scan_runs_is_refused_and_the_first_is_still_answered);
    RUN_TEST(test_a_query_mdns_cannot_begin_is_answered_at_once_as_a_failure);
    RUN_TEST(test_a_request_without_a_service_type_is_refused);
    RUN_TEST(test_a_client_that_leaves_mid_scan_gets_no_reply_and_frees_the_scan);
    RUN_TEST(test_a_client_that_leaves_before_its_query_begins_sends_none);
    RUN_TEST(test_the_same_id_on_another_transport_is_another_client);
    RUN_TEST(test_the_reply_is_dropped_rather_than_waited_for_when_the_client_cannot_take_it);
    RUN_TEST(test_the_scan_holds_mdns_from_the_request_until_its_reply_is_written);
    return UNITY_END();
}
