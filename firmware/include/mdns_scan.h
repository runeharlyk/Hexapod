#pragma once

// The mDNS page's scan of the robot's network. A query collects answers for 3 s, and a transport task that waited for
// them took no other message from its client meanwhile, sticks, modes and pings included. So the transport task only
// queues the query; the service task begins it, polls it on its 100 ms pass, and sends the reply once it has ended,
// with the request's correlation id, through the client's transport; CommAdapterBase::offer drops it rather than wait
// on a client that cannot take it now, as a subscription's frame is dropped.

#include <cstdint>
#include <cstring>
#include <mutex>
#include <optional>
#include <platform_shared/message.pb.h>

// A query of the robot's network for one service type: esp-mdns on the robot (mdns_service.h), a fake in the host
// tests. Called from the service task only, the task that starts and stops mDNS.
class MdnsBrowser {
  public:
    virtual ~MdnsBrowser() = default;
    // Sends the query and collects answers for `timeoutMs` without waiting for them; false when mDNS cannot query.
    virtual bool begin(const char* service, const char* protocol, uint32_t timeoutMs) = 0;
    // Whether the query begun last has ended; true once, after which take() is due.
    virtual bool ended() = 0;
    // Writes the ended query's answers, as many as the response holds, and releases the query.
    virtual void take(api_MDNSQueryResponse& out) = 0;
};

class MdnsScan {
  public:
    static constexpr uint32_t ANSWER_WINDOW_MS = 3000;

    // A client is its adapter and its id on that adapter: ids repeat across transports.
    struct Client {
        const void* adapter;
        int id;
        bool operator==(const Client& other) const { return adapter == other.adapter && id == other.id; }
    };

    // The status a request is answered with at once, and on refusal the reason the app shows.
    struct Reply {
        uint32_t status;
        const char* reason;
    };

    explicit MdnsScan(MdnsBrowser& browser) : browser_(browser) {}

    // A transport task: queues the query and returns at once. 202: the reply follows once the query has ended.
    Reply start(const api_MDNSQueryRequest& request, Client client, uint32_t correlationId) {
        if (request.service[0] == '\0' || request.protocol[0] == '\0')
            return {400, "A scan needs a service type and a protocol"};
        std::lock_guard<std::mutex> lock(mutex_);
        if (state_ != State::IDLE) return {503, "An mDNS scan is already running"};
        request_ = request;
        client_ = client;
        correlationId_ = correlationId;
        clientPresent_ = true;
        state_ = State::QUEUED;
        return {202, ""};
    }

    // The service task, on every pass: begins a queued query; true once its reply is due, which answer() writes.
    bool due() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (state_ == State::QUEUED) {
            began_ = browser_.begin(request_.service, request_.protocol, ANSWER_WINDOW_MS);
            state_ = began_ ? State::RUNNING : State::ENDED;
        }
        if (state_ == State::RUNNING && browser_.ended()) state_ = State::ENDED;
        return state_ == State::ENDED;
    }

    // The service task, once due() is: writes the reply into `res`, frees the scan for the next request, and returns
    // the client it goes to, or none when that client has gone.
    std::optional<Client> answer(socket_message_CorrelationResponse& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (state_ != State::ENDED) return std::nullopt;
        res.correlation_id = correlationId_;
        res.which_response = socket_message_CorrelationResponse_mdns_query_response_tag;
        if (began_) {
            browser_.take(res.response.mdns_query_response);
            res.status_code = 200;
        } else {
            res.status_code = 503;
            strncpy(res.error_message, "mDNS is not running", sizeof(res.error_message) - 1);
        }
        state_ = State::IDLE;
        if (!clientPresent_) return std::nullopt;
        return client_;
    }

    // A transport task, when a client leaves: a query not begun yet is dropped; a running one runs out unanswered.
    void clientGone(Client client) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (state_ == State::IDLE || !(client_ == client)) return;
        if (state_ == State::QUEUED) state_ = State::IDLE;
        clientPresent_ = false;
    }

  private:
    enum class State { IDLE, QUEUED, RUNNING, ENDED };

    MdnsBrowser& browser_;
    std::mutex mutex_;
    State state_ = State::IDLE;
    api_MDNSQueryRequest request_ = api_MDNSQueryRequest_init_zero;
    Client client_ {nullptr, -1};
    uint32_t correlationId_ = 0;
    bool clientPresent_ = false;
    bool began_ = false;
};
