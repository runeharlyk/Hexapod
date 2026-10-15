#pragma once

#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <esp_log.h>
#include <utils/ip_address.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <atomic>
#include <cstring>
#include <memory>

#define DNS_PORT 53
#define DNS_MAX_PACKET_SIZE 512

// Wildcard captive-portal resolver: every A query is answered with the soft-AP address. It runs on
// its own task, so the service loop does not need to poll it.
class DNSServer {
  public:
    DNSServer() = default;
    ~DNSServer() { stop(); }

    DNSServer(const DNSServer&) = delete;
    DNSServer& operator=(const DNSServer&) = delete;

    bool start(uint16_t port, const char* domainName, const IPAddress& resolvedIP) {
        if (_session) return true;

        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock < 0) {
            ESP_LOGE(TAG, "Failed to create socket");
            return false;
        }

        auto session = std::make_shared<Session>(sock, resolvedIP);

        int opt = 1;
        setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        // Bounds how long stop() waits: the worker only observes the stop flag between receives.
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = receiveTimeoutMs * 1000;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        struct sockaddr_in serverAddr = {};
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_addr.s_addr = INADDR_ANY;
        serverAddr.sin_port = htons(port);

        if (bind(sock, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
            ESP_LOGE(TAG, "Failed to bind socket");
            return false;
        }

        // The worker owns a reference of its own, so the session outlives a stop() that gives up waiting.
        auto* workerRef = new std::shared_ptr<Session>(session);
        if (xTaskCreate(dnsTask, "dns_server", 4096, workerRef, 3, nullptr) != pdPASS) {
            ESP_LOGE(TAG, "Failed to create task");
            delete workerRef;
            return false;
        }

        _session = std::move(session);
        ESP_LOGI(TAG, "Started on port %u, resolving %s to %s", port, domainName ? domainName : "*",
                 resolvedIP.toString().c_str());
        return true;
    }

    void stop() {
        if (!_session) return;

        _session->running = false;
        for (uint32_t waited = 0; waited < stopTimeoutMs && !_session->exited; waited += stopPollMs) {
            vTaskDelay(pdMS_TO_TICKS(stopPollMs));
        }
        if (!_session->exited) {
            ESP_LOGW(TAG, "Worker still running after %u ms; releasing ownership to it", stopTimeoutMs);
        }

        _session.reset();
        ESP_LOGI(TAG, "Stopped");
    }

  private:
    static constexpr const char* TAG = "DNSServer";
    static constexpr uint32_t receiveTimeoutMs = 250;
    static constexpr uint32_t stopPollMs = 10;
    static constexpr uint32_t stopTimeoutMs = 2000;
    static constexpr int headerSize = 12;
    static constexpr int answerSize = 16;
    static constexpr uint16_t QTYPE_A = 1;
    static constexpr uint16_t QTYPE_ANY = 255;

    // Shared by the owner and the worker task; whichever releases its reference last closes the socket.
    struct Session {
        Session(int sock, const IPAddress& ip) : socket(sock), resolvedIP(ip) {}
        ~Session() { close(socket); }

        Session(const Session&) = delete;
        Session& operator=(const Session&) = delete;

        const int socket;
        const IPAddress resolvedIP;
        std::atomic<bool> running {true};
        std::atomic<bool> exited {false};
    };

    static void dnsTask(void* param) {
        std::shared_ptr<Session> session = *static_cast<std::shared_ptr<Session>*>(param);
        delete static_cast<std::shared_ptr<Session>*>(param);

        run(*session);

        session->exited = true;
        session.reset();
        vTaskDelete(nullptr);
    }

    static void run(Session& session) {
        uint8_t buffer[DNS_MAX_PACKET_SIZE];
        struct sockaddr_in clientAddr;

        while (session.running) {
            socklen_t addrLen = sizeof(clientAddr);
            int len = recvfrom(session.socket, buffer, sizeof(buffer), 0, (struct sockaddr*)&clientAddr, &addrLen);
            if (len > 0) {
                processRequest(session, buffer, len, clientAddr);
            }
        }
    }

    // Length of the first question (name + QTYPE + QCLASS) starting at the header's end, or 0 when
    // it is malformed or uses a compression pointer, which a question never needs.
    static int questionLength(const uint8_t* buffer, int len) {
        int p = headerSize;
        while (p < len && buffer[p] != 0) {
            if ((buffer[p] & 0xC0) != 0) return 0;
            p += 1 + buffer[p];
        }
        p += 1 + 4;  // root label, QTYPE, QCLASS
        return p <= len ? p - headerSize : 0;
    }

    // Replies with the header and the first question only (dropping any EDNS OPT or other additional
    // records the client sent) and answers A or ANY queries with the soft-AP address. Other types,
    // AAAA included, get an empty NOERROR answer so clients fall back to IPv4 instead of receiving
    // an A record they did not ask for.
    static void processRequest(Session& session, const uint8_t* buffer, int len, const sockaddr_in& clientAddr) {
        if (len < headerSize) return;

        const uint16_t flags = (buffer[2] << 8) | buffer[3];
        const bool isQuery = (flags & 0x8000) == 0;
        const bool standardOpcode = ((flags >> 11) & 0x0F) == 0;
        const uint16_t qdcount = (buffer[4] << 8) | buffer[5];
        if (!isQuery || !standardOpcode || qdcount == 0) return;

        const int qlen = questionLength(buffer, len);
        if (qlen == 0 || headerSize + qlen + answerSize > DNS_MAX_PACKET_SIZE) return;
        const int questionEnd = headerSize + qlen;
        const uint16_t qtype = (buffer[questionEnd - 4] << 8) | buffer[questionEnd - 3];
        const bool answerIt = qtype == QTYPE_A || qtype == QTYPE_ANY;

        uint8_t response[DNS_MAX_PACKET_SIZE];
        memcpy(response, buffer, questionEnd);
        response[2] = 0x80 | (buffer[2] & 0x01);  // QR, standard query, echo RD
        response[3] = 0x80;                       // RA, RCODE NOERROR
        response[4] = 0x00;
        response[5] = 0x01;  // QDCOUNT
        response[6] = 0x00;
        response[7] = answerIt ? 0x01 : 0x00;  // ANCOUNT
        memset(response + 8, 0, 4);            // NSCOUNT, ARCOUNT

        int responseLen = questionEnd;
        if (answerIt) {
            // Answer record: name pointer to the question, type A, class IN, TTL 60 s, rdlength 4, address.
            const uint32_t ip = static_cast<uint32_t>(session.resolvedIP);
            const uint8_t answer[answerSize] = {0xC0,
                                                0x0C,
                                                0x00,
                                                0x01,
                                                0x00,
                                                0x01,
                                                0x00,
                                                0x00,
                                                0x00,
                                                0x3C,
                                                0x00,
                                                0x04,
                                                static_cast<uint8_t>(ip),
                                                static_cast<uint8_t>(ip >> 8),
                                                static_cast<uint8_t>(ip >> 16),
                                                static_cast<uint8_t>(ip >> 24)};
            memcpy(response + questionEnd, answer, answerSize);
            responseLen += answerSize;
        }

        sendto(session.socket, response, responseLen, 0, (const struct sockaddr*)&clientAddr, sizeof(clientAddr));
    }

    std::shared_ptr<Session> _session;
};
