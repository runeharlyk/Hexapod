#pragma once

#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <esp_log.h>
#include <utils/ip_address.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <atomic>
#include <memory>

#define DNS_PORT 53
#define DNS_MAX_PACKET_SIZE 512

// Wildcard captive-portal resolver: every A query is answered with the soft-AP address.
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

    static void processRequest(Session& session, const uint8_t* buffer, int len, const sockaddr_in& clientAddr) {
        if (len < headerSize || len > DNS_MAX_PACKET_SIZE - answerSize) return;

        uint16_t flags = (buffer[2] << 8) | buffer[3];
        if ((flags & 0x8000) != 0) return;

        uint8_t response[DNS_MAX_PACKET_SIZE];
        memcpy(response, buffer, len);

        response[2] = 0x81;
        response[3] = 0x80;
        response[6] = 0x00;
        response[7] = 0x01;

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
        memcpy(response + len, answer, answerSize);

        sendto(session.socket, response, len + answerSize, 0, (const struct sockaddr*)&clientAddr, sizeof(clientAddr));
    }

    std::shared_ptr<Session> _session;
};
