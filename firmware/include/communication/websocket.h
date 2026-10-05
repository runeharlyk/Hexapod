#pragma once

#include <cstdint>
#include <mutex>
#include <set>
#include <communication/webserver.h>
#include <communication/comm_base.hpp>
#include <communication/stall_tracker.h>

class Websocket : public CommAdapterBase {
  public:
    Websocket(WebServer& server, const char* route = "/api/ws");

    void begin() override;
    bool hasClient() const override;

  private:
    WebServer& server_;
    const char* route_;
    // close_fn fires for every HTTP session, and a WebSocket CLOSE frame fires it once more, so the
    // open sockets are tracked by id rather than counted.
    mutable std::mutex socketsMutex_;
    std::set<int> openSockets_;
    static constexpr uint32_t STALL_LIMIT_MS = 10000;
    StallTracker stalls_{STALL_LIMIT_MS};

    void onWsOpen(httpd_req_t* req);
    void onWsClose(int sockfd);
    esp_err_t onFrame(httpd_req_t* req, httpd_ws_frame_t* frame);

    bool ready(int cid) override;
    void send(const uint8_t* data, size_t len, int cid = -1) override;
};
