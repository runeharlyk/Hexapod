#pragma once

#ifndef CONFIG_HTTPD_WS_SUPPORT
#define CONFIG_HTTPD_WS_SUPPORT 1
#endif

#include <esp_http_server.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <functional>
#include <vector>
#include <string>
#include <map>
#include <pb_encode.h>

using HttpHandler = std::function<esp_err_t(httpd_req_t*)>;
using WsFrameHandler = std::function<esp_err_t(httpd_req_t*, httpd_ws_frame_t*)>;
using WsOpenHandler = std::function<void(httpd_req_t*)>;
using WsCloseHandler = std::function<void(int)>;

struct HttpRoute {
    std::string uri;
    httpd_method_t method;
    HttpHandler handler;
    bool isWebsocket;
};

class WebServer {
  public:
    WebServer();
    ~WebServer();

    void config(size_t maxUriHandlers, size_t stackSize);
    esp_err_t listen(uint16_t port);
    void stop();

    void on(const char* uri, httpd_method_t method, HttpHandler handler);

    void onWsFrame(WsFrameHandler handler);
    void onWsOpen(WsOpenHandler handler);
    void onWsClose(WsCloseHandler handler);
    void registerWebsocket(const char* uri);

    esp_err_t wsSend(int sockfd, const uint8_t* data, size_t len);
    esp_err_t wsSendAll(const uint8_t* data, size_t len);
    void addWsClient(int sockfd);
    void removeWsClient(int sockfd);

    void addDefaultHeader(const char* key, const char* value);

    httpd_handle_t getHandle() { return server_; }

    static esp_err_t sendError(httpd_req_t* req, int status, const char* message);
    static esp_err_t sendOk(httpd_req_t* req);
    static esp_err_t send(httpd_req_t* req, int status, const uint8_t* data, size_t len);

    template <typename T>
    static esp_err_t send(httpd_req_t* req, int status, const T& msg, const pb_msgdesc_t* fields) {
        size_t size = 0;
        if (!pb_get_encoded_size(&size, fields, &msg)) {
            return sendError(req, 500, "Failed to calculate proto size");
        }

        uint8_t* buffer = (uint8_t*)malloc(size);
        if (!buffer) {
            return sendError(req, 500, "Failed to allocate memory for proto");
        }

        pb_ostream_t stream = pb_ostream_from_buffer(buffer, size);
        if (!pb_encode(&stream, fields, &msg)) {
            free(buffer);
            return sendError(req, 500, "Failed to encode proto");
        }

        esp_err_t result = send(req, status, buffer, stream.bytes_written);
        free(buffer);
        return result;
    }

  private:
    httpd_handle_t server_ = nullptr;
    httpd_config_t config_;
    std::vector<HttpRoute> routes_;
    std::map<std::string, std::string> defaultHeaders_;
    std::vector<int> wsClients_;
    SemaphoreHandle_t wsMutex_;

    WsFrameHandler wsFrameHandler_;
    WsOpenHandler wsOpenHandler_;
    WsCloseHandler wsCloseHandler_;

    static esp_err_t httpHandler(httpd_req_t* req);
    static esp_err_t wsHandler(httpd_req_t* req);

    // httpd close_fn: runs for every terminated session, not just a graceful WebSocket CLOSE.
    static void sessionClosed(httpd_handle_t hd, int sockfd);

    // Caller must hold wsMutex_; frames from different tasks would otherwise interleave on one socket.
    esp_err_t wsSendFrame(int sockfd, const uint8_t* data, size_t len);

    void applyDefaultHeaders(httpd_req_t* req);
    esp_err_t registerRoute(const HttpRoute& route);
};

extern WebServer server;
