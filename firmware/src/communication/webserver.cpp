#include <communication/webserver.h>
#include <communication/origin_policy.h>
#include <communication/ws_frame.h>
#include <esp_log.h>
#include <lwip/sockets.h>
#include <unistd.h>
#include <cstring>
#include <algorithm>

static const char* TAG = "WebServer";

WebServer server;

WebServer::WebServer() { config_ = HTTPD_DEFAULT_CONFIG(); }

WebServer::~WebServer() { stop(); }

void WebServer::config(size_t maxUriHandlers, size_t stackSize) {
    config_.max_uri_handlers = maxUriHandlers;
    config_.stack_size = stackSize;
    config_.max_resp_headers = 16;
    config_.lru_purge_enable = true;
    config_.uri_match_fn = httpd_uri_match_wildcard;

    // close_fn runs for every terminated session; the CLOSE-frame path alone misses a dropped TCP
    // connection, leaving the client subscribed and every later emit failing to send.
    config_.global_user_ctx = this;
    config_.open_fn = &WebServer::sessionOpened;
    config_.close_fn = &WebServer::sessionClosed;
    // A peer that vanished without closing is found within about 11 s instead of after minutes of
    // retransmissions, so the control-loss stop in main.cpp follows it promptly.
    config_.keep_alive_enable = true;
    config_.keep_alive_idle = 5;
    config_.keep_alive_interval = 2;
    config_.keep_alive_count = 3;
}

// Nagle would hold a small reply until the client acknowledged the previous segment, which a
// delayed ACK puts off by tens of milliseconds on every request.
esp_err_t WebServer::sessionOpened(httpd_handle_t, int sockfd) {
    int noDelay = 1;
    if (setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &noDelay, sizeof(noDelay)) != 0) {
        ESP_LOGW(TAG, "TCP_NODELAY not set on socket %d", sockfd);
    }
    return ESP_OK;
}

void WebServer::sessionClosed(httpd_handle_t hd, int sockfd) {
    auto* self = static_cast<WebServer*>(httpd_get_global_user_ctx(hd));
    if (self && self->wsCloseHandler_) self->wsCloseHandler_(sockfd);
    // The server only closes the socket itself when no custom close_fn is set.
    close(sockfd);
}

esp_err_t WebServer::listen(uint16_t port) {
    config_.server_port = port;
    config_.ctrl_port = port + 32768;

    esp_err_t ret = httpd_start(&server_, &config_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start server: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Server started on port %d", port);
    return ESP_OK;
}

void WebServer::stop() {
    if (server_) {
        httpd_stop(server_);
        server_ = nullptr;
    }
}

void WebServer::applyDefaultHeaders(httpd_req_t* req) {
    for (const auto& [key, value] : defaultHeaders_) {
        httpd_resp_set_hdr(req, key.c_str(), value.c_str());
    }
}

void WebServer::addDefaultHeader(const char* key, const char* value) { defaultHeaders_[key] = value; }

// A header that is absent reads as nullptr; one too long to read refuses the request.
static bool readHeader(httpd_req_t* req, const char* field, char* value, size_t size, const char*& read) {
    esp_err_t err = httpd_req_get_hdr_value_str(req, field, value, size);
    read = err == ESP_OK ? value : nullptr;
    return err == ESP_OK || err == ESP_ERR_NOT_FOUND;
}

struct RequestOrigin {
    char origin[128];
    char host[128];
    const char* originRead = nullptr;
    bool allowed = false;

    explicit RequestOrigin(httpd_req_t* req) {
        const char* hostRead = nullptr;
        allowed = readHeader(req, "Origin", origin, sizeof(origin), originRead) &&
                  readHeader(req, "Host", host, sizeof(host), hostRead) && originAllowed(originRead, hostRead);
    }
};

esp_err_t WebServer::httpHandler(httpd_req_t* req) {
    WebServer* self = static_cast<WebServer*>(req->user_ctx);
    self->applyDefaultHeaders(req);
    // CORS for the pages the origin policy allows, named back rather than "*": a wildcard let any
    // page preflight a firmware upload. httpd keeps a pointer to the value, which lives in this
    // frame until the route's handler has answered.
    const RequestOrigin origin(req);
    if (origin.allowed && origin.originRead) {
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", origin.originRead);
        httpd_resp_set_hdr(req, "Vary", "Origin");
    }

    // req->uri still carries the query string, so match on the path portion only -- otherwise every
    // endpoint taking a query parameter (e.g. /api/files/content?path=...) falls through to the 404.
    const char* queryStart = strchr(req->uri, '?');
    const size_t pathLen = queryStart ? static_cast<size_t>(queryStart - req->uri) : strlen(req->uri);

    for (const auto& route : self->routes_) {
        if (route.isWebsocket) continue;

        bool uriMatch = false;
        if (route.uri.back() == '*') {
            size_t prefixLen = route.uri.length() - 1;
            uriMatch = pathLen >= prefixLen && strncmp(req->uri, route.uri.c_str(), prefixLen) == 0;
        } else {
            uriMatch = pathLen == route.uri.length() && strncmp(req->uri, route.uri.c_str(), pathLen) == 0;
        }

        if (uriMatch && route.method == req->method) return route.handler(req);
    }

    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not found");
    return ESP_FAIL;
}

esp_err_t WebServer::wsHandler(httpd_req_t* req) {
    WebServer* self = static_cast<WebServer*>(req->user_ctx);

    if (req->method == HTTP_GET) {
        int sockfd = httpd_req_to_sockfd(req);
        // IDF 5.4 has no pre-handshake callback, so the upgrade has been answered by now; an error
        // here makes httpd close the session before it carries a single frame.
        const RequestOrigin origin(req);
        if (!origin.allowed) {
            ESP_LOGW(TAG, "Refused a WebSocket opened by %s",
                     origin.originRead ? origin.originRead : "an unreadable origin");
            return ESP_FAIL;
        }
        if (self->wsOpenHandler_) {
            self->wsOpenHandler_(req);
        }
        ESP_LOGI(TAG, "WebSocket client connected: %d", sockfd);
        return ESP_OK;
    }

    httpd_ws_frame_t frame;
    memset(&frame, 0, sizeof(httpd_ws_frame_t));
    frame.type = HTTPD_WS_TYPE_BINARY;

    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get frame len: %s", esp_err_to_name(ret));
        return ret;
    }

    if (frame.len > 0) {
        frame.payload = (uint8_t*)malloc(frame.len);
        if (!frame.payload) {
            ESP_LOGE(TAG, "Failed to allocate frame payload");
            return ESP_ERR_NO_MEM;
        }

        ret = httpd_ws_recv_frame(req, &frame, frame.len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to receive frame: %s", esp_err_to_name(ret));
            free(frame.payload);
            return ret;
        }
    }

    if (frame.type == HTTPD_WS_TYPE_CLOSE) {
        int sockfd = httpd_req_to_sockfd(req);
        if (self->wsCloseHandler_) {
            self->wsCloseHandler_(sockfd);
        }
        ESP_LOGI(TAG, "WebSocket client disconnected: %d", sockfd);
        if (frame.payload) free(frame.payload);
        return ESP_OK;
    }

    esp_err_t result = ESP_OK;
    if (self->wsFrameHandler_) {
        result = self->wsFrameHandler_(req, &frame);
    }

    if (frame.payload) {
        free(frame.payload);
    }

    return result;
}

void WebServer::on(const char* uri, httpd_method_t method, HttpHandler handler) {
    HttpRoute route;
    route.uri = uri;
    route.method = method;
    route.handler = std::move(handler);
    route.isWebsocket = false;
    routes_.push_back(route);

    if (server_) {
        registerRoute(route);
    }
}

// Every route is its own httpd handler, so running past config.max_uri_handlers silently loses routes
// unless the result is checked.
esp_err_t WebServer::registerRoute(const HttpRoute& route) {
    httpd_uri_t httpd_route = {.uri = route.uri.c_str(),
                               .method = route.method,
                               .handler = route.isWebsocket ? wsHandler : httpHandler,
                               .user_ctx = this,
                               .is_websocket = route.isWebsocket,
                               .handle_ws_control_frames = route.isWebsocket,
                               .supported_subprotocol = nullptr};
    const esp_err_t err = httpd_register_uri_handler(server_, &httpd_route);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register %s (method %d): %s", route.uri.c_str(), (int)route.method,
                 esp_err_to_name(err));
    }
    return err;
}

void WebServer::registerWebsocket(const char* uri) {
    HttpRoute route;
    route.uri = uri;
    route.method = HTTP_GET;
    route.isWebsocket = true;
    routes_.push_back(route);

    if (server_) {
        registerRoute(route);
    }
}

void WebServer::onWsFrame(WsFrameHandler handler) { wsFrameHandler_ = handler; }

void WebServer::onWsOpen(WsOpenHandler handler) { wsOpenHandler_ = handler; }

void WebServer::onWsClose(WsCloseHandler handler) { wsCloseHandler_ = handler; }

// httpd writes a frame's header and payload with two sends, which with TCP_NODELAY are two segments
// on the air; a small frame goes out whole from one buffer instead. Not writev: lwIP keeps "more data
// follows" set through a vectored write's last part, so its segment lacks PSH. Callers serialize
// sends per socket (CommAdapterBase holds its encode lock across encode and send).
esp_err_t WebServer::wsSend(int sockfd, const uint8_t* data, size_t len) {
    if (len > SINGLE_SEND_MAX) {
        httpd_ws_frame_t frame = {.final = true,
                                  .fragmented = false,
                                  .type = HTTPD_WS_TYPE_BINARY,
                                  .payload = const_cast<uint8_t*>(data),
                                  .len = len};
        return httpd_ws_send_frame_async(server_, sockfd, &frame);
    }
    uint8_t frame[ws_frame::MAX_HEADER + SINGLE_SEND_MAX];
    const size_t headerLen = ws_frame::binaryHeader(frame, len);
    memcpy(frame + headerLen, data, len);
    const ssize_t frameLen = headerLen + len;
    const ssize_t sent = ::send(sockfd, frame, frameLen, 0);
    if (sent == frameLen) return ESP_OK;
    // Nothing more may go on a socket that holds part of a frame: the client would read the next
    // frame's bytes as the rest of this one.
    if (sent > 0) endSession(sockfd);
    ESP_LOGW(TAG, "Frame to socket %d not sent (%d of %d bytes, errno %d)", sockfd, (int)sent, (int)frameLen, errno);
    return ESP_FAIL;
}

void WebServer::endSession(int sockfd) {
    if (server_) httpd_sess_trigger_close(server_, sockfd);
}

bool WebServer::writable(int sockfd) {
    fd_set set;
    FD_ZERO(&set);
    FD_SET(sockfd, &set);
    timeval now = {.tv_sec = 0, .tv_usec = 0};
    return select(sockfd + 1, nullptr, &set, nullptr, &now) > 0;
}

esp_err_t WebServer::sendError(httpd_req_t* req, int status, const char* message) {
    return send(req, status, (uint8_t*)message, strlen(message));
}

esp_err_t WebServer::sendOk(httpd_req_t* req) { return send(req, 200, nullptr, 0); }

static const char* statusLine(int status) {
    switch (status) {
        case 200: return "200 OK";
        case 202: return "202 Accepted";
        case 400: return "400 Bad Request";
        case 404: return "404 Not Found";
        case 408: return "408 Request Timeout";
        case 409: return "409 Conflict";
        case 413: return "413 Content Too Large";
        case 500: return "500 Internal Server Error";
        case 503: return "503 Service Unavailable";
        default: return status >= 200 && status < 300 ? "200 OK" : "500 Internal Server Error";
    }
}

esp_err_t WebServer::send(httpd_req_t* req, int status, const uint8_t* data, size_t len) {
    httpd_resp_set_status(req, statusLine(status));
    httpd_resp_set_type(req, "application/x-protobuf");
    return httpd_resp_send(req, (const char*)data, len);
}
