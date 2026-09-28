#include <communication/webserver.h>
#include <esp_log.h>
#include <unistd.h>
#include <cstring>
#include <algorithm>

static const char* TAG = "WebServer";

WebServer server;

WebServer::WebServer() {
    config_ = HTTPD_DEFAULT_CONFIG();
    wsMutex_ = xSemaphoreCreateMutex();
}

WebServer::~WebServer() {
    stop();
    vSemaphoreDelete(wsMutex_);
}

void WebServer::config(size_t maxUriHandlers, size_t stackSize) {
    config_.max_uri_handlers = maxUriHandlers;
    config_.stack_size = stackSize;
    config_.max_resp_headers = 16;
    config_.lru_purge_enable = true;
    config_.uri_match_fn = httpd_uri_match_wildcard;

    // close_fn runs for every terminated session; the CLOSE-frame path alone misses a dropped TCP
    // connection, leaving the client subscribed and every later emit failing to send.
    config_.global_user_ctx = this;
    config_.close_fn = &WebServer::sessionClosed;
}

void WebServer::sessionClosed(httpd_handle_t hd, int sockfd) {
    auto* self = static_cast<WebServer*>(httpd_get_global_user_ctx(hd));
    if (self) {
        self->removeWsClient(sockfd);
        if (self->wsCloseHandler_) self->wsCloseHandler_(sockfd);
    }
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

esp_err_t WebServer::httpHandler(httpd_req_t* req) {
    WebServer* self = static_cast<WebServer*>(req->user_ctx);
    self->applyDefaultHeaders(req);

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
        self->addWsClient(sockfd);
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
        self->removeWsClient(sockfd);
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

void WebServer::addWsClient(int sockfd) {
    xSemaphoreTake(wsMutex_, portMAX_DELAY);
    wsClients_.push_back(sockfd);
    xSemaphoreGive(wsMutex_);
}

void WebServer::removeWsClient(int sockfd) {
    xSemaphoreTake(wsMutex_, portMAX_DELAY);
    wsClients_.erase(std::remove(wsClients_.begin(), wsClients_.end(), sockfd), wsClients_.end());
    xSemaphoreGive(wsMutex_);
}


esp_err_t WebServer::wsSendFrame(int sockfd, const uint8_t* data, size_t len) {
    httpd_ws_frame_t frame = {.final = true,
                              .fragmented = false,
                              .type = HTTPD_WS_TYPE_BINARY,
                              .payload = const_cast<uint8_t*>(data),
                              .len = len};
    return httpd_ws_send_frame_async(server_, sockfd, &frame);
}

esp_err_t WebServer::wsSend(int sockfd, const uint8_t* data, size_t len) {
    xSemaphoreTake(wsMutex_, portMAX_DELAY);
    esp_err_t result = wsSendFrame(sockfd, data, len);
    xSemaphoreGive(wsMutex_);
    return result;
}

esp_err_t WebServer::wsSendAll(const uint8_t* data, size_t len) {
    xSemaphoreTake(wsMutex_, portMAX_DELAY);
    for (int sockfd : wsClients_) {
        wsSendFrame(sockfd, data, len);
    }
    xSemaphoreGive(wsMutex_);
    return ESP_OK;
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
