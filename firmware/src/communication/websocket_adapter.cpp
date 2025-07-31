#include <communication/websocket_adapter.h>

static const char *TAG = "Websocket";

Websocket::Websocket() {
    _socket.onOpen((std::bind(&Websocket::onWSOpen, this, std::placeholders::_1)));
    _socket.onClose(std::bind(&Websocket::onWSClose, this, std::placeholders::_1));
    _socket.onFrame(std::bind(&Websocket::onFrame, this, std::placeholders::_1, std::placeholders::_2));
}

void Websocket::onWSOpen(PsychicWebSocketClient *client) {
    ESP_LOGI(TAG, "ws[%s][%u] connect", client->remoteIP().toString().c_str(), client->socket());
}

void Websocket::onWSClose(PsychicWebSocketClient *client) {
    ESP_LOGI(TAG, "ws[%s][%u] disconnect", client->remoteIP().toString().c_str(), client->socket());
}

esp_err_t Websocket::onFrame(PsychicWebSocketRequest *request, httpd_ws_frame *frame) {
    ESP_LOGV(TAG, "ws[%s][%u] opcode[%d]", request->client()->remoteIP().toString().c_str(),
             request->client()->socket(), frame->type);

    if (frame->type != HTTPD_WS_TYPE_TEXT && frame->type != HTTPD_WS_TYPE_BINARY) {
        ESP_LOGE(TAG, "Unsupported frame type");
        return ESP_OK;
    }

    ESP_LOGV(TAG, "Received message from client %d: %s", request->client()->socket(), (char *)frame->payload);

    handleIncoming(frame->payload, frame->len, request->client()->socket());

    return ESP_OK;
}

void Websocket::send(const uint8_t *data, size_t len, int cid) {
    if (cid != -1) {
        auto *client = _socket.getClient(cid);
        if (client) {
            ESP_LOGV(TAG, "Sending to client %s: %s", client->remoteIP().toString().c_str(), data);
#if USE_MSGPACK
            client->sendMessage(HTTPD_WS_TYPE_BINARY, data, len);
#else
            client->sendMessage(HTTPD_WS_TYPE_TEXT, data, len);
#endif
        }
    } else {
        ESP_LOGV(TAG, "Sending to all clients: %s", data);
#if USE_MSGPACK
        _socket.sendAll(HTTPD_WS_TYPE_BINARY, data, len);
#else
        _socket.sendAll(HTTPD_WS_TYPE_TEXT, data, len);
#endif
    }
}
