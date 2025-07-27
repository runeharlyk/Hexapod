#pragma once

#include <PsychicHttp.h>
#include <list>
#include <map>
#include <vector>

#include <template/stateful_service.h>
#include <communication/comm_base.h>

typedef std::function<void(JsonObject &root, int originId)> EventCallback;
typedef std::function<void(const String &originId, bool sync)> SubscribeCallback;

class Websocket {
  public:
    Websocket();

    PsychicWebSocketHandler *getHandler() { return &_socket; }

    bool hasSubscribers(const char *event);

    void onEvent(String event, EventCallback callback);

    void onSubscribe(String event, SubscribeCallback callback);

    void emit(const char *event, const char *payload, const char *originId = "", bool onlyToSameOrigin = false);
    // if onlyToSameOrigin == true, the message will be sent to the originId only, otherwise it will be broadcasted to
    // all clients except the originId

  private:
    PsychicWebSocketHandler _socket;

    std::map<String, std::list<int>> client_subscriptions;
    std::map<String, std::list<EventCallback>> event_callbacks;
    std::map<String, std::list<SubscribeCallback>> subscribe_callbacks;
    void handleEventCallbacks(String event, JsonObject &jsonObject, int originId);
    void handleSubscribeCallbacks(String event, const String &originId);

    void onWSOpen(PsychicWebSocketClient *client);
    void onWSClose(PsychicWebSocketClient *client);
    esp_err_t onFrame(PsychicWebSocketRequest *request, httpd_ws_frame *frame);
};
