#include <communication/event_source_adapter.h>

static const char *TAG = "EventSource";

EventSource::EventSource(PsychicHttpServer &server, const char *route) : _server(server), _route(route) {
    _eventSource.onOpen((std::bind(&EventSource::onSSEOpen, this, std::placeholders::_1)));
    _eventSource.onClose(std::bind(&EventSource::onSSEClose, this, std::placeholders::_1));
}

void EventSource::begin() {
    _server.on(_route, [&](PsychicRequest *request) {
        PsychicWebParameter *topicsParam = request->getParam("topics");
        String topics = topicsParam->value();
        ESP_LOGI(TAG, "Got a new request, with topics: %s", topics.c_str());
        // TODO Subscribe to topics
        return _eventSource.handleRequest(request);
    });
}

void EventSource::onSSEOpen(PsychicEventSourceClient *client) {
    ESP_LOGI(TAG, "SSE[%s][%u] connect", client->remoteIP().toString().c_str(), client->socket());
    // client->
}

void EventSource::onSSEClose(PsychicEventSourceClient *client) {
    ESP_LOGI(TAG, "SSE[%s][%u] disconnect", client->remoteIP().toString().c_str(), client->socket());
    unsubscribeAllEventBus();
}
