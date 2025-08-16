#include <communication/event_source_adapter.h>

static const char *TAG = "EventSource";

EventSource::EventSource(PsychicHttpServer &server, const char *route) : _server(server), _route(route) {
    _eventSource.onOpen((std::bind(&EventSource::onSSEOpen, this, std::placeholders::_1)));
    _eventSource.onClose(std::bind(&EventSource::onSSEClose, this, std::placeholders::_1));
}

void EventSource::begin() {
    _server.on("/__sse_attach__", &_eventSource); // Hack to attach the event source to the server
    _server.on(_route, [&](PsychicRequest *request) {
        Serial.printf("Query: %s\n", request->queryString().c_str());
        PsychicWebParameter *topicsParam = request->getParam("topics");

        if (!topicsParam) {
            ESP_LOGW(TAG, "No topic found in params");
            return ESP_FAIL;
        }
        String topics = topicsParam ? urlDecode(topicsParam->value().c_str()) : "";
        ESP_LOGI(TAG, "New SSE request, topics: %s", topics.c_str());

        if (topics.length() > 0) {
            int cid = request->client()->socket();
            std::vector<String> topicList;
            int start = 0;
            int end = topics.indexOf(',');

            while (end >= 0) {
                topicList.push_back(topics.substring(start, end));
                start = end + 1;
                end = topics.indexOf(',', start);
            }
            topicList.push_back(topics.substring(start));

            for (const String &topicStr : topicList) {
                String trimmed = topicStr;
                trimmed.trim();
                if (trimmed.length() > 0) {
                    message_topic_t topic = static_cast<message_topic_t>(trimmed.toInt());
                    ESP_LOGI("event source", "subbbing to %d", topic);
                    subscribe(topic, cid);
                }
            }
        }

        return _eventSource.handleRequest(request);
    });
}

void EventSource::onSSEOpen(PsychicEventSourceClient *client) {
    ESP_LOGI(TAG, "SSE[%s][%u] connect", client->remoteIP().toString().c_str(), client->socket());
}

void EventSource::onSSEClose(PsychicEventSourceClient *client) {
    ESP_LOGI(TAG, "SSE[%s][%u] disconnect", client->remoteIP().toString().c_str(), client->socket());

    int cid = client->socket();
    xSemaphoreTake(mutex_, portMAX_DELAY);

    for (auto &pair : subs_) {
        pair.second.remove(cid);
        if (pair.second.empty()) {
            unsubscribeIfNeeded(pair.first);
        }
    }

    xSemaphoreGive(mutex_);
}

void EventSource::send(const uint8_t *data, size_t len, int cid) {
    if (cid == -1) return;
    auto *client = _eventSource.getClient(cid);
    if (client) {
        ESP_LOGV(TAG, "Sending to client %s: %.*s", client->remoteIP().toString().c_str(), len, data);
        client->send((const char *)data);
    }
}
