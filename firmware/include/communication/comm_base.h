#pragma once
#include <map>
#include <list>
#include <vector>

#include "event_bus.h"
#include "message_types.h"

enum message_type_t { CONNECT = 0, DISCONNECT = 1, EVENT = 2, PING = 3, PONG = 4 };

class CommAdapterBase {
  public:
    CommAdapterBase() { mutex_ = xSemaphoreCreateMutex(); }
    // ~CommAdapterBase() {
    //     EventBus::unsubscribe<Command>(_cmdSubHandle);
    //     EventBus::unsubscribe<Temp>(_tempSubHandle);
    //     vSemaphoreDelete(mutex_);
    // }

    virtual void begin() {
        _cmdSubHandle = EventBus::subscribe<CommandMsg>([this](CommandMsg const& c) { emit(COMMAND, c); });
        _modeSubHandle = EventBus::subscribe<ModeMsg>([this](ModeMsg const& t) { emit(MODE, t); });
        _gaitSubHandle = EventBus::subscribe<GaitMsg>([this](GaitMsg const& t) { emit(GAIT, t); });
        _servoSubHandle = EventBus::subscribe<ServoSignalMsg>([this](ServoSignalMsg const& t) { emit(SERVO, t); });
    }

    template <typename T>
    void emit(message_topic_t topic, T const& payload) {
        if (!hasSubscribers(topic)) return;

        JsonDocument doc;
        JsonArray array = doc.to<JsonArray>();
        array.add((int)EVENT);
        array.add((int)topic);
        JsonObject obj = array.add<JsonObject>();
        toJson(obj, payload);

        String out;
#if USE_MSGPACK
        serializeMsgPack(doc, out);
#else
        serializeJson(doc, out);
#endif

        send(out.c_str());
    }

  protected:
    void* _cmdSubHandle {nullptr};
    void* _tempSubHandle {nullptr};
    void* _modeSubHandle {nullptr};
    void* _gaitSubHandle {nullptr};
    void* _servoSubHandle {nullptr};

    void subscribe(message_topic_t t, int cid) {
        xSemaphoreTake(mutex_, portMAX_DELAY);
        subs_[t].push_back(cid);
        xSemaphoreGive(mutex_);
    }

    void unsubscribe(message_topic_t t, int cid) {
        xSemaphoreTake(mutex_, portMAX_DELAY);
        subs_[t].remove(cid);
        xSemaphoreGive(mutex_);
    }

    bool hasSubscribers(message_topic_t t) {
        xSemaphoreTake(mutex_, portMAX_DELAY);
        bool r = !subs_[t].empty();
        xSemaphoreGive(mutex_);
        return r;
    }

    virtual void send(const char* data, int cid = -1) = 0;

    virtual void handleIncoming(const std::string& data, int cid = 0) {
        JsonDocument doc;
#if USE_MSGPACK
        DeserializationError error = deserializeMsgPack(doc, data);
#else
        DeserializationError error = deserializeJson(doc, data);
#endif
        if (error) {
            throw std::runtime_error(error.c_str());
        }

        serializeJson(doc, Serial);
        Serial.println();
        JsonArray obj = doc.as<JsonArray>();

        message_type_t type = obj[0].as<message_type_t>();

        switch (type) {
            case CONNECT: {
                message_topic_t topic = obj[1].as<message_topic_t>();
                ESP_LOGI("BluetoothService", "Connecting to topic: %d", topic);
                subscribe(topic, cid);
                break;
            }
            case DISCONNECT: {
                message_topic_t topic = obj[1].as<message_topic_t>();
                ESP_LOGI("BluetoothService", "Disconnecting to topic: %d", topic);
                unsubscribe(topic, cid);
                break;
            }

            case EVENT: {
                message_topic_t topic = obj[1].as<message_topic_t>();
                ESP_LOGI("BluetoothService", "Got payload for topic: %d", topic);
                if (topic == SERVO) {
                    ServoSignalMsg payload;
                    payload.fromJson(obj[2]);
                    EventBus::publish<ServoSignalMsg>(payload, _servoSubHandle);
                } else if (topic == COMMAND) {
                    CommandMsg payload;
                    payload.fromJson(obj[2]);
                    EventBus::publish<CommandMsg>(payload, _cmdSubHandle);
                } else if (topic == MODE) {
                    ModeMsg payload;
                    payload.fromJson(obj[2]);
                    EventBus::publish<ModeMsg>(payload, _modeSubHandle);
                } else if (topic == GAIT) {
                    GaitMsg payload;
                    payload.fromJson(obj[2]);
                    EventBus::publish<GaitMsg>(payload, _gaitSubHandle);
                } else {
                    ESP_LOGI("MESSAGE", "Could not parse topic: %d", topic);
                };
                break;
            }
            case PING: {
                ESP_LOGI("BluetoothService", "Ping");
                send("[4]", cid);
                break;
            }
            case PONG: {
                ESP_LOGI("BluetoothService", "Pong");
                break;
            }

            default: ESP_LOGW("BluetoothService", "Unknown message type: %d", type); break;
        }
    }

  private:
    SemaphoreHandle_t mutex_;
    std::map<message_topic_t, std::list<int>> subs_;
};