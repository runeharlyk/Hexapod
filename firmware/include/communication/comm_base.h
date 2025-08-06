#pragma once
#include <map>
#include <list>
#include <vector>

#include "event_bus.h"
#include "message_types.h"

#ifndef MAX_CID
#define MAX_CID 64
#endif

enum class MsgKind : uint8_t { CONNECT = 0, DISCONNECT = 1, EVENT = 2, PING = 3, PONG = 4 };

// TODO: Make subscriptions to eventbus be dynamic.

class CommAdapterBase {
  public:
    CommAdapterBase() { mutex_ = xSemaphoreCreateMutex(); }
    ~CommAdapterBase() {
        _cmdSubHandle.unsubscribe();
        _modeSubHandle.unsubscribe();
        _gaitSubHandle.unsubscribe();
        _angleSubHandle.unsubscribe();
        _servoSubHandle.unsubscribe();
        _servoSettingsMsgSubHandle.unsubscribe();
        vSemaphoreDelete(mutex_);
    }

    virtual void begin() {
        _cmdSubHandle = EventBus<CommandMsg>::subscribe([this](const CommandMsg& c) { emit(COMMAND, c); });
        _modeSubHandle = EventBus<ModeMsg>::subscribe([this](const ModeMsg& t) { emit(MODE, t); });
        _gaitSubHandle = EventBus<GaitMsg>::subscribe([this](const GaitMsg& t) { emit(GAIT, t); });
        _angleSubHandle = EventBus<ServoAnglesMsg>::subscribe([this](const ServoAnglesMsg& t) { emit(GAIT, t); });
        _servoSubHandle =
            EventBus<ServoSignalMsg>::subscribe([this](const ServoSignalMsg& s) { emit(SERVO_SIGNAL, s); });
        _servoSettingsMsgSubHandle =
            EventBus<ServoSettingsMsg>::subscribe([this](const ServoSettingsMsg& s) { emit(SERVO_SETTINGS, s); });
    }

    template <typename T>
    void emit(message_topic_t topic, T const& payload, int cid = -1) {
        if (!hasSubscribers(topic)) return;

        JsonDocument doc;
        JsonArray array = doc.to<JsonArray>();
        array.add(static_cast<uint8_t>(MsgKind::EVENT));
        array.add(static_cast<uint8_t>(topic));
        toJson(array.add<JsonVariant>(), payload);

#if USE_MSGPACK
        std::string bin;
        serializeMsgPack(doc, bin);
        send(reinterpret_cast<const uint8_t*>(bin.data()), bin.size(), cid);
#else
        String out;
        serializeJson(doc, out);
        send(out.c_str(), cid);
#endif
    }

  protected:
    EventBus<CommandMsg>::Handle _cmdSubHandle;
    EventBus<ModeMsg>::Handle _modeSubHandle;
    EventBus<GaitMsg>::Handle _gaitSubHandle;
    EventBus<ServoAnglesMsg>::Handle _angleSubHandle;
    EventBus<ServoSignalMsg>::Handle _servoSubHandle;
    EventBus<ServoSettingsMsg>::Handle _servoSettingsMsgSubHandle;

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

    void send(const char* data, int cid = -1) { send(reinterpret_cast<const uint8_t*>(data), strlen(data), cid); }

    virtual void send(const uint8_t* data, size_t len, int cid = -1) = 0;

    virtual void handleIncoming(const uint8_t* data, size_t len, int cid = 0) {
        JsonDocument doc;
#if USE_MSGPACK
        DeserializationError error = deserializeMsgPack(doc, data, len);
#else
        DeserializationError error = deserializeJson(doc, data, len);
#endif
        if (error) {
            ESP_LOGE("Comm Base", "Failed to deserialize incoming: (%s)", error.c_str());
            return;
        }

        JsonArrayConst obj = doc.as<JsonArrayConst>();

        MsgKind type = static_cast<MsgKind>(obj[0].as<uint8_t>());

        switch (type) {
            case MsgKind::CONNECT: {
                message_topic_t topic = obj[1].as<message_topic_t>();
                ESP_LOGI("Comm Base", "Connecting to topic: %d", topic);
                subscribe(topic, cid);
                switch (topic) {
                    case SERVO_SETTINGS:
                        ServoSettingsMsg m;
                        if (EventBus<ServoSettingsMsg>::peek(m))
                            emit(SERVO_SETTINGS, m, cid);
                        else
                            ESP_LOGE("Comm Base", "Could not peek last value for topic: %d", topic);
                        break;
                }
                break;
            }
            case MsgKind::DISCONNECT: {
                message_topic_t topic = obj[1].as<message_topic_t>();
                ESP_LOGI("Comm Base", "Disconnecting to topic: %d", topic);
                unsubscribe(topic, cid);
                break;
            }

            case MsgKind::EVENT: {
                message_topic_t topic = obj[1].as<message_topic_t>();
                ESP_LOGD("Comm Base", "Got payload for topic: %d", topic);
                if (topic == SERVO_SIGNAL) {
                    ServoSignalMsg payload;
                    payload.fromJson(obj[2]);
                    EventBus<ServoSignalMsg>::publish(payload, _servoSubHandle);
                } else if (topic == COMMAND) {
                    CommandMsg payload;
                    payload.fromJson(obj[2]);
                    EventBus<CommandMsg>::publish(payload, _cmdSubHandle);
                } else if (topic == MODE) {
                    ModeMsg payload;
                    payload.fromJson(obj[2]);
                    EventBus<ModeMsg>::publish(payload, _modeSubHandle);
                } else if (topic == GAIT) {
                    GaitMsg payload;
                    payload.fromJson(obj[2]);
                    EventBus<GaitMsg>::publish(payload, _gaitSubHandle);
                } else if (topic == SERVO_SETTINGS) {
                    ServoSettingsMsg payload;
                    payload.fromJson(obj[2]);
                    EventBus<ServoSettingsMsg>::publish(payload, _servoSettingsMsgSubHandle);
                } else {
                    ESP_LOGI("MESSAGE", "Could not parse topic: %d", topic);
                };
                break;
            }
            case MsgKind::PING: {
                ESP_LOGI("Comm Base", "Ping");
#if USE_MSGPACK
                static const uint8_t pong[] = {0x91, 0x04}; // [4] in MsgPack
                send(pong, sizeof(pong), cid);
#else
                send("[4]", cid);
#endif
                break;
            }
            case MsgKind::PONG: {
                ESP_LOGI("Comm Base", "Pong");
                break;
            }

            default: ESP_LOGW("Comm Base", "Unknown message type: %d", type); break;
        }
    }

  private:
    SemaphoreHandle_t mutex_;
    std::map<message_topic_t, std::list<int>> subs_;
};