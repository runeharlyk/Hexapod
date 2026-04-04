#pragma once
#include <map>
#include <list>
#include <vector>

#include "event_bus.h"
#include "message_types.h"
#include "utils/timing.h"

#ifndef MAX_CID
#define MAX_CID 64
#endif

enum class MsgKind : uint8_t { CONNECT = 0, DISCONNECT = 1, EVENT = 2, PING = 3, PONG = 4 };

class CommAdapterBase {
  public:
    CommAdapterBase() {
        mutex_ = xSemaphoreCreateMutex();
        if (!busMutex_) {
            busMutex_ = xSemaphoreCreateMutex();
        }
    }

    ~CommAdapterBase() {
        unsubscribeAllEventBus();
        vSemaphoreDelete(mutex_);
    }

    virtual void begin() {}

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

    void send(const char* data, int cid = -1) { send(reinterpret_cast<const uint8_t*>(data), strlen(data), cid); }
    virtual void send(const uint8_t* data, size_t len, int cid = -1) = 0;

    virtual void handleIncoming(const uint8_t* data, size_t len, int cid = 0) {
        CALLS_PER_SECOND(Handle_incoming);
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
                ESP_LOGI("Comm Base", "CONNECT topic: %d (cid=%d)", topic, cid);
                subscribe(topic, cid);
                break;
            }

            case MsgKind::DISCONNECT: {
                message_topic_t topic = obj[1].as<message_topic_t>();
                ESP_LOGI("Comm Base", "DISCONNECT topic: %d (cid=%d)", topic, cid);
                unsubscribe(topic, cid);
                break;
            }

            case MsgKind::EVENT: {
                message_topic_t topic = obj[1].as<message_topic_t>();
                ESP_LOGD("Comm Base", "EVENT payload for topic: %d", topic);

                // Mark this as an incoming message to prevent re-emission
                _incomingMessage = true;

                if (topic == SERVO_SIGNAL) {
                    ServoSignalMsg payload;
                    payload.fromJson(obj[2]);
                    EventBus<ServoSignalMsg>::publish(payload);
                } else if (topic == COMMAND) {
                    CommandMsg payload;
                    payload.fromJson(obj[2]);
                    EventBus<CommandMsg>::publish(payload);
                } else if (topic == MODE) {
                    ModeMsg payload;
                    payload.fromJson(obj[2]);
                    EventBus<ModeMsg>::publish(payload);
                } else if (topic == GAIT) {
                    GaitMsg payload;
                    payload.fromJson(obj[2]);
                    ESP_LOGI("Comm Base", "Publishing GAIT message with value %d, _incomingMessage=%s",
                             (int)payload.gait, _incomingMessage ? "true" : "false");
                    EventBus<GaitMsg>::publish(payload);
                } else if (topic == SERVO_SETTINGS) {
                    ServoSettingsMsg payload;
                    payload.fromJson(obj[2]);
                    EventBus<ServoSettingsMsg>::publish(payload);
                } else if (topic == ANGLE) {
                    ServoAnglesMsg payload;
                    payload.fromJson(obj[2]);
                    EventBus<ServoAnglesMsg>::publish(payload);
                } else {
                    ESP_LOGW("Comm Base", "Unknown EVENT topic: %d", topic);
                }

                _incomingMessage = false;
                break;
            }

            case MsgKind::PING: {
                ESP_LOGI("Comm Base", "PING (cid=%d)", cid);
#if USE_MSGPACK
                static const uint8_t pong[] = {0x91, 0x04};
                send(pong, sizeof(pong), cid);
#else
                send("[4]", cid);
#endif
                break;
            }

            case MsgKind::PONG: ESP_LOGI("Comm Base", "PONG (cid=%d)", cid); break;

            default: ESP_LOGW("Comm Base", "Unknown message type: %d", static_cast<int>(type)); break;
        }
    }

  protected:
    static EventBus<CommandMsg>::Handle _cmdSubHandle;
    static EventBus<ModeMsg>::Handle _modeSubHandle;
    static EventBus<GaitMsg>::Handle _gaitSubHandle;
    static EventBus<ServoAnglesMsg>::Handle _angleSubHandle;
    static EventBus<ServoSignalMsg>::Handle _servoSubHandle;
    static EventBus<ServoSettingsMsg>::Handle _servoSettingsMsgSubHandle;

    std::map<message_topic_t, std::list<int>> subs_;

    static std::map<message_topic_t, bool> busActive_;
    static SemaphoreHandle_t busMutex_;
    static thread_local bool _incomingMessage;

    SemaphoreHandle_t mutex_;

    void subscribe(message_topic_t t, int cid) {
        xSemaphoreTake(mutex_, portMAX_DELAY);
        auto& list = subs_[t];
        bool wasEmpty = list.empty();
        list.push_back(cid);
        xSemaphoreGive(mutex_);

        if (wasEmpty) subscribeIfNeeded(t);
        sendLatestIfAvailable(t, cid);
    }

    void unsubscribe(message_topic_t t, int cid) {
        xSemaphoreTake(mutex_, portMAX_DELAY);
        auto& list = subs_[t];
        list.remove(cid);
        bool isEmpty = list.empty();
        xSemaphoreGive(mutex_);

        if (isEmpty) unsubscribeIfNeeded(t);
    }

    bool hasSubscribers(message_topic_t t) {
        xSemaphoreTake(mutex_, portMAX_DELAY);
        bool r = !subs_[t].empty();
        xSemaphoreGive(mutex_);
        return r;
    }

    void subscribeIfNeeded(message_topic_t t) {
        ESP_LOGI("Comm Base", "subscribeIfNeeded called for topic %d by adapter %p", t, this);
        xSemaphoreTake(busMutex_, portMAX_DELAY);
        if (busActive_[t]) {
            ESP_LOGI("Comm Base", "Topic %d already active, skipping subscription", t);
            xSemaphoreGive(busMutex_);
            return;
        }
        ESP_LOGI("Comm Base", "Creating new subscription for topic %d", t);

        switch (t) {
            case COMMAND:
                _cmdSubHandle = EventBus<CommandMsg>::subscribe([this](const CommandMsg& c) {
                    if (!_incomingMessage) {
                        emit(COMMAND, c);
                    }
                });
                break;
            case MODE:
                _modeSubHandle = EventBus<ModeMsg>::subscribe([this](const ModeMsg& m) {
                    if (!_incomingMessage) {
                        emit(MODE, m);
                    }
                });
                break;
            case GAIT:
                ESP_LOGI("Comm Base", "Creating GAIT subscription for adapter %p", this);
                _gaitSubHandle = EventBus<GaitMsg>::subscribe([this](const GaitMsg& g) {
                    ESP_LOGI("Comm Base", "GAIT event received by adapter %p, _incomingMessage=%s", this,
                             _incomingMessage ? "true" : "false");
                    if (!_incomingMessage) {
                        ESP_LOGI("Comm Base", "Emitting GAIT to clients from adapter %p", this);
                        emit(GAIT, g);
                    } else {
                        ESP_LOGI("Comm Base", "Skipping GAIT emit (incoming message) from adapter %p", this);
                    }
                });
                break;
            case SERVO_SIGNAL:
                _servoSubHandle = EventBus<ServoSignalMsg>::subscribe([this](const ServoSignalMsg& s) {
                    if (!_incomingMessage) {
                        emit(SERVO_SIGNAL, s);
                    }
                });
                break;
            case SERVO_SETTINGS:
                _servoSettingsMsgSubHandle = EventBus<ServoSettingsMsg>::subscribe([this](const ServoSettingsMsg& s) {
                    if (!_incomingMessage) {
                        emit(SERVO_SETTINGS, s);
                    }
                });
                break;
            case ANGLE:
                _angleSubHandle = EventBus<ServoAnglesMsg>::subscribe([this](const ServoAnglesMsg& a) {
                    if (!_incomingMessage) {
                        emit(ANGLE, a);
                    }
                });
                break;
            default: break;
        }
        busActive_[t] = true;
        xSemaphoreGive(busMutex_);
    }

    void unsubscribeIfNeeded(message_topic_t t) {
        xSemaphoreTake(busMutex_, portMAX_DELAY);
        if (!busActive_[t]) {
            xSemaphoreGive(busMutex_);
            return;
        }

        switch (t) {
            case COMMAND: _cmdSubHandle.unsubscribe(); break;
            case MODE: _modeSubHandle.unsubscribe(); break;
            case GAIT: _gaitSubHandle.unsubscribe(); break;
            case SERVO_SIGNAL: _servoSubHandle.unsubscribe(); break;
            case SERVO_SETTINGS: _servoSettingsMsgSubHandle.unsubscribe(); break;
            case ANGLE: _angleSubHandle.unsubscribe(); break;
            default: break;
        }
        busActive_[t] = false;
        xSemaphoreGive(busMutex_);
    }

    void unsubscribeAllEventBus() {
        _cmdSubHandle.unsubscribe();
        _modeSubHandle.unsubscribe();
        _gaitSubHandle.unsubscribe();
        _angleSubHandle.unsubscribe();
        _servoSubHandle.unsubscribe();
        _servoSettingsMsgSubHandle.unsubscribe();
    }

    void sendLatestIfAvailable(message_topic_t t, int cid) {
        switch (t) {
            case COMMAND: {
                CommandMsg m;
                if (EventBus<CommandMsg>::peek(m)) emit(COMMAND, m, cid);
                break;
            }
            case MODE: {
                ModeMsg m;
                if (EventBus<ModeMsg>::peek(m)) emit(MODE, m, cid);
                break;
            }
            case GAIT: {
                GaitMsg m;
                if (EventBus<GaitMsg>::peek(m)) emit(GAIT, m, cid);
                break;
            }
            case SERVO_SIGNAL: {
                ServoSignalMsg m;
                if (EventBus<ServoSignalMsg>::peek(m)) emit(SERVO_SIGNAL, m, cid);
                break;
            }
            case SERVO_SETTINGS: {
                ServoSettingsMsg m;
                if (EventBus<ServoSettingsMsg>::peek(m)) emit(SERVO_SETTINGS, m, cid);
                break;
            }
            case ANGLE: {
                ServoAnglesMsg m;
                if (EventBus<ServoAnglesMsg>::peek(m)) emit(ANGLE, m, cid);
                break;
            }
            default: break;
        }
    }

    void ping(int cid) {
#if USE_MSGPACK
        static const uint8_t pong[] = {0x91, 0x03};
        send(pong, sizeof(pong), cid);
#else
        send("[3]", cid);
#endif
    }
};