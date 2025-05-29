#pragma once

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <filesystem.h>
#include <template/stateful_service.h>
#include <template/stateful_persistence.h>
#include <template/stateful_endpoint.h>
#include <settings/bluetooth_settings.h>
#include "event_bus.h"
#include "message_types.h"
#include <event_socket.h>

#include <map>
#include <list>
#include <vector>

#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

#define CALLS_PER_SECOND(name)                                            \
    static unsigned long name##_count = 0;                                \
    static unsigned long last_time = 0;                                   \
    name##_count++;                                                       \
    if (millis() - last_time >= 1000) {                                   \
        Serial.printf("%s: %lu calls per second\n", #name, name##_count); \
        name##_count = 0;                                                 \
        last_time = millis();                                             \
    }

class CommAdapterBase {
  public:
    CommAdapterBase() { mutex_ = xSemaphoreCreateMutex(); }
    ~CommAdapterBase() { vSemaphoreDelete(mutex_); }

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
    virtual void send(const char* data) = 0;

  private:
    SemaphoreHandle_t mutex_;
    std::map<message_topic_t, std::list<int>> subs_;
};

class BluetoothService : public StatefulService<BluetoothSettings>, public CommAdapterBase {
  private:
    FSPersistence<BluetoothSettings> _persistence;
    BLEServer* _server;
    BLECharacteristic* _txCharacteristic;
    BLECharacteristic* _rxCharacteristic;
    bool _deviceConnected;
    void* _cmdSubHandle;
    void* _tempSubHandle;

    class ServerCallbacks : public BLEServerCallbacks {
        BluetoothService* _service;

      public:
        ServerCallbacks(BluetoothService* service) : _service(service) {}
        void onConnect(BLEServer* pServer) override;
        void onDisconnect(BLEServer* pServer) override;
    };

    class RXCallbacks : public BLECharacteristicCallbacks {
        BluetoothService* _service;

      public:
        RXCallbacks(BluetoothService* service) : _service(service) {}
        void onWrite(BLECharacteristic* characteristic) override;
    };

    void restart();
    void setup();
    void handleReceivedData(const std::string& data);
    void send(const char* data) override;

  public:
    BluetoothService();
    ~BluetoothService();

    void begin();

    bool isDeviceConnected() const { return _deviceConnected; }
    void sendData(const std::string& data);
    void sendData(uint8_t* data, size_t length);

    StatefulHttpEndpoint<BluetoothSettings> endpoint;

    static esp_err_t getStatus(PsychicRequest* request);
};