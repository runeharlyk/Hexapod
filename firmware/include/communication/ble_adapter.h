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
#include "communication/comm_base.h"

#include <map>
#include <list>
#include <vector>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

struct BLEMessage {
    uint8_t data[512];
    size_t length;
};

class BLE : public CommAdapterBase {
  private:
    BLEServer* _server {nullptr};
    BLECharacteristic* _txCharacteristic {nullptr};
    BLECharacteristic* _rxCharacteristic {nullptr};
    bool _deviceConnected {false};

    QueueHandle_t _messageQueue {nullptr};
    TaskHandle_t _processingTask {nullptr};
    bool _taskRunning {false};

    class ServerCallbacks : public BLEServerCallbacks {
        BLE* _service;

      public:
        ServerCallbacks(BLE* service) : _service(service) {}
        void onConnect(BLEServer* pServer) override;
        void onDisconnect(BLEServer* pServer) override;
    };

    class RXCallbacks : public BLECharacteristicCallbacks {
        BLE* _service;

      public:
        RXCallbacks(BLE* service) : _service(service) {}
        void onWrite(BLECharacteristic* characteristic) override;
    };

    void restart();
    void setup();
    void send(const uint8_t* data, size_t len, int cid = -1) override;
    static void messageProcessingTask(void* parameter);
    void processMessage(const uint8_t* data, size_t len);

  public:
    BLE() {};
    ~BLE() {
        if (_server) BLEDevice::deinit(true);
        if (_messageQueue) vQueueDelete(_messageQueue);
        if (_processingTask) vTaskDelete(_processingTask);
    };

    void begin() override;
    bool isDeviceConnected() const { return _deviceConnected; }
};