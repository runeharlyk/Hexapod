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

#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

class BluetoothService : public StatefulService<BluetoothSettings> {
  private:
    FSPersistence<BluetoothSettings> _persistence;
    BLEServer* _server;
    BLECharacteristic* _txCharacteristic;
    BLECharacteristic* _rxCharacteristic;
    bool _deviceConnected;

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

    void setupBLE();
    void restartBLE();
    void handleReceivedData(const std::string& data);

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