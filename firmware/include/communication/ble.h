#pragma once

#include <NimBLEDevice.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <vector>

#include <communication/comm_base.hpp>

// Nordic UART Service BLE transport (esp-nimble-cpp), carrying the same protobuf frames as the WS.
#define BLE_SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_CHARACTERISTIC_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_CHARACTERISTIC_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

// Upper bound for a single GATT write; the app chunks to the MTU, well below this.
#ifndef BLE_MAX_MESSAGE_SIZE
#define BLE_MAX_MESSAGE_SIZE 512
#endif

// A reassembled frame spans several writes and can outgrow one (a WiFi settings update is ~730 bytes), so it is
// bounded by the same size the encoder uses. Anything longer is a corrupted length prefix.
static constexpr size_t BLE_MAX_FRAME_SIZE = PROTO_BUFFER_SIZE;
static constexpr size_t BLE_RX_BUFFER_LIMIT = 2 * BLE_MAX_FRAME_SIZE;

struct BLEMessage {
    uint8_t data[BLE_MAX_MESSAGE_SIZE];
    size_t length;
};

class BLE : public CommAdapterBase {
  public:
    BLE() = default;
    ~BLE();

    void begin() override;

  private:
    NimBLEServer *_server{nullptr};
    NimBLECharacteristic *_txCharacteristic{nullptr};
    NimBLECharacteristic *_rxCharacteristic{nullptr};
    volatile bool _deviceConnected{false};

    QueueHandle_t _messageQueue{nullptr};
    TaskHandle_t _processingTask{nullptr};
    bool _taskRunning{false};
    // Serializes send() so concurrent emitters' chunks can't interleave on the wire.
    SemaphoreHandle_t _txMutex{nullptr};

    // Reassembles the length-prefixed chunks a >MTU message is split into. BLE_Process task only; other
    // tasks go through requestRxReset(). A zero-length queued message is the reset marker.
    std::vector<uint8_t> _rxBuffer;
    void requestRxReset();

    class ServerCallbacks : public NimBLEServerCallbacks {
        BLE *_service;

      public:
        explicit ServerCallbacks(BLE *service) : _service(service) {}
        void onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) override;
        void onDisconnect(NimBLEServer *server, NimBLEConnInfo &connInfo, int reason) override;
    };

    class RXCallbacks : public NimBLECharacteristicCallbacks {
        BLE *_service;

      public:
        explicit RXCallbacks(BLE *service) : _service(service) {}
        void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &connInfo) override;
    };

    void setup();
    void send(const uint8_t *data, size_t len, int cid = -1) override;
    void reassemble(const uint8_t *data, size_t len);
    static void messageProcessingTask(void *parameter);
};
