#include <communication/ble_adapter.h>
#include <cstring>

void BLE::begin() {
    _messageQueue = xQueueCreate(30, sizeof(BLEMessage));
    if (!_messageQueue) {
        ESP_LOGE("BluetoothService", "Failed to create message queue");
        return;
    }

    _taskRunning = true;
    BaseType_t result =
        xTaskCreatePinnedToCore(messageProcessingTask, "BLE_Process", 8192, this, 6, &_processingTask, 0);

    if (result != pdPASS) {
        ESP_LOGE("BluetoothService", "Failed to create processing task");
        _taskRunning = false;
        vQueueDelete(_messageQueue);
        _messageQueue = nullptr;
        return;
    }

    setup();
}

void BLE::setup() {
    ESP_LOGI("BluetoothService", "Initializing BLE with device name: %s", "Hexapod");

    NimBLEDevice::init("Hexapod");
    NimBLEDevice::setMTU(255);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    _server = NimBLEDevice::createServer();
    _server->setCallbacks(new ServerCallbacks(this));

    NimBLEService* service = _server->createService(SERVICE_UUID);

    _txCharacteristic = service->createCharacteristic(CHARACTERISTIC_TX, NIMBLE_PROPERTY::NOTIFY);

    _rxCharacteristic = service->createCharacteristic(CHARACTERISTIC_RX, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    _rxCharacteristic->setCallbacks(new RXCallbacks(this));

    service->start();
    NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->start();

    ESP_LOGI("BluetoothService", "BLE UART service started, advertising as %s", "Hexapod");
}

void BLE::restart() {
    ESP_LOGI("BluetoothService", "Restarting BLE service due to settings update.");
    if (_server) {
        NimBLEDevice::deinit(true);
        _server = nullptr;
        _txCharacteristic = nullptr;
        _rxCharacteristic = nullptr;
    }
    setup();
}

void BLE::ServerCallbacks::onConnect(NimBLEServer* pServer) {
    _service->_deviceConnected = true;
    ESP_LOGI("BluetoothService", "Client connected");
}

void BLE::ServerCallbacks::onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) {
    _service->_deviceConnected = true;
    ESP_LOGI("BluetoothService", "Client connected, handle: %d", connInfo.getConnHandle());

    // Optimize connection parameters for low latency
    // minInterval: 15ms (12 * 1.25ms), maxInterval: 30ms (24 * 1.25ms), latency: 0, timeout: 400 (4s)
    pServer->updateConnParams(connInfo.getConnHandle(), 12, 24, 0, 400);
}

void BLE::ServerCallbacks::onDisconnect(NimBLEServer* pServer) {
    _service->_deviceConnected = false;
    ESP_LOGI("BluetoothService", "Client disconnected");
    pServer->startAdvertising();
    ESP_LOGI("BluetoothService", "Restarting advertising");
}

void BLE::ServerCallbacks::onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) {
    onDisconnect(pServer);
}

void BLE::RXCallbacks::onWrite(NimBLECharacteristic* characteristic) {
    NimBLEAttValue value = characteristic->getValue();
    const uint8_t* data = value.data();
    size_t len = value.length();
    if (len && len <= BLE_MAX_MESSAGE_SIZE) {
        BLEMessage msg;
        memcpy(msg.data, data, len);
        msg.length = len;

        if (xQueueSend(_service->_messageQueue, &msg, 0) != pdTRUE) {
            ESP_LOGW("BluetoothService", "Message queue full, dropping message");
        }
    }
}

void BLE::RXCallbacks::onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) { onWrite(characteristic); }

void BLE::messageProcessingTask(void* parameter) {
    BLE* ble = static_cast<BLE*>(parameter);
    BLEMessage msg;

    while (ble->_taskRunning) {
        if (xQueueReceive(ble->_messageQueue, &msg, portMAX_DELAY) == pdTRUE) {
            ble->processMessage(msg.data, msg.length);

            // Process any additional messages immediately without yielding
            while (xQueueReceive(ble->_messageQueue, &msg, 0) == pdTRUE) {
                ble->processMessage(msg.data, msg.length);
            }
        }
    }

    vTaskDelete(nullptr);
}

void BLE::processMessage(const uint8_t* data, size_t len) {
    if (data && len > 0) {
        handleIncoming(data, len);
    }
}

void BLE::send(const uint8_t* data, size_t len, int cid) {
    if (!_deviceConnected) return;
    _txCharacteristic->setValue(const_cast<uint8_t*>(data), static_cast<uint16_t>(len));
    _txCharacteristic->notify();
}
