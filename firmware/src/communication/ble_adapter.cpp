#include <communication/ble_adapter.h>

void BLE::begin() {
    CommAdapterBase::begin();

    setup();
}

void BLE::setup() {
    ESP_LOGI("BluetoothService", "Initializing BLE with device name: %s", "Hexapod");

    BLEDevice::init("Hexapod");
    _server = BLEDevice::createServer();
    _server->setCallbacks(new ServerCallbacks(this));

    BLEService* service = _server->createService(SERVICE_UUID);

    _txCharacteristic = service->createCharacteristic(CHARACTERISTIC_TX, BLECharacteristic::PROPERTY_NOTIFY);
    _txCharacteristic->addDescriptor(new BLE2902());

    _rxCharacteristic = service->createCharacteristic(CHARACTERISTIC_RX, BLECharacteristic::PROPERTY_WRITE);
    _rxCharacteristic->setCallbacks(new RXCallbacks(this));

    service->start();
    _server->getAdvertising()->start();

    ESP_LOGI("BluetoothService", "BLE UART service started, advertising as %s", "Hexapod");
}

void BLE::restart() {
    ESP_LOGI("BluetoothService", "Restarting BLE service due to settings update.");
    if (_server) {
        BLEDevice::deinit(true);
        _server = nullptr;
        _txCharacteristic = nullptr;
        _rxCharacteristic = nullptr;
    }
    setup();
}

void BLE::ServerCallbacks::onConnect(BLEServer* pServer) {
    _service->_deviceConnected = true;
    ESP_LOGI("BluetoothService", "Client connected");
}

void BLE::ServerCallbacks::onDisconnect(BLEServer* pServer) {
    _service->_deviceConnected = false;
    ESP_LOGI("BluetoothService", "Client disconnected");
    pServer->startAdvertising();
    ESP_LOGI("BluetoothService", "Restarting advertising");
}

void BLE::RXCallbacks::onWrite(BLECharacteristic* characteristic) {
    uint8_t* data = characteristic->getData();
    size_t len = characteristic->getLength();
    if (len) {
        _service->handleIncoming(data, len);
    }
}

void BLE::send(const uint8_t* data, size_t len, int cid) {
    if (!_deviceConnected) return;
    _txCharacteristic->setValue(const_cast<uint8_t*>(data), static_cast<uint16_t>(len));
    _txCharacteristic->notify();
}
