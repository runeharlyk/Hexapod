#include <bluetooth_service.h>
#include <PsychicHttp.h>

BluetoothService::BluetoothService()
    : StatefulService<BluetoothSettings>(),
      endpoint(BluetoothSettings::read, BluetoothSettings::update, this),
      _persistence(BluetoothSettings::read, BluetoothSettings::update, this, BLUETOOTH_SETTINGS_FILE),
      _server(nullptr),
      _txCharacteristic(nullptr),
      _rxCharacteristic(nullptr),
      _deviceConnected(false) {
    addUpdateHandler([&](const String& originId) { restartBLE(); }, false);
}

BluetoothService::~BluetoothService() {
    if (_server) {
        BLEDevice::deinit(true);
    }
}

void BluetoothService::begin() {
    _persistence.readFromFS();
    setupBLE();
}

void BluetoothService::setupBLE() {
    const std::string& deviceName = state().settings.deviceName.c_str();
    ESP_LOGI("BluetoothService", "Initializing BLE with device name: %s", deviceName.c_str());

    BLEDevice::init(deviceName);
    _server = BLEDevice::createServer();
    _server->setCallbacks(new ServerCallbacks(this));

    BLEService* service = _server->createService(SERVICE_UUID);

    _txCharacteristic = service->createCharacteristic(CHARACTERISTIC_TX, BLECharacteristic::PROPERTY_NOTIFY);
    _txCharacteristic->addDescriptor(new BLE2902());

    _rxCharacteristic = service->createCharacteristic(CHARACTERISTIC_RX, BLECharacteristic::PROPERTY_WRITE);
    _rxCharacteristic->setCallbacks(new RXCallbacks(this));

    service->start();
    _server->getAdvertising()->start();

    ESP_LOGI("BluetoothService", "BLE UART service started, advertising as %s", deviceName.c_str());
}

void BluetoothService::restartBLE() {
    ESP_LOGI("BluetoothService", "Restarting BLE service due to settings update.");
    if (_server) {
        BLEDevice::deinit(true);
        _server = nullptr;
        _txCharacteristic = nullptr;
        _rxCharacteristic = nullptr;
    }
    setupBLE();
}

void BluetoothService::ServerCallbacks::onConnect(BLEServer* pServer) {
    _service->_deviceConnected = true;
    ESP_LOGI("BluetoothService", "Client connected");
}

void BluetoothService::ServerCallbacks::onDisconnect(BLEServer* pServer) {
    _service->_deviceConnected = false;
    ESP_LOGI("BluetoothService", "Client disconnected");
    pServer->startAdvertising();
    ESP_LOGI("BluetoothService", "Restarting advertising");
}

void BluetoothService::RXCallbacks::onWrite(BLECharacteristic* characteristic) {
    std::string value = characteristic->getValue();
    if (!value.empty()) {
        _service->handleReceivedData(value);
    }
}

void BluetoothService::handleReceivedData(const std::string& data) {
    ESP_LOGI("BluetoothService", "Received: %s", data.c_str());

    std::string response = "ACK: " + data;
    sendData(response);
}

void BluetoothService::sendData(const std::string& data) {
    if (_deviceConnected && _txCharacteristic) {
        _txCharacteristic->setValue(data);
        _txCharacteristic->notify();
        ESP_LOGI("BluetoothService", "Sent: %s", data.c_str());
    } else {
        ESP_LOGW("BluetoothService", "Cannot send data, no device connected or TX characteristic invalid.");
    }
}

void BluetoothService::sendData(uint8_t* data, size_t length) {
    if (_deviceConnected && _txCharacteristic) {
        _txCharacteristic->setValue(data, length);
        _txCharacteristic->notify();
        ESP_LOGI("BluetoothService", "Sent %d bytes", length);
    } else {
        ESP_LOGW("BluetoothService", "Cannot send data, no device connected or TX characteristic invalid.");
    }
}

esp_err_t BluetoothService::getStatus(PsychicRequest* request) {
    PsychicJsonResponse response = PsychicJsonResponse(request, false);
    JsonObject root = response.getRoot();

    return response.send();
}